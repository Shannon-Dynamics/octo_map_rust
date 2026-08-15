//! Reading and writing OctoMap map files.
//!
//! Two formats, both a plain-text header followed by a binary payload:
//!
//! - **`.bt`** — binary occupancy. Two bits per child, so every voxel is only
//!   free or occupied. Written after collapsing the map onto its clamps.
//! - **`.ot`** — full tree. Stores each node's `f32` log-odds verbatim, so a
//!   round trip is lossless.
//!
//! # Byte order
//!
//! The reference writes node values with a raw `memcpy` of the in-memory
//! `float`, so the on-disk layout follows the writing machine's endianness. In
//! practice every OctoMap file in circulation is little-endian, and this
//! implementation reads and writes little-endian on every platform. A file
//! produced by a big-endian build of the C++ library would not interoperate —
//! with this port or with a little-endian build of the reference.

use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::Path;

use crate::error::OctomapError;
use crate::node::{Node, CHILD_COUNT};
use crate::occupancy::{OcTree, OccupancyValue, SensorModel};

/// First line of a `.ot` file.
pub const FULL_FILE_HEADER: &str = "# Octomap OcTree file";

/// First line of a `.bt` file.
pub const BINARY_FILE_HEADER: &str = "# Octomap OcTree binary file";

/// The tree type id this crate reads and writes.
pub const TREE_TYPE: &str = "OcTree";

/// Something went wrong reading or writing a map file.
#[derive(Debug)]
pub enum IoError {
    /// Underlying stream failure.
    Io(io::Error),
    /// The first line was not the expected format marker.
    BadHeader {
        /// The marker the file should have started with.
        expected: &'static str,
    },
    /// The header ended before the `data` keyword.
    UnterminatedHeader,
    /// A required header field was missing.
    MissingField(&'static str),
    /// The header declared a resolution that cannot describe a tree.
    InvalidResolution(f64),
    /// The file holds a tree type this crate does not implement.
    UnsupportedTreeType(String),
    /// The node count in the header disagrees with the payload.
    SizeMismatch {
        /// Count the header declared.
        declared: usize,
        /// Count actually decoded.
        actual: usize,
    },
    /// The payload nested deeper than the tree depth allows.
    ///
    /// A well-formed file cannot do this; a corrupt or hostile one can, and
    /// following it would exhaust the stack.
    TooDeep {
        /// The depth limit that was exceeded.
        limit: u32,
    },
    /// The payload ended mid-node.
    UnexpectedEof,
    /// The header described a tree this crate cannot build.
    Tree(OctomapError),
}

impl std::fmt::Display for IoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "stream error: {e}"),
            Self::BadHeader { expected } => {
                write!(f, "file does not start with {expected:?}")
            }
            Self::UnterminatedHeader => {
                write!(f, "header ended before the \"data\" keyword")
            }
            Self::MissingField(name) => write!(f, "header is missing {name:?}"),
            Self::InvalidResolution(r) => write!(f, "header resolution {r} is not usable"),
            Self::UnsupportedTreeType(id) => {
                write!(
                    f,
                    "tree type {id:?} is not supported, expected {TREE_TYPE:?}"
                )
            }
            Self::SizeMismatch { declared, actual } => write!(
                f,
                "header declares {declared} nodes but the payload holds {actual}"
            ),
            Self::TooDeep { limit } => {
                write!(f, "payload nests deeper than the tree depth of {limit}")
            }
            Self::UnexpectedEof => write!(f, "payload ended mid-node"),
            Self::Tree(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for IoError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            Self::Tree(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for IoError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

impl From<OctomapError> for IoError {
    fn from(e: OctomapError) -> Self {
        Self::Tree(e)
    }
}

/// Convenience alias for results in this module.
pub type Result<T> = core::result::Result<T, IoError>;

/// Formats a `f64` the way C++ `operator<<` does at default precision.
///
/// The reference writes the resolution with an unconfigured stream, which is
/// `%g` at six significant digits — `0.1` becomes `"0.1"`, not `"0.100000"`.
/// Matching this matters: a byte-comparison against a C++-written file fails on
/// the header otherwise. It is also lossy for resolutions needing more than six
/// digits, which is the reference's behavior and not something this port
/// silently improves.
fn format_cpp_double(value: f64) -> String {
    const PRECISION: usize = 6;

    if value == 0.0 {
        return "0".to_string();
    }
    if value.is_nan() {
        return "nan".to_string();
    }
    if value.is_infinite() {
        return if value > 0.0 { "inf" } else { "-inf" }.to_string();
    }

    let scientific = format!("{:.*e}", PRECISION - 1, value);
    let (mantissa, exponent) = scientific
        .split_once('e')
        .expect("Rust always emits an exponent for {:e}");
    let exponent: i32 = exponent.parse().expect("exponent is an integer");

    let trim = |s: &str| -> String {
        if !s.contains('.') {
            return s.to_string();
        }
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    };

    if exponent < -4 || exponent >= PRECISION as i32 {
        format!(
            "{}e{}{:02}",
            trim(mantissa),
            if exponent < 0 { '-' } else { '+' },
            exponent.abs()
        )
    } else {
        let decimals = (PRECISION as i32 - 1 - exponent).max(0) as usize;
        trim(&format!("{value:.decimals$}"))
    }
}

/// Fields parsed out of a map-file header.
struct Header {
    id: String,
    size: usize,
    resolution: f64,
}

/// Reads bytes until `\n`, returning the line without it.
///
/// Returns `None` at end of input with nothing read.
fn read_line<R: Read>(reader: &mut R) -> Result<Option<String>> {
    let mut line = Vec::new();
    let mut byte = [0u8; 1];
    loop {
        match reader.read(&mut byte)? {
            0 => break,
            _ => {
                if byte[0] == b'\n' {
                    let mut s = String::from_utf8_lossy(&line).into_owned();
                    if s.ends_with('\r') {
                        s.pop();
                    }
                    return Ok(Some(s));
                }
                line.push(byte[0]);
            }
        }
    }
    if line.is_empty() {
        Ok(None)
    } else {
        Ok(Some(String::from_utf8_lossy(&line).into_owned()))
    }
}

/// Parses the `id` / `size` / `res` / `data` block.
///
/// Mirrors `AbstractOcTree::readHeader`: unknown keywords are skipped with a
/// warning rather than rejected, so a file carrying extra metadata still loads.
fn read_header<R: Read>(reader: &mut R) -> Result<Header> {
    let mut id: Option<String> = None;
    let mut size: Option<usize> = None;
    let mut resolution: Option<f64> = None;

    loop {
        let Some(line) = read_line(reader)? else {
            return Err(IoError::UnterminatedHeader);
        };
        let mut tokens = line.split_whitespace();
        let Some(keyword) = tokens.next() else {
            continue;
        };

        match keyword {
            "data" => break,
            "id" => id = tokens.next().map(str::to_string),
            "size" => size = tokens.next().and_then(|t| t.parse().ok()),
            "res" => resolution = tokens.next().and_then(|t| t.parse().ok()),
            // Comments and anything unrecognised: skip the whole line.
            _ => {}
        }
    }

    let id = id
        .filter(|s| !s.is_empty())
        .ok_or(IoError::MissingField("id"))?;
    let size = size.ok_or(IoError::MissingField("size"))?;
    let resolution = resolution.ok_or(IoError::MissingField("res"))?;

    if !resolution.is_finite() || resolution <= 0.0 {
        return Err(IoError::InvalidResolution(resolution));
    }
    if id != TREE_TYPE {
        return Err(IoError::UnsupportedTreeType(id));
    }

    Ok(Header {
        id,
        size,
        resolution,
    })
}

fn write_header<W: Write>(
    writer: &mut W,
    marker: &str,
    size: usize,
    resolution: f64,
) -> Result<()> {
    write!(
        writer,
        "{marker}\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n"
    )?;
    writeln!(writer, "id {TREE_TYPE}")?;
    writeln!(writer, "size {size}")?;
    writeln!(writer, "res {}", format_cpp_double(resolution))?;
    writeln!(writer, "data")?;
    Ok(())
}

fn read_exact<R: Read>(reader: &mut R, buf: &mut [u8]) -> Result<()> {
    match reader.read_exact(buf) {
        Ok(()) => Ok(()),
        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => Err(IoError::UnexpectedEof),
        Err(e) => Err(IoError::Io(e)),
    }
}

// ---- .ot, the full tree ----------------------------------------------------

fn write_full_node<W: Write>(node: &Node<OccupancyValue>, writer: &mut W) -> Result<()> {
    writer.write_all(&node.value().log_odds.to_bits().to_le_bytes())?;

    let mut mask = 0u8;
    for i in 0..CHILD_COUNT as u8 {
        if node.child_exists(i) {
            mask |= 1 << i;
        }
    }
    writer.write_all(&[mask])?;

    for i in 0..CHILD_COUNT as u8 {
        if let Some(child) = node.child(i) {
            write_full_node(child, writer)?;
        }
    }
    Ok(())
}

fn read_full_node<R: Read>(reader: &mut R, depth: u32, limit: u32) -> Result<Node<OccupancyValue>> {
    if depth > limit {
        return Err(IoError::TooDeep { limit });
    }

    let mut value = [0u8; 4];
    read_exact(reader, &mut value)?;
    let mut node = Node::new(OccupancyValue::new(f32::from_bits(u32::from_le_bytes(
        value,
    ))));

    let mut mask = [0u8; 1];
    read_exact(reader, &mut mask)?;

    for i in 0..CHILD_COUNT as u8 {
        if mask[0] & (1 << i) != 0 {
            let child = read_full_node(reader, depth + 1, limit)?;
            node.create_child(i, OccupancyValue::default());
            *node.child_mut(i).expect("just created") = child;
        }
    }
    Ok(node)
}

/// Writes the map to a `.ot` stream, preserving every node's exact log-odds.
pub fn write_full<W: Write>(tree: &OcTree, writer: &mut W) -> Result<()> {
    write_header(writer, FULL_FILE_HEADER, tree.len(), tree.resolution())?;
    if let Some(root) = tree.core().root() {
        write_full_node(root, writer)?;
    }
    Ok(())
}

/// Reads a `.ot` stream.
pub fn read_full<R: Read>(reader: &mut R) -> Result<OcTree> {
    let Some(first) = read_line(reader)? else {
        return Err(IoError::BadHeader {
            expected: FULL_FILE_HEADER,
        });
    };
    if !first.starts_with(FULL_FILE_HEADER) {
        return Err(IoError::BadHeader {
            expected: FULL_FILE_HEADER,
        });
    }

    let header = read_header(reader)?;
    debug_assert_eq!(header.id, TREE_TYPE);

    let mut tree = OcTree::new(header.resolution)?;
    if header.size > 0 {
        let limit = tree.geometry().tree_depth();
        let root = read_full_node(reader, 0, limit)?;
        tree.core_mut().set_root(Some(root));
    }

    if tree.len() != header.size {
        return Err(IoError::SizeMismatch {
            declared: header.size,
            actual: tree.len(),
        });
    }
    Ok(tree)
}

// ---- .bt, binary occupancy -------------------------------------------------

/// The two-bit code a child contributes to its parent's descriptor.
mod child_code {
    /// No child at this index.
    pub const UNKNOWN: u8 = 0b00;
    /// Childless and below the occupancy threshold.
    pub const FREE: u8 = 0b01;
    /// Childless and at or above the occupancy threshold.
    pub const OCCUPIED: u8 = 0b10;
    /// Has children of its own; its descriptor follows.
    pub const INNER: u8 = 0b11;
}

/// Packs four children into one descriptor byte, low index in the low bits.
///
/// The reference builds this with `std::bitset<8>`, where index 0 is the least
/// significant bit — hence the shift by `i * 2`. Note the bit order within each
/// pair: free is `bit[2i] = 1`, which is `0b01` when read as a little-end pair.
fn pack_descriptor(node: &Node<OccupancyValue>, base: u8, sensor: &SensorModel) -> u8 {
    let mut byte = 0u8;
    for i in 0..4u8 {
        let code = match node.child(base + i) {
            None => child_code::UNKNOWN,
            Some(child) => {
                if child.has_children() {
                    child_code::INNER
                } else if sensor.is_occupied(*child.value()) {
                    child_code::OCCUPIED
                } else {
                    child_code::FREE
                }
            }
        };
        byte |= code << (i * 2);
    }
    byte
}

fn write_binary_node<W: Write>(
    node: &Node<OccupancyValue>,
    sensor: &SensorModel,
    writer: &mut W,
) -> Result<()> {
    writer.write_all(&[
        pack_descriptor(node, 0, sensor),
        pack_descriptor(node, 4, sensor),
    ])?;

    for i in 0..CHILD_COUNT as u8 {
        if let Some(child) = node.child(i) {
            if child.has_children() {
                write_binary_node(child, sensor, writer)?;
            }
        }
    }
    Ok(())
}

fn read_binary_node<R: Read>(
    reader: &mut R,
    sensor: &SensorModel,
    depth: u32,
    limit: u32,
) -> Result<Node<OccupancyValue>> {
    if depth > limit {
        return Err(IoError::TooDeep { limit });
    }

    let mut descriptors = [0u8; 2];
    read_exact(reader, &mut descriptors)?;

    // Inner nodes are written as occupied and corrected from their children
    // once those are read.
    let mut node = Node::new(OccupancyValue::new(sensor.clamping_thres_max()));
    let mut inner = [false; CHILD_COUNT];

    for (half, descriptor) in descriptors.iter().enumerate() {
        for i in 0..4u8 {
            let index = half as u8 * 4 + i;
            match (descriptor >> (i * 2)) & 0b11 {
                child_code::UNKNOWN => {}
                child_code::FREE => {
                    node.create_child(index, OccupancyValue::new(sensor.clamping_thres_min()));
                }
                child_code::OCCUPIED => {
                    node.create_child(index, OccupancyValue::new(sensor.clamping_thres_max()));
                }
                _ => {
                    node.create_child(index, OccupancyValue::default());
                    inner[index as usize] = true;
                }
            }
        }
    }

    for i in 0..CHILD_COUNT as u8 {
        if inner[i as usize] {
            let child = read_binary_node(reader, sensor, depth + 1, limit)?;
            // Inner nodes carry the maximum of their children, conservatively.
            let value = crate::occupancy::max_child_log_odds(&child);
            *node.child_mut(i).expect("marked inner") = child;
            node.child_mut(i)
                .expect("marked inner")
                .set_value(OccupancyValue::new(value));
        }
    }
    Ok(node)
}

/// Writes the map to a `.bt` stream **without** modifying it.
///
/// The occupancy of each voxel is reduced to one bit, so anything between the
/// clamps is rounded to whichever side of the threshold it falls on. Use
/// [`write_binary`] to collapse and prune the map first, which is what the
/// reference's `writeBinary` does and what makes `.bt` files compact.
pub fn write_binary_const<W: Write>(tree: &OcTree, writer: &mut W) -> Result<()> {
    write_header(writer, BINARY_FILE_HEADER, tree.len(), tree.resolution())?;
    if let Some(root) = tree.core().root() {
        write_binary_node(root, tree.sensor(), writer)?;
    }
    Ok(())
}

/// Collapses the map onto its clamps, prunes it, then writes a `.bt` stream.
///
/// **This modifies the map**, exactly as the reference's `writeBinary` does.
/// The collapse is what lets the pruning merge whole regions, which is the
/// point of the format. Use [`write_binary_const`] to leave the map alone.
pub fn write_binary<W: Write>(tree: &mut OcTree, writer: &mut W) -> Result<()> {
    tree.to_max_likelihood();
    tree.prune();
    write_binary_const(tree, writer)
}

/// Reads a `.bt` stream.
///
/// The legacy headerless format the reference still accepts is **not**
/// supported; those files predate the current header and the reference itself
/// tells you to convert them.
pub fn read_binary<R: Read>(reader: &mut R) -> Result<OcTree> {
    let Some(first) = read_line(reader)? else {
        return Err(IoError::BadHeader {
            expected: BINARY_FILE_HEADER,
        });
    };
    if !first.starts_with(BINARY_FILE_HEADER) {
        return Err(IoError::BadHeader {
            expected: BINARY_FILE_HEADER,
        });
    }

    let header = read_header(reader)?;
    let mut tree = OcTree::new(header.resolution)?;

    if header.size > 0 {
        let sensor = *tree.sensor();
        let limit = tree.geometry().tree_depth();
        let root = read_binary_node(reader, &sensor, 0, limit)?;
        tree.core_mut().set_root(Some(root));
    }

    if tree.len() != header.size {
        return Err(IoError::SizeMismatch {
            declared: header.size,
            actual: tree.len(),
        });
    }
    Ok(tree)
}

// ---- headerless payloads ---------------------------------------------------

/// Reads one byte, or `None` if the stream is already at its end.
///
/// The headerless readers need to tell "empty map" apart from "truncated
/// payload", and the only difference is whether the very first read returns
/// anything.
fn read_optional_byte<R: Read>(reader: &mut R) -> Result<Option<u8>> {
    let mut buf = [0u8; 1];
    loop {
        match reader.read(&mut buf) {
            Ok(0) => return Ok(None),
            Ok(_) => return Ok(Some(buf[0])),
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(IoError::Io(e)),
        }
    }
}

/// Writes the `.bt` node payload **without** the text header.
///
/// This is the reference's `writeBinaryData`, and it is what
/// `octomap_msgs::binaryMapToMsg` puts in an `octomap_msgs/Octomap` message —
/// the resolution and tree type travel in their own message fields, so the
/// header would be redundant. Nothing is written for an empty map.
///
/// The map is **not** collapsed or pruned first, matching the reference. Call
/// [`OcTree::to_max_likelihood`] and [`OcTree::prune`] beforehand for the
/// compact output the format is capable of.
pub fn write_binary_data<W: Write>(tree: &OcTree, writer: &mut W) -> Result<()> {
    if let Some(root) = tree.core().root() {
        write_binary_node(root, tree.sensor(), writer)?;
    }
    Ok(())
}

/// Reads a `.bt` node payload that carries no header, at a known resolution.
///
/// The reference's `readBinaryData`. Since the payload declares neither the
/// resolution nor the node count, the caller supplies the former and the latter
/// cannot be cross-checked. An empty stream yields an empty map.
pub fn read_binary_data<R: Read>(reader: &mut R, resolution: f64) -> Result<OcTree> {
    let mut tree = OcTree::new(resolution)?;
    let Some(first) = read_optional_byte(reader)? else {
        return Ok(tree);
    };

    let head = [first];
    let mut stream = (&head[..]).chain(reader);
    let sensor = *tree.sensor();
    let limit = tree.geometry().tree_depth();
    let root = read_binary_node(&mut stream, &sensor, 0, limit)?;
    tree.core_mut().set_root(Some(root));
    Ok(tree)
}

/// Writes the `.ot` node payload **without** the text header.
///
/// The reference's `writeData`, used by `octomap_msgs::fullMapToMsg`. Every
/// node's exact log-odds is preserved. Nothing is written for an empty map.
pub fn write_full_data<W: Write>(tree: &OcTree, writer: &mut W) -> Result<()> {
    if let Some(root) = tree.core().root() {
        write_full_node(root, writer)?;
    }
    Ok(())
}

/// Reads a `.ot` node payload that carries no header, at a known resolution.
///
/// The reference's `readData`. An empty stream yields an empty map.
pub fn read_full_data<R: Read>(reader: &mut R, resolution: f64) -> Result<OcTree> {
    let mut tree = OcTree::new(resolution)?;
    let Some(first) = read_optional_byte(reader)? else {
        return Ok(tree);
    };

    let head = [first];
    let mut stream = (&head[..]).chain(reader);
    let limit = tree.geometry().tree_depth();
    let root = read_full_node(&mut stream, 0, limit)?;
    tree.core_mut().set_root(Some(root));
    Ok(tree)
}

// ---- file convenience ------------------------------------------------------

/// Writes a `.bt` file, collapsing and pruning the map first.
pub fn write_binary_file<P: AsRef<Path>>(tree: &mut OcTree, path: P) -> Result<()> {
    let mut writer = BufWriter::new(File::create(path)?);
    write_binary(tree, &mut writer)?;
    writer.flush()?;
    Ok(())
}

/// Reads a `.bt` file.
pub fn read_binary_file<P: AsRef<Path>>(path: P) -> Result<OcTree> {
    read_binary(&mut BufReader::new(File::open(path)?))
}

/// Writes a `.ot` file.
pub fn write_full_file<P: AsRef<Path>>(tree: &OcTree, path: P) -> Result<()> {
    let mut writer = BufWriter::new(File::create(path)?);
    write_full(tree, &mut writer)?;
    writer.flush()?;
    Ok(())
}

/// Reads a `.ot` file.
pub fn read_full_file<P: AsRef<Path>>(path: P) -> Result<OcTree> {
    read_full(&mut BufReader::new(File::open(path)?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::point::Point3;

    fn scene() -> OcTree {
        let mut tree = OcTree::new(0.1).unwrap();
        for i in 0..12u16 {
            tree.update_node(
                crate::key::OcTreeKey::new(32768 + i, 32768, 32768),
                i % 3 != 0,
            );
        }
        tree.update_node_at(Point3::new(1.05, 1.05, 1.05), true);
        tree.update_node_at(Point3::new(-2.05, 0.35, 0.45), false);
        tree
    }

    #[test]
    fn cpp_double_formatting_matches_six_significant_digits() {
        assert_eq!(format_cpp_double(0.1), "0.1");
        assert_eq!(format_cpp_double(0.05), "0.05");
        assert_eq!(format_cpp_double(0.02), "0.02");
        assert_eq!(format_cpp_double(1.0), "1");
        assert_eq!(format_cpp_double(0.0), "0");
        assert_eq!(format_cpp_double(32768.0), "32768");
        // Six significant digits, then trailing zeros stripped.
        assert_eq!(format_cpp_double(0.123_456_789), "0.123457");
        assert_eq!(format_cpp_double(1_234_567.0), "1.23457e+06");
        assert_eq!(format_cpp_double(0.000_012_345_6), "1.23456e-05");
    }

    #[test]
    fn full_round_trip_preserves_every_log_odds_value() {
        let tree = scene();
        let mut buffer = Vec::new();
        write_full(&tree, &mut buffer).unwrap();

        let back = read_full(&mut buffer.as_slice()).unwrap();
        assert_eq!(back.len(), tree.len());
        assert_eq!(back.resolution(), tree.resolution());

        let original: Vec<_> = tree
            .iter_leaves()
            .map(|v| (v.key(), v.depth(), v.value().log_odds.to_bits()))
            .collect();
        let restored: Vec<_> = back
            .iter_leaves()
            .map(|v| (v.key(), v.depth(), v.value().log_odds.to_bits()))
            .collect();
        assert_eq!(restored, original, ".ot must be lossless");
    }

    #[test]
    fn binary_round_trip_preserves_occupancy() {
        let mut tree = scene();
        let mut buffer = Vec::new();
        write_binary(&mut tree, &mut buffer).unwrap();

        let back = read_binary(&mut buffer.as_slice()).unwrap();
        assert_eq!(back.len(), tree.len());

        for leaf in tree.iter_leaves() {
            let want = tree.sensor().is_occupied(*leaf.value());
            assert_eq!(
                back.is_occupied(leaf.key()),
                Some(want),
                "occupancy lost for {:?}",
                leaf.key()
            );
        }
    }

    #[test]
    fn write_binary_collapses_the_map_but_write_binary_const_does_not() {
        let mut a = scene();
        let before: Vec<_> = a.iter_leaves().map(|v| v.value().log_odds).collect();

        let mut buffer = Vec::new();
        write_binary_const(&a, &mut buffer).unwrap();
        let after_const: Vec<_> = a.iter_leaves().map(|v| v.value().log_odds).collect();
        assert_eq!(after_const, before, "the const writer must not mutate");

        buffer.clear();
        write_binary(&mut a, &mut buffer).unwrap();
        let after: Vec<_> = a.iter_leaves().map(|v| v.value().log_odds).collect();
        assert_ne!(after, before, "writeBinary is documented to collapse");
        for l in after {
            assert!(l == a.sensor().clamping_thres_max() || l == a.sensor().clamping_thres_min());
        }
    }

    #[test]
    fn an_empty_tree_round_trips() {
        let tree = OcTree::new(0.25).unwrap();
        let mut buffer = Vec::new();
        write_full(&tree, &mut buffer).unwrap();

        let back = read_full(&mut buffer.as_slice()).unwrap();
        assert!(back.is_empty());
        assert_eq!(back.resolution(), 0.25);
    }

    #[test]
    fn a_headerless_payload_is_the_tail_of_the_file_with_a_header() {
        let tree = scene();

        let mut with_header = Vec::new();
        write_full(&tree, &mut with_header).unwrap();
        let mut without = Vec::new();
        write_full_data(&tree, &mut without).unwrap();

        assert!(with_header.ends_with(&without));
        assert_eq!(
            with_header.len() - without.len(),
            with_header
                .windows(5)
                .position(|w| w == b"data\n")
                .expect("header ends with the data keyword")
                + 5
        );
    }

    #[test]
    fn full_data_round_trips_without_a_header() {
        let tree = scene();
        let mut buffer = Vec::new();
        write_full_data(&tree, &mut buffer).unwrap();

        let back = read_full_data(&mut buffer.as_slice(), tree.resolution()).unwrap();
        assert_eq!(back.len(), tree.len());

        let original: Vec<_> = tree
            .iter_leaves()
            .map(|v| (v.key(), v.depth(), v.value().log_odds.to_bits()))
            .collect();
        let restored: Vec<_> = back
            .iter_leaves()
            .map(|v| (v.key(), v.depth(), v.value().log_odds.to_bits()))
            .collect();
        assert_eq!(restored, original);
    }

    #[test]
    fn binary_data_round_trips_without_a_header() {
        let mut tree = scene();
        tree.to_max_likelihood();
        tree.prune();

        let mut buffer = Vec::new();
        write_binary_data(&tree, &mut buffer).unwrap();

        let back = read_binary_data(&mut buffer.as_slice(), tree.resolution()).unwrap();
        assert_eq!(back.len(), tree.len());
        for leaf in tree.iter_leaves() {
            let want = tree.sensor().is_occupied(*leaf.value());
            assert_eq!(back.is_occupied(leaf.key()), Some(want));
        }
    }

    #[test]
    fn an_empty_payload_decodes_to_an_empty_map() {
        // The reference writes nothing at all for a map with no root, and
        // octomap_msgs guards the decode on a non-empty `data` field. Both
        // readers have to survive the zero-byte case on their own.
        let empty: &[u8] = &[];
        let full = read_full_data(&mut { empty }, 0.05).unwrap();
        assert!(full.is_empty());
        assert_eq!(full.resolution(), 0.05);

        let binary = read_binary_data(&mut { empty }, 0.05).unwrap();
        assert!(binary.is_empty());
    }

    #[test]
    fn a_truncated_payload_is_rejected_rather_than_half_decoded() {
        let tree = scene();
        let mut buffer = Vec::new();
        write_full_data(&tree, &mut buffer).unwrap();
        buffer.truncate(buffer.len() - 1);

        assert!(matches!(
            read_full_data(&mut buffer.as_slice(), tree.resolution()),
            Err(IoError::UnexpectedEof)
        ));
    }

    #[test]
    fn the_header_is_written_in_the_reference_layout() {
        let tree = OcTree::new(0.1).unwrap();
        let mut buffer = Vec::new();
        write_full(&tree, &mut buffer).unwrap();

        let text = String::from_utf8(buffer).unwrap();
        let lines: Vec<_> = text.lines().collect();
        assert_eq!(lines[0], FULL_FILE_HEADER);
        assert_eq!(lines[3], "id OcTree");
        assert_eq!(lines[4], "size 0");
        assert_eq!(lines[5], "res 0.1");
        assert_eq!(lines[6], "data");
    }

    #[test]
    fn a_wrong_first_line_is_rejected() {
        let bad = b"# Not an octomap file\nid OcTree\nsize 0\nres 0.1\ndata\n";
        assert!(matches!(
            read_full(&mut bad.as_slice()),
            Err(IoError::BadHeader { .. })
        ));
        assert!(matches!(
            read_binary(&mut bad.as_slice()),
            Err(IoError::BadHeader { .. })
        ));
    }

    #[test]
    fn a_binary_file_is_not_accepted_as_a_full_file() {
        let mut tree = scene();
        let mut buffer = Vec::new();
        write_binary(&mut tree, &mut buffer).unwrap();
        assert!(matches!(
            read_full(&mut buffer.as_slice()),
            Err(IoError::BadHeader { .. })
        ));
    }

    #[test]
    fn a_header_without_data_is_rejected() {
        let bad = format!("{FULL_FILE_HEADER}\nid OcTree\nsize 1\nres 0.1\n");
        assert!(matches!(
            read_full(&mut bad.as_bytes()),
            Err(IoError::UnterminatedHeader)
        ));
    }

    #[test]
    fn a_missing_field_is_reported_rather_than_guessed() {
        let no_res = format!("{FULL_FILE_HEADER}\nid OcTree\nsize 0\ndata\n");
        assert!(matches!(
            read_full(&mut no_res.as_bytes()),
            Err(IoError::MissingField("res"))
        ));

        let no_id = format!("{FULL_FILE_HEADER}\nsize 0\nres 0.1\ndata\n");
        assert!(matches!(
            read_full(&mut no_id.as_bytes()),
            Err(IoError::MissingField("id"))
        ));
    }

    #[test]
    fn a_non_positive_resolution_is_rejected() {
        let bad = format!("{FULL_FILE_HEADER}\nid OcTree\nsize 0\nres 0\ndata\n");
        assert!(matches!(
            read_full(&mut bad.as_bytes()),
            Err(IoError::InvalidResolution(_))
        ));
    }

    #[test]
    fn an_unknown_tree_type_is_rejected() {
        let bad = format!("{FULL_FILE_HEADER}\nid ColorOcTree\nsize 0\nres 0.1\ndata\n");
        assert!(matches!(
            read_full(&mut bad.as_bytes()),
            Err(IoError::UnsupportedTreeType(_))
        ));
    }

    #[test]
    fn unknown_header_keywords_are_skipped_like_the_reference() {
        let extra = format!(
            "{FULL_FILE_HEADER}\n# a comment\nsomething unexpected\nid OcTree\nsize 0\nres 0.1\ndata\n"
        );
        let tree = read_full(&mut extra.as_bytes()).unwrap();
        assert_eq!(tree.resolution(), 0.1);
    }

    #[test]
    fn a_truncated_payload_is_reported_rather_than_panicking() {
        let tree = scene();
        let mut buffer = Vec::new();
        write_full(&tree, &mut buffer).unwrap();
        buffer.truncate(buffer.len() - 5);

        assert!(matches!(
            read_full(&mut buffer.as_slice()),
            Err(IoError::UnexpectedEof)
        ));
    }

    #[test]
    fn a_size_mismatch_is_reported() {
        let tree = scene();
        let mut buffer = Vec::new();
        write_full(&tree, &mut buffer).unwrap();

        // Patch the ASCII header in place. Going through a String would mangle
        // the binary payload, and the read would then fail for the wrong reason.
        let needle = format!("size {}\n", tree.len());
        let at = buffer
            .windows(needle.len())
            .position(|w| w == needle.as_bytes())
            .expect("header carries a size field");
        let mut corrupted = buffer[..at].to_vec();
        corrupted.extend_from_slice(b"size 999999\n");
        corrupted.extend_from_slice(&buffer[at + needle.len()..]);

        assert!(matches!(
            read_full(&mut corrupted.as_slice()),
            Err(IoError::SizeMismatch { .. })
        ));
    }

    #[test]
    fn an_endlessly_nesting_payload_is_refused_instead_of_overflowing_the_stack() {
        // Every node claims child 0 has children, forever. The reference would
        // recurse until the stack gives out.
        let mut data =
            format!("{BINARY_FILE_HEADER}\nid OcTree\nsize 1\nres 0.1\ndata\n").into_bytes();
        for _ in 0..2000 {
            data.extend_from_slice(&[0b0000_0011, 0]);
        }
        assert!(matches!(
            read_binary(&mut data.as_slice()),
            Err(IoError::TooDeep { .. })
        ));
    }

    #[test]
    fn random_bytes_after_a_valid_header_never_panic() {
        // Not a correctness check — a crash check. Every outcome must be an
        // error or a tree, never a panic.
        let mut seed = 0x2545_F491_4F6C_DD1Du64;
        for _ in 0..200 {
            let mut payload = Vec::new();
            for _ in 0..64 {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                payload.push((seed >> 24) as u8);
            }
            let mut data =
                format!("{BINARY_FILE_HEADER}\nid OcTree\nsize 5\nres 0.1\ndata\n").into_bytes();
            data.extend_from_slice(&payload);
            let _ = read_binary(&mut data.as_slice());

            let mut data =
                format!("{FULL_FILE_HEADER}\nid OcTree\nsize 5\nres 0.1\ndata\n").into_bytes();
            data.extend_from_slice(&payload);
            let _ = read_full(&mut data.as_slice());
        }
    }

    #[test]
    fn descriptor_packing_uses_the_reference_bit_order() {
        let sensor = SensorModel::default();
        let mut node = Node::new(OccupancyValue::default());

        // Child 0 free, child 1 occupied, child 2 absent, child 3 inner.
        node.create_child(0, OccupancyValue::new(sensor.clamping_thres_min()));
        node.create_child(1, OccupancyValue::new(sensor.clamping_thres_max()));
        node.create_child(3, OccupancyValue::default());
        node.child_mut(3).unwrap().expand();

        let byte = pack_descriptor(&node, 0, &sensor);
        // free=01 at bits 0-1, occupied=10 at bits 2-3, unknown=00, inner=11.
        assert_eq!(byte, 0b1100_1001, "got {byte:08b}");
    }
}
