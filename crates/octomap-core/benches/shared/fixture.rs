//! The benchmark scene, and the only place it is generated.
//!
//! Both the Rust benchmark and `scripts/bench_cpp.cpp` measure the *same*
//! points, read from the same file. Writing a second generator for the C++
//! side — one that is only *supposed* to produce the same points — is exactly
//! the failure mode this project's audit already caught three times (the
//! reciprocal trap, the one-ULP `Quaternion::norm` bug, the
//! `computeRayKeys`/`castRay` narrowing difference). So there is one
//! generator, here, and a plain-text file everything else reads.
//!
//! The file carries a checksum over every coordinate's raw `f32` bits. Both
//! readers recompute it and print it; if the two ever disagree, the benchmark
//! comparison is invalid and says so instead of quietly measuring two
//! different scenes.

#![allow(dead_code)]

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use octomap_core::Point3;

/// Format marker, so a stale fixture from an older layout is rejected rather
/// than misparsed.
pub const FORMAT_TAG: &str = "octomap-bench-fixture 1";

/// Voxel size the benchmarks build their map at, in meters.
pub const RESOLUTION: f64 = 0.1;

/// Sensor position for every measurement.
pub const SENSOR: Point3 = Point3::new(0.05, 0.05, 0.05);

/// Grid side of the synthetic depth frame. 224² = 50176 returns.
pub const SCAN_SIDE: usize = 224;

/// Number of random occupancy lookups.
pub const QUERY_COUNT: usize = 10_000;

/// Number of random ray-cast directions.
pub const DIRECTION_COUNT: usize = 1_000;

/// Everything the benchmarks need, once.
pub struct Scene {
    /// Voxel size, in meters.
    pub resolution: f64,
    /// Sensor position.
    pub sensor: Point3,
    /// Synthetic depth-camera returns.
    pub scan: Vec<Point3>,
    /// Points to look up after the scan is integrated.
    pub queries: Vec<Point3>,
    /// Directions to cast rays along.
    pub directions: Vec<Point3>,
}

impl Scene {
    /// Wrapping sum over the raw `f32` bits of every coordinate.
    ///
    /// Cheap, order-sensitive, and enough to catch a parser that dropped a
    /// point or rounded one differently.
    pub fn checksum(&self) -> u64 {
        let mut sum = 0u64;
        for group in [&self.scan, &self.queries, &self.directions] {
            for p in group {
                sum = sum.wrapping_add(u64::from(p.x.to_bits()));
                sum = sum.wrapping_add(u64::from(p.y.to_bits()));
                sum = sum.wrapping_add(u64::from(p.z.to_bits()));
            }
        }
        sum
    }
}

/// Deterministic PRNG, so every run benchmarks the same scene.
///
/// xorshift64* — enough to scatter points, and it keeps the crate free of a
/// `rand` dependency for what is only test scaffolding.
struct Rng(u64);

impl Rng {
    fn new() -> Self {
        Self(0x2545_F491_4F6C_DD1D)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    /// Uniform in `[lo, hi)`. 24 bits is the `f32` mantissa, so this covers the
    /// representable range without clumping.
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        let unit = (self.next_u64() >> 40) as f32 / (1u32 << 24) as f32;
        lo + unit * (hi - lo)
    }
}

/// Builds the scene from scratch. The single source of truth for what gets
/// measured.
pub fn generate() -> Scene {
    let mut rng = Rng::new();

    // A depth-camera-shaped frame: a grid of returns on a wall about five
    // metres out, with enough depth variation that the rays differ in length.
    let mut scan = Vec::with_capacity(SCAN_SIDE * SCAN_SIDE);
    for iy in 0..SCAN_SIDE {
        for iz in 0..SCAN_SIDE {
            let fy = (iy as f32 / SCAN_SIDE as f32) * 2.0 - 1.0;
            let fz = (iz as f32 / SCAN_SIDE as f32) * 2.0 - 1.0;
            let depth = 5.0 + rng.range(-0.5, 0.5);
            scan.push(Point3::new(
                SENSOR.x + depth,
                SENSOR.y + fy * depth * 0.5,
                SENSOR.z + fz * depth * 0.5,
            ));
        }
    }

    // Scattered across the volume the scan touched, so lookups hit a realistic
    // mix of occupied, free and unknown voxels.
    let mut qrng = Rng::new();
    let queries = (0..QUERY_COUNT)
        .map(|_| {
            Point3::new(
                qrng.range(-1.0, 7.0),
                qrng.range(-3.0, 3.0),
                qrng.range(-3.0, 3.0),
            )
        })
        .collect();

    let mut drng = Rng::new();
    let directions = (0..DIRECTION_COUNT)
        .map(|_| Point3::new(1.0, drng.range(-0.5, 0.5), drng.range(-0.5, 0.5)))
        .collect();

    Scene {
        resolution: RESOLUTION,
        sensor: SENSOR,
        scan,
        queries,
        directions,
    }
}

/// Serializes the scene as plain text.
///
/// Rust's `f32` Display emits the shortest decimal that round-trips, and C++
/// `strtof` is correctly rounded, so both sides recover bit-identical values
/// from the same line.
pub fn serialize(scene: &Scene) -> String {
    let mut out = String::with_capacity(scene.scan.len() * 32);
    let _ = writeln!(out, "{FORMAT_TAG}");
    let _ = writeln!(
        out,
        "# generated by: cargo run --release --example dump_bench_fixture"
    );
    let _ = writeln!(out, "resolution {}", scene.resolution);
    let _ = writeln!(
        out,
        "sensor {} {} {}",
        scene.sensor.x, scene.sensor.y, scene.sensor.z
    );

    for (label, group) in [
        ("scan", &scene.scan),
        ("queries", &scene.queries),
        ("directions", &scene.directions),
    ] {
        let _ = writeln!(out, "{label} {}", group.len());
        for p in group {
            let _ = writeln!(out, "{} {} {}", p.x, p.y, p.z);
        }
    }

    let _ = writeln!(out, "checksum {}", scene.checksum());
    out
}

/// Parses a scene written by [`serialize`].
pub fn deserialize(text: &str) -> Result<Scene, String> {
    let mut lines = text
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'));

    let tag = lines.next().ok_or("fixture is empty")?;
    if tag != FORMAT_TAG {
        return Err(format!("expected {FORMAT_TAG:?}, found {tag:?}"));
    }

    let scalar = |line: Option<&str>, key: &str| -> Result<f64, String> {
        let line = line.ok_or_else(|| format!("missing {key}"))?;
        let rest = line
            .strip_prefix(key)
            .ok_or_else(|| format!("expected {key}, found {line:?}"))?;
        rest.trim().parse().map_err(|e| format!("bad {key}: {e}"))
    };

    let resolution = scalar(lines.next(), "resolution")?;

    let sensor_line = lines.next().ok_or("missing sensor")?;
    let sensor = parse_point(
        sensor_line
            .strip_prefix("sensor")
            .ok_or_else(|| format!("expected sensor, found {sensor_line:?}"))?,
    )?;

    let mut group = |label: &str| -> Result<Vec<Point3>, String> {
        let header = lines.next().ok_or_else(|| format!("missing {label}"))?;
        let count: usize = header
            .strip_prefix(label)
            .ok_or_else(|| format!("expected {label}, found {header:?}"))?
            .trim()
            .parse()
            .map_err(|e| format!("bad {label} count: {e}"))?;

        let mut points = Vec::with_capacity(count);
        for i in 0..count {
            let line = lines
                .next()
                .ok_or_else(|| format!("{label}: ran out at point {i} of {count}"))?;
            points.push(parse_point(line)?);
        }
        Ok(points)
    };

    let scan = group("scan")?;
    let queries = group("queries")?;
    let directions = group("directions")?;

    let scene = Scene {
        resolution,
        sensor,
        scan,
        queries,
        directions,
    };

    let declared = scalar(lines.next(), "checksum")? as u64;
    let actual = scene.checksum();
    if declared != actual {
        return Err(format!(
            "checksum mismatch: file says {declared}, points hash to {actual}"
        ));
    }
    Ok(scene)
}

fn parse_point(line: &str) -> Result<Point3, String> {
    let mut it = line.split_whitespace();
    let mut next = |axis: &str| -> Result<f32, String> {
        it.next()
            .ok_or_else(|| format!("missing {axis} in {line:?}"))?
            .parse()
            .map_err(|e| format!("bad {axis} in {line:?}: {e}"))
    };
    Ok(Point3::new(next("x")?, next("y")?, next("z")?))
}

/// Path the fixture lives at, relative to the repository root.
pub const FIXTURE_PATH: &str = "tests/bench/scene.txt";

fn fixture_file() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(FIXTURE_PATH)
}

/// Reads the fixture, generating and writing it first if it is not there.
///
/// # Panics
///
/// Panics if the fixture exists but cannot be parsed — a corrupt fixture would
/// otherwise silently benchmark the wrong scene.
pub fn load() -> Scene {
    let path = fixture_file();
    if let Ok(text) = fs::read_to_string(&path) {
        return deserialize(&text)
            .unwrap_or_else(|e| panic!("{} is unusable: {e}", path.display()));
    }

    let scene = generate();
    if let Some(parent) = path.parent() {
        let _ = fs::create_dir_all(parent);
    }
    fs::write(&path, serialize(&scene))
        .unwrap_or_else(|e| panic!("could not write {}: {e}", path.display()));
    scene
}

/// Writes the fixture, overwriting any existing one, and returns its path.
pub fn write_fixture() -> std::path::PathBuf {
    let path = fixture_file();
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).expect("could not create the fixture directory");
    }
    let scene = generate();
    fs::write(&path, serialize(&scene)).expect("could not write the fixture");
    path
}
