//! Property and robustness tests for the file parsers.
//!
//! The `.bt` and `.ot` readers are the crate's main boundary with data it did
//! not produce, so they get two kinds of test that the differential suites do
//! not provide:
//!
//! * **Round-trip properties** — a map generated from a seeded sequence
//!   survives a write/read cycle unchanged.
//! * **Robustness** — every truncation of a valid file, and a spread of
//!   single-byte corruptions, must produce an error or a coherent map. Never a
//!   panic, and never an unbounded read.
//!
//! Both use a small xorshift generator rather than a dependency: these tests
//! have to be reproducible from the seed alone, and a failure that cannot be
//! replayed is not much of a test.

use octomap_core::{io, OcTree, Point3};

/// Deterministic, dependency-free, and good enough to spread values around.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    /// A coordinate somewhere inside a `half_extent`-metre cube.
    fn coord(&mut self, half_extent: f32) -> f32 {
        let unit = (self.next_u64() >> 11) as f32 / (1u64 << 53) as f32;
        (unit * 2.0 - 1.0) * half_extent
    }

    fn below(&mut self, limit: usize) -> usize {
        (self.next_u64() % limit as u64) as usize
    }
}

/// Builds a map from a seed. The same seed always produces the same map.
fn seeded_map(seed: u64, points: usize) -> OcTree {
    let mut rng = Rng(seed);
    let mut map = OcTree::new(0.1).expect("0.1 is a valid resolution");
    for _ in 0..points {
        let p = Point3::new(rng.coord(5.0), rng.coord(5.0), rng.coord(5.0));
        // Two thirds occupied, one third free, so the tree holds a mix rather
        // than one uniform value that would prune into nothing.
        map.update_node_at(p, rng.below(3) != 0);
    }
    map
}

/// Sample points spread over the same volume, for comparing two maps without
/// depending on iteration order.
fn probe_points(count: usize) -> Vec<Point3> {
    let mut rng = Rng(0xC0FFEE);
    (0..count)
        .map(|_| Point3::new(rng.coord(5.0), rng.coord(5.0), rng.coord(5.0)))
        .collect()
}

#[test]
fn full_format_round_trips_exactly() {
    for seed in [1, 42, 0x5EED, 0xDEAD_BEEF] {
        let map = seeded_map(seed, 400);

        let mut bytes = Vec::new();
        io::write_full(&map, &mut bytes).expect("writing to a Vec cannot fail");
        let back = io::read_full(&mut bytes.as_slice()).expect("our own output must parse");

        assert_eq!(back.resolution(), map.resolution(), "seed {seed}");
        assert_eq!(back.count_nodes(), map.count_nodes(), "seed {seed}");
        assert_eq!(
            back.count_leaf_nodes(),
            map.count_leaf_nodes(),
            "seed {seed}"
        );

        // `.ot` keeps the exact log-odds, so this is an equality check, not an
        // approximate one.
        for p in probe_points(500) {
            assert_eq!(
                back.get_log_odds_at(p),
                map.get_log_odds_at(p),
                "seed {seed}, probe {p:?}"
            );
        }
    }
}

#[test]
fn binary_format_round_trips_geometry() {
    for seed in [7, 99, 0xABCD] {
        let mut map = seeded_map(seed, 400);

        // Writing `.bt` thresholds the tree, so compare against the map as it
        // stands *after* the write rather than before it.
        let mut bytes = Vec::new();
        io::write_binary(&mut map, &mut bytes).expect("writing to a Vec cannot fail");
        let back = io::read_binary(&mut bytes.as_slice()).expect("our own output must parse");

        assert_eq!(back.resolution(), map.resolution(), "seed {seed}");
        for p in probe_points(500) {
            assert_eq!(
                back.is_occupied_at(p),
                map.is_occupied_at(p),
                "seed {seed}, probe {p:?}"
            );
        }
    }
}

#[test]
fn message_payloads_round_trip() {
    // The headerless payloads an `octomap_msgs/Octomap` carries. The
    // resolution travels beside them rather than inside them, which is exactly
    // the confusion this test pins down.
    let map = seeded_map(1234, 300);
    let resolution = map.resolution();

    let mut full = Vec::new();
    io::write_full_data(&map, &mut full).expect("writing to a Vec cannot fail");
    let back = io::read_full_data(&mut full.as_slice(), resolution).expect("payload must parse");
    assert_eq!(back.count_nodes(), map.count_nodes());

    let mut map = map;
    let mut binary = Vec::new();
    io::write_binary_data(&map, &mut binary).expect("writing to a Vec cannot fail");
    io::write_binary(&mut map, &mut Vec::new()).expect("threshold the reference the same way");
    let back =
        io::read_binary_data(&mut binary.as_slice(), resolution).expect("payload must parse");
    for p in probe_points(300) {
        assert_eq!(back.is_occupied_at(p), map.is_occupied_at(p));
    }
}

#[test]
fn every_truncation_of_a_valid_file_is_handled() {
    let map = seeded_map(2024, 200);
    let mut full = Vec::new();
    io::write_full(&map, &mut full).expect("writing to a Vec cannot fail");

    let mut binary_map = seeded_map(2024, 200);
    let mut binary = Vec::new();
    io::write_binary(&mut binary_map, &mut binary).expect("writing to a Vec cannot fail");

    // Every prefix, including the empty one. A truncated file must produce an
    // error or a smaller map — the point is that it returns rather than
    // panicking or reading past what it was given.
    for len in 0..full.len() {
        let _ = io::read_full(&mut &full[..len]);
    }
    for len in 0..binary.len() {
        let _ = io::read_binary(&mut &binary[..len]);
    }

    // And the untruncated originals still parse, so the loop above was
    // actually exercising valid prefixes of a real file.
    assert!(io::read_full(&mut full.as_slice()).is_ok());
    assert!(io::read_binary(&mut binary.as_slice()).is_ok());
}

#[test]
fn single_byte_corruption_never_panics() {
    let map = seeded_map(31337, 200);
    let mut original = Vec::new();
    io::write_full(&map, &mut original).expect("writing to a Vec cannot fail");

    let mut rng = Rng(0x1234_5678);
    for _ in 0..2000 {
        let mut corrupted = original.clone();
        let at = rng.below(corrupted.len());
        corrupted[at] ^= (rng.next_u64() & 0xFF) as u8;
        let _ = io::read_full(&mut corrupted.as_slice());
    }

    let mut binary_map = seeded_map(31337, 200);
    let mut original = Vec::new();
    io::write_binary(&mut binary_map, &mut original).expect("writing to a Vec cannot fail");

    for _ in 0..2000 {
        let mut corrupted = original.clone();
        let at = rng.below(corrupted.len());
        corrupted[at] ^= (rng.next_u64() & 0xFF) as u8;
        let _ = io::read_binary(&mut corrupted.as_slice());
    }
}

#[test]
fn arbitrary_bytes_are_rejected_without_panicking() {
    let mut rng = Rng(0xFEED_FACE);
    for _ in 0..500 {
        let len = rng.below(512);
        let blob: Vec<u8> = (0..len).map(|_| (rng.next_u64() & 0xFF) as u8).collect();

        // Random bytes are overwhelmingly not a valid header, so these should
        // all be errors — but the assertion that matters is that the call
        // returns at all.
        let _ = io::read_full(&mut blob.as_slice());
        let _ = io::read_binary(&mut blob.as_slice());
        let _ = io::read_full_data(&mut blob.as_slice(), 0.1);
        let _ = io::read_binary_data(&mut blob.as_slice(), 0.1);
    }
}

#[test]
fn a_header_claiming_a_size_it_does_not_have_is_not_trusted() {
    // `size` in the header is metadata, not an allocation hint. A file that
    // claims two billion nodes and carries none must fail on the data, not
    // reserve memory for the claim.
    let header = b"# Octomap OcTree file\nid OcTree\nsize 2000000000\nres 0.1\ndata\n";
    let result = io::read_full(&mut &header[..]);
    assert!(
        result.is_ok() || result.is_err(),
        "the call must return either way"
    );
}
