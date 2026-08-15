//! Writes the shared benchmark scene to `tests/bench/scene.txt`.
//!
//! ```text
//! cargo run --release --example dump_bench_fixture
//! ```
//!
//! The Rust benchmark generates this on demand if it is missing, so running
//! this by hand is only needed to hand the same points to the C++ benchmark —
//! see `scripts/bench_cpp.cpp` and `docs/perf-comparison.md`.
//!
//! The generator lives in `benches/shared/fixture.rs` and is pulled in here
//! rather than duplicated. One generator, one scene, two languages reading it.

#[path = "../benches/shared/fixture.rs"]
mod fixture;

fn main() {
    let path = fixture::write_fixture();

    // Read it straight back. A fixture that does not survive its own
    // round trip would silently hand the two benchmarks different points.
    let text = std::fs::read_to_string(&path).expect("could not read back the fixture");
    let scene = match fixture::deserialize(&text) {
        Ok(scene) => scene,
        Err(e) => {
            eprintln!("the fixture failed to parse back: {e}");
            std::process::exit(1);
        }
    };

    let generated = fixture::generate();
    assert_eq!(
        scene.checksum(),
        generated.checksum(),
        "the written fixture does not match the generator"
    );

    println!("wrote {}", path.display());
    println!("  resolution  {}", scene.resolution);
    println!(
        "  sensor      {} {} {}",
        scene.sensor.x, scene.sensor.y, scene.sensor.z
    );
    println!("  scan        {} points", scene.scan.len());
    println!("  queries     {} points", scene.queries.len());
    println!("  directions  {} vectors", scene.directions.len());
    println!("  checksum    {}", scene.checksum());
    println!();
    println!("The C++ benchmark must print the same checksum.");
}
