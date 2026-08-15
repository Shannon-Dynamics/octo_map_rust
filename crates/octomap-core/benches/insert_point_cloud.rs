//! Baseline measurements for the two operations a live sensor pipeline runs
//! continuously: integrating a scan, and querying occupancy.
//!
//! ```text
//! cargo bench
//! ```
//!
//! The scene comes from `tests/bench/scene.txt`, the same file
//! `scripts/bench_cpp.cpp` reads, so the two sides measure identical points.
//! See `docs/perf-comparison.md` for the C++ comparison and
//! `benches/shared/fixture.rs` for how the scene is built.

// `criterion_group!` generates an undocumented public function, which trips the
// workspace-wide `missing_docs` lint. Nothing to document there.
#![allow(missing_docs)]

#[path = "shared/fixture.rs"]
mod fixture;

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use octomap_core::{OcTree, Point3, PointCloud};

use fixture::Scene;

/// Loads the shared scene and announces its checksum.
///
/// The C++ benchmark prints the same number. If they differ, the two are not
/// measuring the same points and the comparison is void.
fn scene() -> Scene {
    let scene = fixture::load();
    eprintln!(
        "fixture: {} scan / {} queries / {} directions, checksum {}",
        scene.scan.len(),
        scene.queries.len(),
        scene.directions.len(),
        scene.checksum()
    );
    scene
}

fn cloud(scene: &Scene) -> PointCloud {
    scene.scan.iter().copied().collect()
}

fn bench_insert(c: &mut Criterion) {
    let scene = scene();
    let scan = cloud(&scene);
    let (sensor, resolution) = (scene.sensor, scene.resolution);

    let mut group = c.benchmark_group("insert_point_cloud");
    group.throughput(Throughput::Elements(scan.len() as u64));
    group.sample_size(10);

    // Eager: prunes and refreshes inner nodes during the descent.
    group.bench_function(BenchmarkId::new("eager", scan.len()), |b| {
        b.iter(|| {
            let mut map = OcTree::new(resolution).unwrap();
            map.insert_point_cloud(black_box(&scan), sensor, -1.0, false, false);
            black_box(map.len())
        });
    });

    // Lazy: skips both, then pays for them once at the end. The pair is worth
    // measuring together — lazy only wins if the deferred work is cheaper in
    // bulk than spread across the insert.
    group.bench_function(BenchmarkId::new("lazy_then_finish", scan.len()), |b| {
        b.iter(|| {
            let mut map = OcTree::new(resolution).unwrap();
            map.insert_point_cloud(black_box(&scan), sensor, -1.0, true, false);
            map.update_inner_occupancy();
            map.prune();
            black_box(map.len())
        });
    });

    // Discretized: one ray per endpoint voxel instead of one per return.
    group.bench_function(BenchmarkId::new("discretized", scan.len()), |b| {
        b.iter(|| {
            let mut map = OcTree::new(resolution).unwrap();
            map.insert_point_cloud(black_box(&scan), sensor, -1.0, false, true);
            black_box(map.len())
        });
    });

    group.finish();
}

/// The map every query and cast benchmark runs against.
///
/// Prints its size, which the C++ benchmark also prints. Equal node and leaf
/// counts mean both sides really did build the same tree from the fixture —
/// a stronger check than the checksum alone, which only covers the input.
fn populated(scene: &Scene) -> OcTree {
    let mut map = OcTree::new(scene.resolution).unwrap();
    map.insert_point_cloud(&cloud(scene), scene.sensor, -1.0, false, false);
    eprintln!(
        "populated map: {} nodes, {} leaves",
        map.len(),
        map.count_leaf_nodes()
    );
    map
}

fn bench_query(c: &mut Criterion) {
    let scene = fixture::load();
    let map = populated(&scene);

    let points: &[Point3] = &scene.queries;
    let keys: Vec<_> = points
        .iter()
        .filter_map(|p| map.geometry().coord_to_key_checked(*p))
        .collect();

    let mut group = c.benchmark_group("query");
    group.throughput(Throughput::Elements(points.len() as u64));

    // The call an application makes: world coordinates in, three-state out.
    group.bench_function(BenchmarkId::new("is_occupied_at", points.len()), |b| {
        b.iter(|| {
            let mut hits = 0usize;
            for p in black_box(points) {
                if map.is_occupied_at(*p) == Some(true) {
                    hits += 1;
                }
            }
            black_box(hits)
        });
    });

    // The same lookups with the coordinate conversion already done, to show
    // what that conversion costs.
    group.bench_function(BenchmarkId::new("is_occupied_by_key", keys.len()), |b| {
        b.iter(|| {
            let mut hits = 0usize;
            for k in black_box(&keys) {
                if map.is_occupied(*k) == Some(true) {
                    hits += 1;
                }
            }
            black_box(hits)
        });
    });

    group.finish();
}

fn bench_ray(c: &mut Criterion) {
    let scene = fixture::load();
    let map = populated(&scene);
    let directions: &[Point3] = &scene.directions;

    let mut group = c.benchmark_group("cast_ray");
    group.throughput(Throughput::Elements(directions.len() as u64));
    group.bench_function(BenchmarkId::new("casts", directions.len()), |b| {
        b.iter(|| {
            let mut hits = 0usize;
            for d in black_box(directions) {
                if map.cast_ray(scene.sensor, *d, true, -1.0).is_hit() {
                    hits += 1;
                }
            }
            black_box(hits)
        });
    });
    group.finish();
}

criterion_group!(benches, bench_insert, bench_query, bench_ray);
criterion_main!(benches);
