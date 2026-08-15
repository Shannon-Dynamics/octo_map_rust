//! Casting a ray into the map, and the four different ways a cast can fail.
//!
//! ```text
//! cargo run --example ray_cast
//! ```
//!
//! `cast_ray` is the query a planner or a line-of-sight check actually asks:
//! *following this direction from here, what do I hit first?* The interesting
//! part is not the hit — it is that "no hit" has several distinct meanings, and
//! collapsing them into `false` throws away the one a caller usually needs.
//!
//! The reference returns a `bool` and writes the endpoint through an out
//! parameter, which leaves the caller asking whether that parameter was
//! written. Here the result is an enum, so the question does not come up.

use octomap_core::{OcTree, Point3, PointCloud, RayCast, RayCastMiss};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;

    // A wall at x = 2.05 m, seen from near the origin. Everything between the
    // sensor and the wall becomes free; everything behind it stays unknown.
    let sensor = Point3::new(0.05, 0.05, 0.05);
    let mut scan = PointCloud::new();
    for dy in -5..=5 {
        for dz in -5..=5 {
            scan.push(Point3::new(
                2.05,
                0.05 + 0.1 * dy as f32,
                0.05 + 0.1 * dz as f32,
            ));
        }
    }
    map.insert_point_cloud(&scan, sensor, -1.0, false, false);
    println!(
        "map: wall at x = 2.05 m, {} leaves\n",
        map.count_leaf_nodes()
    );

    let along_x = Point3::new(1.0, 0.0, 0.0);

    // 1. The ordinary case: a ray that reaches the wall.
    let hit = map.cast_ray(sensor, along_x, false, 0.0);
    report("straight at the wall", hit);
    assert!(hit.is_hit());
    let hit_point = hit.hit_point().expect("just asserted it is a hit");
    assert!(
        (hit_point.x - 2.05).abs() < 0.05,
        "hit lands on the wall voxel"
    );

    // 2. Stopped by unknown space. Aiming past the edge of the scanned volume
    //    leaves the sensor cone almost immediately, and the first unobserved
    //    voxel ends the cast — because a map that has never seen a place
    //    cannot promise a ray would pass through it.
    let sideways = map.cast_ray(sensor, Point3::new(0.0, 1.0, 0.0), false, 0.0);
    report("sideways, out of the scanned cone", sideways);

    // 3. The same ray with `ignore_unknown` set travels on through unobserved
    //    space until it leaves the addressable volume. Use this when unknown
    //    should be treated as traversable — and only when that is true.
    let sideways_ignoring = map.cast_ray(sensor, Point3::new(0.0, 1.0, 0.0), true, 0.0);
    report("the same ray, ignoring unknown", sideways_ignoring);

    // 4. Stopped by max_range before it ever got to the wall.
    let short = map.cast_ray(sensor, along_x, true, 1.0);
    report("at the wall, but max_range 1.0 m", short);
    assert!(matches!(
        short,
        RayCast::Miss {
            reason: RayCastMiss::MaxRange,
            ..
        }
    ));

    // 5. A direction of zero has nothing to follow. This is the failure mode
    //    that a normalize-then-cast helper would hide behind a division by
    //    zero; here it is a named reason.
    let nowhere = map.cast_ray(sensor, Point3::new(0.0, 0.0, 0.0), true, 0.0);
    report("zero direction", nowhere);
    assert!(matches!(
        nowhere,
        RayCast::Miss {
            reason: RayCastMiss::ZeroDirection,
            ..
        }
    ));

    println!("\nA miss is not one answer. `UnknownVoxel` means the map cannot say,");
    println!("`MaxRange` means it was not asked far enough, and `OutOfBounds` means");
    println!("the ray left the volume the tree can address. They call for different");
    println!("responses from a planner, so the API keeps them apart.");
    Ok(())
}

fn report(label: &str, result: RayCast) {
    match result {
        RayCast::Hit(p) => println!(
            "{label:<38} HIT   at ({:5.2}, {:5.2}, {:5.2})",
            p.x, p.y, p.z
        ),
        RayCast::Miss { last, reason } => {
            let where_it_stopped = match last {
                Some(p) => format!("last voxel ({:5.2}, {:5.2}, {:5.2})", p.x, p.y, p.z),
                None => "never entered the map".to_string(),
            };
            println!("{label:<38} MISS  {reason:?} — {where_it_stopped}");
        }
    }
}
