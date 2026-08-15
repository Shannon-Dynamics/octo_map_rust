//! `octomap_server_rs` — a ROS 2 occupancy mapping node backed by the pure-Rust
//! OctoMap port.
//!
//! It does what `octomap_server` does, with the same topic and service names,
//! so an existing launch file, RViz config or downstream subscriber does not
//! need to know which implementation is running.
//!
//! ```text
//!            sensor_msgs/PointCloud2  ──►  cloud_in
//!                                             │
//!                                     [ filter, transform ]
//!                                             │
//!                                          OcTree
//!                                             │
//!            octomap_msgs/Octomap    ◄──  octomap_binary, octomap_full
//!            visualization_msgs/…    ◄──  occupied_cells_vis_array
//!            sensor_msgs/PointCloud2 ◄──  octomap_point_cloud_centers
//! ```
//!
//! # Where the work is
//!
//! Almost none of it is here. Decoding a `PointCloud2`, filtering a scan,
//! serializing a map into a message payload and turning cells into cubes all
//! live in the `octomap-ros` crate, which has no ROS dependency and is unit
//! tested. This binary moves fields between `r2r` structs and those functions,
//! and owns the parts that genuinely need a middleware: parameters, QoS,
//! the frame graph, and the publish loop.
//!
//! # Threading
//!
//! One map, one owner, no locks. `r2r`'s executor runs on a blocking thread and
//! feeds streams; every stream is consumed by a single `select!` loop that owns
//! the `OcTree`. Insertion is the expensive step and it happens inline, which
//! means a slow scan delays the next one rather than racing it — with a
//! best-effort subscription the middleware drops what it must, which is the
//! right failure mode for a sensor.

mod params;
mod publish;
mod tf;

use std::time::Duration as StdDuration;

use futures::StreamExt;

use octomap_core::OcTree;
use octomap_ros::pointcloud2::{Cloud, FieldRef};
use octomap_ros::{msg as octomap_payload, ScanFilter, Transform3};

use r2r::builtin_interfaces::msg::Time;
use r2r::octomap_msgs::msg::Octomap as OctomapMsg;
use r2r::octomap_msgs::srv::GetOctomap;
use r2r::sensor_msgs::msg::PointCloud2;
use r2r::std_srvs::srv::Empty;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::visualization_msgs::msg::MarkerArray;
use r2r::{Clock, ClockType, Context, Node, Publisher, QosProfile};

use params::Params;
use tf::TfBuffer;

/// Marker namespace for the occupied cells.
const OCCUPIED_NS: &str = "occupied_cells";
/// Marker namespace for the free cells.
const FREE_NS: &str = "free_cells";

/// Everything the publish step needs.
struct Publishers {
    binary: Publisher<OctomapMsg>,
    full: Publisher<OctomapMsg>,
    markers: Publisher<MarkerArray>,
    free_markers: Publisher<MarkerArray>,
    centers: Publisher<PointCloud2>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "octomap_server", "")?;
    let logger = node.logger().to_string();

    let (param_handler, _param_events) = node.make_parameter_handler()?;
    let params = Params::load(&node);
    let mut tree = build_tree(&params, &logger)?;

    // A best-effort subscriber receives from publishers of either reliability;
    // a reliable one silently receives nothing from a best-effort sensor
    // driver. That asymmetry is why the default is the permissive side.
    let cloud_qos = if params.best_effort_cloud_qos {
        QosProfile::sensor_data()
    } else {
        QosProfile::default().reliable().keep_last(10)
    };

    // Latching, in ROS 2 terms, is transient-local durability: RViz connecting
    // a minute after the last publish still gets the map instead of a blank
    // display until the next one.
    let map_qos = || {
        let qos = QosProfile::default().reliable().keep_last(1);
        if params.latch {
            qos.transient_local()
        } else {
            qos
        }
    };

    let mut clouds = node.subscribe::<PointCloud2>("cloud_in", cloud_qos)?;

    // `/tf_static` is published once, before this node exists in the common
    // case, so it has to be transient-local and keep everything — a depth-1
    // subscription would receive only the last of several static publishers.
    let mut tf_dynamic =
        node.subscribe::<TFMessage>("/tf", QosProfile::default().reliable().keep_last(200))?;
    let mut tf_static = node.subscribe::<TFMessage>(
        "/tf_static",
        QosProfile::default().reliable().transient_local().keep_all(),
    )?;

    let publishers = Publishers {
        binary: node.create_publisher::<OctomapMsg>("octomap_binary", map_qos())?,
        full: node.create_publisher::<OctomapMsg>("octomap_full", map_qos())?,
        markers: node.create_publisher::<MarkerArray>("occupied_cells_vis_array", map_qos())?,
        free_markers: node.create_publisher::<MarkerArray>("free_cells_vis_array", map_qos())?,
        centers: node.create_publisher::<PointCloud2>("octomap_point_cloud_centers", map_qos())?,
    };

    let mut reset_service =
        node.create_service::<Empty::Service>("~/reset", QosProfile::services_default())?;
    let mut binary_service = node
        .create_service::<GetOctomap::Service>("octomap_binary", QosProfile::services_default())?;
    let mut full_service = node
        .create_service::<GetOctomap::Service>("octomap_full", QosProfile::services_default())?;

    let mut clock = Clock::create(ClockType::RosTime)?;
    let static_transform = static_transform(&params, &logger);

    r2r::log_info!(&logger, "octomap_server_rs up: {}", params.summary());
    if !params.use_tf {
        r2r::log_info!(
            &logger,
            "use_tf is false: clouds are taken as already being in {:?}",
            params.frame_id
        );
    }

    let spin_logger = logger.clone();
    let spin = tokio::task::spawn_blocking(move || loop {
        node.spin_once(StdDuration::from_millis(10));
    });
    tokio::spawn(param_handler);
    drop(spin_logger);

    let mut frames = TfBuffer::new();
    let mut ticker = tokio::time::interval(StdDuration::from_secs_f64(params.publish_period));
    let mut latest_stamp: Option<Time> = None;
    let mut scans_integrated: u64 = 0;
    let mut scans_dropped: u64 = 0;
    let mut published_once = false;

    loop {
        tokio::select! {
            Some(cloud) = clouds.next() => {
                let stamp = cloud.header.stamp.clone();
                if integrate(&mut tree, &cloud, &params, &frames, &static_transform, &logger) {
                    scans_integrated += 1;
                    latest_stamp = Some(stamp);
                } else {
                    scans_dropped += 1;
                }
            }

            Some(message) = tf_dynamic.next() => absorb(&mut frames, message, false),
            Some(message) = tf_static.next() => absorb(&mut frames, message, true),

            Some(request) = reset_service.next() => {
                tree.clear();
                latest_stamp = None;
                r2r::log_info!(&logger, "map cleared on request");

                let stamp = stamp_or_now(&latest_stamp, &mut clock);
                let cleared = publish::clear_markers(OCCUPIED_NS, &params.frame_id, &stamp);
                let _ = publishers.markers.publish(&cleared);
                let cleared = publish::clear_markers(FREE_NS, &params.frame_id, &stamp);
                let _ = publishers.free_markers.publish(&cleared);
                publish_all(&tree, &params, &publishers, &stamp, &logger);

                if let Err(e) = request.respond(Empty::Response {}) {
                    r2r::log_warn!(&logger, "could not answer reset request: {e}");
                }
            }

            Some(request) = binary_service.next() => {
                let stamp = stamp_or_now(&latest_stamp, &mut clock);
                let map = match octomap_payload::binary_payload(&tree) {
                    Ok(payload) => publish::octomap_message(payload, &params.frame_id, stamp),
                    Err(e) => {
                        r2r::log_error!(&logger, "could not serialize the map: {e}");
                        OctomapMsg::default()
                    }
                };
                if let Err(e) = request.respond(GetOctomap::Response { map }) {
                    r2r::log_warn!(&logger, "could not answer octomap_binary request: {e}");
                }
            }

            Some(request) = full_service.next() => {
                let stamp = stamp_or_now(&latest_stamp, &mut clock);
                let map = match octomap_payload::full_payload(&tree) {
                    Ok(payload) => publish::octomap_message(payload, &params.frame_id, stamp),
                    Err(e) => {
                        r2r::log_error!(&logger, "could not serialize the map: {e}");
                        OctomapMsg::default()
                    }
                };
                if let Err(e) = request.respond(GetOctomap::Response { map }) {
                    r2r::log_warn!(&logger, "could not answer octomap_full request: {e}");
                }
            }

            _ = ticker.tick() => {
                // Nothing has arrived and nothing was ever published: stay
                // quiet rather than filling the log with empty maps. Once a
                // scan lands, publish on every tick, so a map that stops
                // changing keeps being available to late subscribers.
                if scans_integrated == 0 && published_once {
                    continue;
                }
                let stamp = stamp_or_now(&latest_stamp, &mut clock);
                publish_all(&tree, &params, &publishers, &stamp, &logger);
                published_once = true;
            }

            _ = tokio::signal::ctrl_c() => {
                r2r::log_info!(
                    &logger,
                    "shutting down after {scans_integrated} scans ({scans_dropped} dropped), \
                     {} nodes in the map",
                    tree.len()
                );
                break;
            }
        }
    }

    spin.abort();
    Ok(())
}

/// Builds the map and applies the sensor model from the parameters.
fn build_tree(params: &Params, logger: &str) -> Result<OcTree, Box<dyn std::error::Error>> {
    let mut tree = OcTree::new(params.resolution)?;

    // Each setter validates its own argument, and a rejected one leaves the
    // reference default in place. Reporting that is worth more than failing to
    // start: the node is still usable, just not configured the way it was asked.
    let sensor = tree.sensor_mut();
    let apply = |name: &str, result: octomap_core::Result<()>| {
        if let Err(e) = result {
            r2r::log_warn!(logger, "ignoring parameter {name}: {e}");
        }
    };
    apply("prob_hit", sensor.set_prob_hit(params.prob_hit));
    apply("prob_miss", sensor.set_prob_miss(params.prob_miss));
    apply(
        "occupancy_thres",
        sensor.set_occupancy_thres(params.occupancy_thres),
    );
    apply(
        "clamping_thres_min",
        sensor.set_clamping_thres_min(params.clamping_thres_min),
    );
    apply(
        "clamping_thres_max",
        sensor.set_clamping_thres_max(params.clamping_thres_max),
    );

    Ok(tree)
}

/// Reads the fixed sensor pose used when TF is switched off.
fn static_transform(params: &Params, logger: &str) -> Transform3 {
    match params.sensor_transform.len() {
        0 => Transform3::IDENTITY,
        6 => {
            let v = &params.sensor_transform;
            let mut t = Transform3::from_rpy(v[3], v[4], v[5]);
            t.translation = [v[0], v[1], v[2]];
            r2r::log_info!(
                logger,
                "fixed sensor pose: xyz [{}, {}, {}], rpy [{}, {}, {}]",
                v[0],
                v[1],
                v[2],
                v[3],
                v[4],
                v[5]
            );
            t
        }
        n => {
            r2r::log_warn!(
                logger,
                "sensor_transform needs 6 values [x y z roll pitch yaw], got {n}; using identity"
            );
            Transform3::IDENTITY
        }
    }
}

/// Records every transform in a `/tf` or `/tf_static` message.
fn absorb(frames: &mut TfBuffer, message: TFMessage, is_static: bool) {
    for t in message.transforms {
        let stamp_ns =
            i64::from(t.header.stamp.sec) * 1_000_000_000 + i64::from(t.header.stamp.nanosec);
        let transform = Transform3::new(
            [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ],
            [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ],
        );

        // A NaN transform would turn every point of every future scan into a
        // NaN, and the scan would then vanish in the filter with no obvious
        // cause. Refusing it here keeps the graph usable.
        if !transform.is_finite() {
            continue;
        }
        frames.insert(
            &t.header.frame_id,
            &t.child_frame_id,
            transform,
            stamp_ns,
            is_static,
        );
    }
}

/// Decodes one cloud and folds it into the map. Returns whether it landed.
fn integrate(
    tree: &mut OcTree,
    message: &PointCloud2,
    params: &Params,
    frames: &TfBuffer,
    static_transform: &Transform3,
    logger: &str,
) -> bool {
    let fields: Vec<FieldRef<'_>> = message
        .fields
        .iter()
        .map(|f| FieldRef::new(&f.name, f.offset, f.datatype, f.count))
        .collect();

    let cloud = match Cloud::new(
        &fields,
        &message.data,
        message.width,
        message.height,
        message.point_step,
        message.row_step,
        message.is_bigendian,
    ) {
        Ok(cloud) => cloud,
        Err(e) => {
            r2r::log_error!(logger, "unusable point cloud: {e}");
            return false;
        }
    };

    let sensor_frame = if params.sensor_frame_id.is_empty() {
        message.header.frame_id.as_str()
    } else {
        params.sensor_frame_id.as_str()
    };

    let transform = if params.use_tf {
        match frames.lookup(&params.frame_id, sensor_frame) {
            Ok(t) => t,
            Err(e) => {
                // Expected for the first few scans, while /tf fills in. Logged
                // at warn rather than error because it usually resolves itself
                // — and with the edge count, because "0 transforms known" and
                // "40 known but not the one needed" are different problems.
                r2r::log_warn!(
                    logger,
                    "dropping scan: {e} ({} transforms known)",
                    frames.len()
                );
                return false;
            }
        }
    } else {
        *static_transform
    };

    let filter = ScanFilter {
        min_range: params.min_range,
        min_z: params.min_z,
        max_z: params.max_z,
        stride: params.point_stride,
    };
    let (scan, stats) = filter.apply_with_stats(&cloud, &transform);

    if scan.is_empty() {
        r2r::log_warn!(
            logger,
            "no usable points in a cloud of {}: {} non-finite, {} too close, {} outside \
             [{}, {}] in z",
            stats.total,
            stats.non_finite,
            stats.below_min_range,
            stats.outside_height,
            params.min_z,
            params.max_z
        );
        return false;
    }

    // The sensor sits at the origin of its own frame, so the transform's
    // translation is where the rays start in the map frame.
    tree.insert_point_cloud(
        &scan,
        transform.origin(),
        params.max_range,
        false,
        params.discretize,
    );

    if params.compress_map {
        tree.prune();
    }

    r2r::log_debug!(
        logger,
        "integrated {} of {} points, map is now {} nodes",
        stats.kept,
        stats.total,
        tree.len()
    );
    true
}

/// The stamp of the last integrated cloud, or the current time.
///
/// Reusing the cloud's stamp is what keeps the published map aligned with the
/// TF at the moment it was built, which is what RViz needs to draw it in the
/// right place when replaying a bag.
fn stamp_or_now(latest: &Option<Time>, clock: &mut Clock) -> Time {
    if let Some(stamp) = latest {
        return stamp.clone();
    }
    clock
        .get_now()
        .map(|now| Clock::to_builtin_time(&now))
        .unwrap_or(Time { sec: 0, nanosec: 0 })
}

/// Publishes every enabled representation of the map.
fn publish_all(
    tree: &OcTree,
    params: &Params,
    publishers: &Publishers,
    stamp: &Time,
    logger: &str,
) {
    match octomap_payload::binary_payload(tree) {
        Ok(payload) => {
            let message = publish::octomap_message(payload, &params.frame_id, stamp.clone());
            if let Err(e) = publishers.binary.publish(&message) {
                r2r::log_warn!(logger, "could not publish octomap_binary: {e}");
            }
        }
        Err(e) => r2r::log_error!(logger, "could not serialize the binary map: {e}"),
    }

    if params.publish_full_map {
        match octomap_payload::full_payload(tree) {
            Ok(payload) => {
                let message = publish::octomap_message(payload, &params.frame_id, stamp.clone());
                if let Err(e) = publishers.full.publish(&message) {
                    r2r::log_warn!(logger, "could not publish octomap_full: {e}");
                }
            }
            Err(e) => r2r::log_error!(logger, "could not serialize the full map: {e}"),
        }
    }

    if params.publish_markers {
        let markers = publish::marker_array(
            tree,
            &params.frame_id,
            stamp,
            OCCUPIED_NS,
            params.color_factor,
            true,
        );
        if let Err(e) = publishers.markers.publish(&markers) {
            r2r::log_warn!(logger, "could not publish occupied markers: {e}");
        }
    }

    if params.publish_free_markers {
        let markers = publish::marker_array(
            tree,
            &params.frame_id,
            stamp,
            FREE_NS,
            params.color_factor,
            false,
        );
        if let Err(e) = publishers.free_markers.publish(&markers) {
            r2r::log_warn!(logger, "could not publish free markers: {e}");
        }
    }

    if params.publish_centers {
        let centers = publish::centers_cloud(tree, &params.frame_id, stamp);
        if let Err(e) = publishers.centers.publish(&centers) {
            r2r::log_warn!(logger, "could not publish cell centers: {e}");
        }
    }
}
