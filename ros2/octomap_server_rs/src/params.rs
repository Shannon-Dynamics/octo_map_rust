//! The node's ROS parameters.
//!
//! Read once, at startup. `r2r` populates `node.params` from `--ros-args -p`
//! and from any YAML file passed with `--params-file`, and this module turns
//! that into a plain struct with the defaults filled in.
//!
//! Every parameter is also written back into `node.params` with its effective
//! value, so `ros2 param list` and `ros2 param get` report what the node is
//! actually using rather than only the handful that were passed explicitly.
//! Changing one afterwards with `ros2 param set` updates that record but does
//! not reconfigure the running node — the map is already built to the old
//! resolution and sensor model, and quietly changing either mid-run would
//! corrupt it. Restart the node instead.

use r2r::{Node, Parameter, ParameterValue};

/// Everything the node reads from the parameter server.
#[derive(Debug, Clone)]
pub struct Params {
    /// Edge length of the smallest voxel, in meters.
    pub resolution: f64,
    /// Frame the map is built and published in.
    pub frame_id: String,
    /// Frame the incoming cloud is treated as being in.
    ///
    /// Empty means "believe the cloud's own `header.frame_id`", which is what
    /// a correctly configured driver provides. Set it to override a driver
    /// that leaves the field blank or wrong.
    pub sensor_frame_id: String,
    /// Whether to resolve the cloud's frame into [`frame_id`](Self::frame_id)
    /// through `/tf`.
    pub use_tf: bool,
    /// Fixed sensor pose used when [`use_tf`](Self::use_tf) is false.
    ///
    /// `[x, y, z, roll, pitch, yaw]`, meters and radians, giving the sensor's
    /// pose in the map frame. Empty means the identity — the cloud is already
    /// in the map frame and the sensor sits at the origin.
    ///
    /// This is what makes `use_tf: false` usable for a fixed sensor mounted at
    /// some height: without it every ray would be traced from the floor, and
    /// the free space under the sensor would come out wrong.
    pub sensor_transform: Vec<f64>,

    /// Longest ray integrated, in meters; negative means unlimited.
    pub max_range: f64,
    /// Points nearer than this to the sensor are discarded, in meters.
    pub min_range: f64,
    /// Lowest point height kept, in the map frame.
    pub min_z: f64,
    /// Highest point height kept, in the map frame.
    pub max_z: f64,
    /// Keep one point in every `point_stride`.
    pub point_stride: usize,
    /// Collapse duplicate endpoints to one ray per voxel before integrating.
    pub discretize: bool,
    /// Prune the tree after each scan.
    pub compress_map: bool,

    /// Sensor model: probability assigned to a hit.
    pub prob_hit: f64,
    /// Sensor model: probability assigned to a miss.
    pub prob_miss: f64,
    /// Probability at or above which a voxel counts as occupied.
    pub occupancy_thres: f64,
    /// Lower clamp on a voxel's probability.
    pub clamping_thres_min: f64,
    /// Upper clamp on a voxel's probability.
    pub clamping_thres_max: f64,

    /// Seconds between publishes; the map is published on a timer, not per scan.
    pub publish_period: f64,
    /// Publish the full-probability map alongside the binary one.
    pub publish_full_map: bool,
    /// Publish occupied cells as a `MarkerArray`.
    pub publish_markers: bool,
    /// Publish free cells as a `MarkerArray` too.
    pub publish_free_markers: bool,
    /// Publish occupied cell centers as a `PointCloud2`.
    pub publish_centers: bool,
    /// Compression of the height-color spectrum; the C++ node uses 0.8.
    pub color_factor: f64,

    /// Publish with `transient_local` durability, so late subscribers get the
    /// last map without waiting for the next publish.
    pub latch: bool,
    /// Subscribe to the cloud with best-effort reliability.
    ///
    /// True by default, and it is the compatible choice: a best-effort
    /// subscriber receives from both kinds of publisher, while a reliable one
    /// silently receives nothing from a best-effort sensor driver.
    pub best_effort_cloud_qos: bool,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            resolution: 0.1,
            frame_id: "map".to_string(),
            sensor_frame_id: String::new(),
            use_tf: true,
            sensor_transform: Vec::new(),

            max_range: -1.0,
            min_range: 0.0,
            min_z: f64::NEG_INFINITY,
            max_z: f64::INFINITY,
            point_stride: 1,
            discretize: true,
            compress_map: true,

            // The reference OcTree's own defaults. Repeated here rather than
            // read off a tree so that `ros2 param get` shows a number before
            // the first scan builds one.
            prob_hit: 0.7,
            prob_miss: 0.4,
            occupancy_thres: 0.5,
            clamping_thres_min: 0.12,
            clamping_thres_max: 0.97,

            publish_period: 1.0,
            publish_full_map: true,
            publish_markers: true,
            publish_free_markers: false,
            publish_centers: true,
            color_factor: 0.8,

            latch: true,
            best_effort_cloud_qos: true,
        }
    }
}

/// Reads a parameter, falling back to `default`, and records what was used.
fn double(node: &Node, name: &str, default: f64) -> f64 {
    let value = node
        .get_parameter::<Option<f64>>(name)
        .ok()
        .flatten()
        // A double parameter given as `-p max_range:=5` arrives as an integer,
        // which would otherwise be reported as a type error and silently
        // replaced by the default.
        .or_else(|| {
            node.get_parameter::<Option<i64>>(name)
                .ok()
                .flatten()
                .map(|v| v as f64)
        })
        .unwrap_or(default);
    record(node, name, ParameterValue::Double(value));
    value
}

/// Reads an integer parameter, clamped to non-negative.
fn count(node: &Node, name: &str, default: usize) -> usize {
    let value = node
        .get_parameter::<Option<i64>>(name)
        .ok()
        .flatten()
        .map(|v| v.max(0) as usize)
        .unwrap_or(default);
    record(node, name, ParameterValue::Integer(value as i64));
    value
}

/// Reads a boolean parameter.
fn boolean(node: &Node, name: &str, default: bool) -> bool {
    let value = node
        .get_parameter::<Option<bool>>(name)
        .ok()
        .flatten()
        .unwrap_or(default);
    record(node, name, ParameterValue::Bool(value));
    value
}

/// Reads a string parameter.
fn text(node: &Node, name: &str, default: &str) -> String {
    let value = node
        .get_parameter::<Option<String>>(name)
        .ok()
        .flatten()
        .unwrap_or_else(|| default.to_string());
    record(node, name, ParameterValue::String(value.clone()));
    value
}

/// Reads a double-array parameter.
fn doubles(node: &Node, name: &str, default: &[f64]) -> Vec<f64> {
    let value = node
        .get_parameter::<Option<Vec<f64>>>(name)
        .ok()
        .flatten()
        .unwrap_or_else(|| default.to_vec());
    record(node, name, ParameterValue::DoubleArray(value.clone()));
    value
}

fn record(node: &Node, name: &str, value: ParameterValue) {
    if let Ok(mut params) = node.params.lock() {
        params.insert(name.to_string(), Parameter::new(value));
    }
}

impl Params {
    /// Reads every parameter off the node, filling in defaults.
    pub fn load(node: &Node) -> Self {
        let d = Params::default();

        Self {
            resolution: double(node, "resolution", d.resolution),
            frame_id: text(node, "frame_id", &d.frame_id),
            sensor_frame_id: text(node, "sensor_frame_id", &d.sensor_frame_id),
            use_tf: boolean(node, "use_tf", d.use_tf),
            sensor_transform: doubles(node, "sensor_transform", &d.sensor_transform),

            max_range: double(node, "max_range", d.max_range),
            min_range: double(node, "min_range", d.min_range),
            min_z: double(node, "min_z", d.min_z),
            max_z: double(node, "max_z", d.max_z),
            point_stride: count(node, "point_stride", d.point_stride).max(1),
            discretize: boolean(node, "discretize", d.discretize),
            compress_map: boolean(node, "compress_map", d.compress_map),

            prob_hit: double(node, "prob_hit", d.prob_hit),
            prob_miss: double(node, "prob_miss", d.prob_miss),
            occupancy_thres: double(node, "occupancy_thres", d.occupancy_thres),
            clamping_thres_min: double(node, "clamping_thres_min", d.clamping_thres_min),
            clamping_thres_max: double(node, "clamping_thres_max", d.clamping_thres_max),

            publish_period: double(node, "publish_period", d.publish_period).max(0.01),
            publish_full_map: boolean(node, "publish_full_map", d.publish_full_map),
            publish_markers: boolean(node, "publish_markers", d.publish_markers),
            publish_free_markers: boolean(node, "publish_free_markers", d.publish_free_markers),
            publish_centers: boolean(node, "publish_centers", d.publish_centers),
            color_factor: double(node, "color_factor", d.color_factor),

            latch: boolean(node, "latch", d.latch),
            best_effort_cloud_qos: boolean(
                node,
                "best_effort_cloud_qos",
                d.best_effort_cloud_qos,
            ),
        }
    }

    /// A one-line summary for the startup log.
    pub fn summary(&self) -> String {
        format!(
            "resolution={} m, frame_id={}, use_tf={}, max_range={}, stride={}, publish_period={} s",
            self.resolution,
            self.frame_id,
            self.use_tf,
            if self.max_range < 0.0 {
                "unlimited".to_string()
            } else {
                format!("{} m", self.max_range)
            },
            self.point_stride,
            self.publish_period,
        )
    }
}
