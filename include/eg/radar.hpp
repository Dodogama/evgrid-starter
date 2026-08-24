// radar.hpp — a deliberately simplified radar model and inverse sensor model.
//
// Real automotive/aerospace radar gives you far more (range rate, RCS, elevation,
// per-detection covariance, clustering). This is the skeleton those hang off of.
#pragma once

#include <vector>

#include "eg/grid.hpp"

namespace eg {

// 2D rigid-body pose of the platform in the world frame.
struct Pose2D {
    double x     = 0.0;
    double y     = 0.0;
    double theta = 0.0;  // radians, CCW from +x
};

// One radar return in the SENSOR frame.
struct Detection {
    double range     = 0.0;  // meters
    double azimuth   = 0.0;  // radians, 0 = boresight
    double range_rate = 0.0; // meters/sec, + = receding  (unused in step 1)
    double snr       = 0.0;  // dB
};

using Scan = std::vector<Detection>;

// Tunables for the inverse sensor model. Grouping parameters in a struct beats
// a 6-argument function, and gives you one thing to serialize from YAML later.
struct SensorModel {
    float  p_occupied     = 0.75f;  // belief a cell at the detection is occupied
    float  p_free         = 0.35f;  // belief a traversed cell is free
    double range_sigma    = 0.30;   // meters; thickness of the "occupied" band
    double max_range      = 40.0;
};

// Fuse one scan into the grid, given where the platform was when it was taken.
void integrateScan(LogOddsGrid& grid,
                   const Scan& scan,
                   const Pose2D& sensor_pose,
                   const SensorModel& model);

// Transform a detection from sensor-polar to world-cartesian.
void detectionToWorld(const Detection& d, const Pose2D& pose, double& wx, double& wy);

}  // namespace eg
