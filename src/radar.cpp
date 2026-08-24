#include "eg/radar.hpp"

#include <cmath>

namespace eg {

void detectionToWorld(const Detection& d, const Pose2D& pose, double& wx, double& wy) {
    const double a = pose.theta + d.azimuth;
    wx = pose.x + d.range * std::cos(a);
    wy = pose.y + d.range * std::sin(a);
}

void integrateScan(LogOddsGrid& grid,
                   const Scan& scan,
                   const Pose2D& sensor_pose,
                   const SensorModel& model) {
    const float l_occ  = logOddsFromProbability(model.p_occupied);
    const float l_free = logOddsFromProbability(model.p_free);
    const double step  = grid.spec().resolution * 0.5;  // sample twice per cell

    // Range-for over a const& — no copies of Detection, no index arithmetic.
    for (const Detection& d : scan) {
        if (d.range <= 0.0 || d.range > model.max_range) continue;

        const double a  = sensor_pose.theta + d.azimuth;
        const double ux = std::cos(a);
        const double uy = std::sin(a);

        // 1. March along the ray marking free space, stopping short of the hit.
        const double free_until = d.range - 2.0 * model.range_sigma;
        for (double r = 0.0; r < free_until; r += step) {
            const Cell c = grid.worldToCell(sensor_pose.x + r * ux, sensor_pose.y + r * uy);
            grid.updateCell(c, l_free);
        }

        // 2. Mark the occupied band around the measured range.
        const double lo = d.range - model.range_sigma;
        const double hi = d.range + model.range_sigma;
        for (double r = lo; r <= hi; r += step) {
            const Cell c = grid.worldToCell(sensor_pose.x + r * ux, sensor_pose.y + r * uy);
            grid.updateCell(c, l_occ);
        }
    }
}

}  // namespace eg
