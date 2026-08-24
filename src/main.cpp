// main.cpp — drive the whole thing: simulate a platform moving past two walls
// and a pole, integrate 20 radar scans, print the resulting evidence grid.
//
// Run it, then break it on purpose. Change the resolution. Comment out the
// free-space pass. Set p_free to 0.49. Watching the map degrade teaches more
// than reading about it.

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "eg/grid.hpp"
#include "eg/radar.hpp"

namespace {

// --- a toy world of line segments the radar can hit ------------------------
struct Segment {
    double x0, y0, x1, y1;
};

// Ray-segment intersection. Returns range along the ray, or -1 if no hit.
double raySegment(double ox, double oy, double dx, double dy, const Segment& s) {
    const double ex = s.x1 - s.x0;
    const double ey = s.y1 - s.y0;
    const double den = dx * ey - dy * ex;
    if (std::abs(den) < 1e-12) return -1.0;  // parallel

    const double t = ((s.x0 - ox) * ey - (s.y0 - oy) * ex) / den;  // along ray
    const double u = ((s.x0 - ox) * dy - (s.y0 - oy) * dx) / den;  // along segment
    if (t < 0.0 || u < 0.0 || u > 1.0) return -1.0;
    return t;
}

eg::Scan simulateScan(const eg::Pose2D& pose,
                      const std::vector<Segment>& world,
                      std::mt19937& rng) {
    std::normal_distribution<double> range_noise(0.0, 0.08);
    eg::Scan scan;

    // 121 beams across a +/-60 degree field of view.
    for (int i = 0; i < 121; ++i) {
        const double az = (-60.0 + i) * M_PI / 180.0;
        const double a  = pose.theta + az;
        const double dx = std::cos(a);
        const double dy = std::sin(a);

        double best = -1.0;
        for (const Segment& s : world) {
            const double t = raySegment(pose.x, pose.y, dx, dy, s);
            if (t > 0.0 && (best < 0.0 || t < best)) best = t;
        }
        if (best > 0.0 && best < 40.0) {
            scan.push_back(eg::Detection{best + range_noise(rng), az, 0.0, 20.0});
        }
    }
    return scan;
}

void render(const eg::LogOddsGrid& grid) {
    const eg::GridSpec& s = grid.spec();
    // Top row of the grid is the highest y, so walk y downward when printing.
    for (int y = s.height - 1; y >= 0; y -= 2) {   // -=2 because chars are ~2:1 tall
        std::string line;
        for (int x = 0; x < s.width; ++x) {
            const float p = grid.probability(eg::Cell{x, y});
            char ch = ' ';
            if      (p > 0.80f) ch = '#';
            else if (p > 0.60f) ch = '+';
            else if (p > 0.45f) ch = '.';   // unknown sits at exactly 0.5
            else if (p > 0.25f) ch = ',';
            else                ch = ' ';   // confidently free
            line.push_back(ch);
        }
        std::printf("%s\n", line.c_str());
    }
}

}  // namespace

int main() {
    eg::GridSpec spec;
    spec.width      = 120;
    spec.height     = 80;
    spec.resolution = 0.5;
    spec.origin_x   = -10.0;
    spec.origin_y   = -20.0;

    eg::LogOddsGrid grid(spec);
    eg::SensorModel model;

    const std::vector<Segment> world = {
        {  5.0,  8.0, 45.0,  8.0},   // wall to the left of track
        {  5.0, -8.0, 45.0, -8.0},   // wall to the right
        { 30.0, -1.0, 30.0,  1.0},   // a short pole down range
    };

    std::mt19937 rng(42);

    // Platform drives down the +x axis. In step 1 we trust the pose exactly;
    // scenario 4 in the roadmap is where you stop being able to.
    for (int k = 0; k < 20; ++k) {
        eg::Pose2D pose;
        pose.x     = 0.5 * k;
        pose.y     = 0.0;
        pose.theta = 0.0;

        const eg::Scan scan = simulateScan(pose, world, rng);
        eg::integrateScan(grid, scan, pose, model);
    }

    std::printf("evidence grid  ('#' occupied, ' ' free, '.' unknown)\n\n");
    render(grid);

    // Sanity check: the pole at (30, 0) should be believed occupied.
    const eg::Cell pole = grid.worldToCell(30.0, 0.0);
    std::printf("\nP(occupied) at world (30, 0) = %.3f\n", grid.probability(pole));
    return 0;
}
