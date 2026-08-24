#include "eg/grid.hpp"

#include <algorithm>  // std::fill, std::clamp
#include <cmath>      // std::exp, std::log, std::floor

namespace eg {

// Constructor: the member-initializer list runs before the body. Prefer it —
// members get constructed once instead of default-constructed then assigned.
LogOddsGrid::LogOddsGrid(const GridSpec& spec)
    : spec_(spec),
      data_(static_cast<std::size_t>(spec.width) * static_cast<std::size_t>(spec.height), 0.0f) {}

bool LogOddsGrid::inBounds(Cell c) const {
    return c.x >= 0 && c.y >= 0 && c.x < spec_.width && c.y < spec_.height;
}

Cell LogOddsGrid::worldToCell(double wx, double wy) const {
    const double fx = (wx - spec_.origin_x) / spec_.resolution;
    const double fy = (wy - spec_.origin_y) / spec_.resolution;
    return Cell{static_cast<int>(std::floor(fx)), static_cast<int>(std::floor(fy))};
}

float LogOddsGrid::at(Cell c) const { return data_[index(c)]; }

float& LogOddsGrid::at(Cell c) { return data_[index(c)]; }

void LogOddsGrid::updateCell(Cell c, float delta_logodds) {
    if (!inBounds(c)) return;
    float& l = at(c);                        // reference into the vector — no copy
    l = std::clamp(l + delta_logodds, -8.0f, 8.0f);  // clamping keeps the map responsive
}

void LogOddsGrid::clear() { std::fill(data_.begin(), data_.end(), 0.0f); }

float LogOddsGrid::probability(Cell c) const { return probabilityFromLogOdds(at(c)); }

float probabilityFromLogOdds(float l) { return 1.0f - 1.0f / (1.0f + std::exp(l)); }

float logOddsFromProbability(float p) {
    p = std::clamp(p, 1e-4f, 1.0f - 1e-4f);
    return std::log(p / (1.0f - p));
}

}  // namespace eg
