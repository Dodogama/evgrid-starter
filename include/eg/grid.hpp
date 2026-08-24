// grid.hpp — the core data structure for a 2D evidence grid.
//
// C++ concepts on display here (look these up as you read):
//   * header/implementation split and #pragma once
//   * namespaces
//   * struct vs class, and why the difference is only the default access
//   * const-correctness (methods that promise not to mutate the object)
//   * references (&) vs values, and why we pass big things by const&
//   * std::vector owning heap memory so you never call new/delete
#pragma once

#include <cstddef>   // std::size_t
#include <cstdint>   // fixed-width ints
#include <vector>

namespace eg {

// Plain data. No invariants to protect, so `struct` (public by default) is right.
struct GridSpec {
    int    width       = 200;   // cells in x
    int    height      = 200;   // cells in y
    double resolution  = 0.25;  // meters per cell
    double origin_x    = -25.0; // world coords of cell (0,0) lower-left corner
    double origin_y    = -25.0;
};

// A cell index. Signed on purpose: intermediate math goes negative, and
// unsigned underflow is one of the classic C++ footguns.
struct Cell {
    int x = 0;
    int y = 0;
};

// Log-odds occupancy grid.
//
// We store log-odds instead of probability because Bayesian updates become
// plain addition (no repeated multiply/normalize, no float underflow).
//   l = log(p / (1 - p))
class LogOddsGrid {
public:
    explicit LogOddsGrid(const GridSpec& spec);

    // --- geometry -------------------------------------------------------
    [[nodiscard]] const GridSpec& spec() const { return spec_; }
    [[nodiscard]] bool inBounds(Cell c) const;
    [[nodiscard]] Cell worldToCell(double wx, double wy) const;

    // --- data access ----------------------------------------------------
    // Two overloads: one const (read), one non-const (write). This is the
    // standard C++ idiom, and the compiler picks based on constness.
    [[nodiscard]] float  at(Cell c) const;
    float& at(Cell c);

    void updateCell(Cell c, float delta_logodds);
    void clear();

    // --- conversion -----------------------------------------------------
    [[nodiscard]] float probability(Cell c) const;

private:
    [[nodiscard]] std::size_t index(Cell c) const {
        return static_cast<std::size_t>(c.y) * static_cast<std::size_t>(spec_.width)
             + static_cast<std::size_t>(c.x);
    }

    GridSpec           spec_;
    std::vector<float> data_;  // row-major, size = width * height
};

// Free functions — not everything needs to be a method.
float probabilityFromLogOdds(float l);
float logOddsFromProbability(float p);

}  // namespace eg
