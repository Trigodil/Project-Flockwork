#ifndef SWARM_CONTROL__FORMATION_SHAPES_HPP_
#define SWARM_CONTROL__FORMATION_SHAPES_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace swarm_control
{

// Centered on zero, empty for an unknown shape.
inline std::vector<Eigen::Vector2d> formationSlots(
  const std::string & shape, int num_drones, double spacing)
{
  std::vector<Eigen::Vector2d> slots;
  if (num_drones <= 0) {
    return slots;
  }

  if (shape == "grid") {
    const int cols = std::max(1, static_cast<int>(std::ceil(std::sqrt(num_drones))));
    for (int i = 0; i < num_drones; ++i) {
      slots.emplace_back((i % cols) * spacing, (i / cols) * spacing);
    }
  } else if (shape == "line") {
    for (int i = 0; i < num_drones; ++i) {
      slots.emplace_back(0.0, i * spacing);
    }
  } else if (shape == "column") {
    for (int i = 0; i < num_drones; ++i) {
      slots.emplace_back(-i * spacing, 0.0);
    }
  } else if (shape == "v") {
    const double arm = spacing / std::sqrt(2.0);
    for (int i = 0; i < num_drones; ++i) {
      const int k = (i + 1) / 2;
      const double side = (i % 2 == 1) ? 1.0 : -1.0;
      slots.emplace_back(-k * arm, side * k * arm);
    }
  } else if (shape == "circle") {
    const double radius = num_drones > 1 ? spacing / (2.0 * std::sin(M_PI / num_drones)) : 0.0;
    for (int i = 0; i < num_drones; ++i) {
      const double angle = 2.0 * M_PI * i / num_drones;
      slots.emplace_back(radius * std::cos(angle), radius * std::sin(angle));
    }
  } else {
    return slots;
  }

  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto & s : slots) {
    centroid += s;
  }
  centroid /= num_drones;
  for (auto & s : slots) {
    s -= centroid;
  }
  return slots;
}

// result[i] is drone i's slot in next, min total squared distance (CAPT).
inline std::vector<int> assignSlots(
  const std::vector<Eigen::Vector2d> & current, const std::vector<Eigen::Vector2d> & next)
{
  const int n = static_cast<int>(current.size());
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0), min_v(n + 1);
  std::vector<int> match(n + 1, 0), way(n + 1, 0);
  for (int row = 1; row <= n; ++row) {
    match[0] = row;
    int col = 0;
    std::fill(min_v.begin(), min_v.end(), inf);
    std::vector<bool> used(n + 1, false);
    do {
      used[col] = true;
      const int r = match[col];
      double delta = inf;
      int next_col = 0;
      for (int c = 1; c <= n; ++c) {
        if (used[c]) {
          continue;
        }
        const double cost = (current[r - 1] - next[c - 1]).squaredNorm() - u[r] - v[c];
        if (cost < min_v[c]) {
          min_v[c] = cost;
          way[c] = col;
        }
        if (min_v[c] < delta) {
          delta = min_v[c];
          next_col = c;
        }
      }
      for (int c = 0; c <= n; ++c) {
        if (used[c]) {
          u[match[c]] += delta;
          v[c] -= delta;
        } else {
          min_v[c] -= delta;
        }
      }
      col = next_col;
    } while (match[col] != 0);
    do {
      const int prev = way[col];
      match[col] = match[prev];
      col = prev;
    } while (col != 0);
  }
  std::vector<int> result(n, 0);
  for (int c = 1; c <= n; ++c) {
    result[match[c] - 1] = c - 1;
  }
  return result;
}

}  // namespace swarm_control

#endif  // SWARM_CONTROL__FORMATION_SHAPES_HPP_
