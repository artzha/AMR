#ifndef OCCUPANCY_MAP_H
#define OCCUPANCY_MAP_H

#include <cmath>
#include <iostream>
#include <vector>

#include "shared/math/line2d.h"
#include "shared/util/timer.h"
#include "visualization/visualization.h"

#define MAX_INT8 (int8_t)127
#define MIN_INT8 (int8_t)(-32)

#define LOG_ODDS_OCCUPIED 3
#define LOG_ODDS_FREE -2

namespace navigation {

using amrl_msgs::VisualizationMsg;
using geometry::line2f;

class OccupancyMap {
 public:
  OccupancyMap() {}
  OccupancyMap(const std::vector<line2f>& lines, const NavigationParams& params)
      : params_(params) {
    resolution = params_.resolution;
    robotRadius = params_.robotRadius;

    // 1 Compute min and max bounds for map
    minX = minY = std::numeric_limits<float>::max();
    maxX = maxY = std::numeric_limits<float>::min();

    for (const auto& line : lines) {
      minX = std::min({minX, line.p0.x(), line.p1.x()});
      minY = std::min({minY, line.p0.y(), line.p1.y()});
      maxX = std::max({maxX, line.p0.x(), line.p1.x()});
      maxY = std::max({maxY, line.p0.y(), line.p1.y()});
    }

    std::cout << "minX: " << minX << " minY: " << minY << " maxX: " << maxX
              << " maxY: " << maxY << std::endl;

    // 2 Initialize map memory
    mapWidth = static_cast<int>((maxX - minX) / resolution);
    mapHeight = static_cast<int>((maxY - minY) / resolution);
    grid.resize(mapHeight, std::vector<int8_t>(mapWidth, 0));
    static_grid.resize(mapHeight, std::vector<bool>(mapWidth, false));

    std::cout << "mapWidth: " << mapWidth << " mapHeight: " << mapHeight << std::endl;

    // 3 Calculate the number of cells to inflate around the line to accommodate the
    // robot size
    robotRadiusCells = static_cast<int>(std::ceil(robotRadius / resolution));

    std::cout << "robotRadius: " << robotRadius << std::endl;
    std::cout << "robotRadiusCells: " << robotRadiusCells << std::endl;

    // 4 Initialize occupancy grid values
    for (const auto& line : lines) {
      addLine(line);
    }
  }

  // Bresenham's line algorithm for integer grids
  void updateOccupancyTrace(
      int x0, int y0, int x1, int y1, std::function<void(int, int)> action) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;  // error value e_xy

    while (true) {
      action(x0, y0);  // Perform action at the current cell
      if (x0 == x1 && y0 == y1) break;
      e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x0 += sx;
      }  // e_xy+e_x > 0
      if (e2 <= dx) {
        err += dx;
        y0 += sy;
      }  // e_xy+e_y < 0
    }
  }

  void updateOccupancy(const Eigen::Vector2f& loc,
                       const std::vector<Eigen::Vector2f>& points) {
    static CumulativeFunctionTimer updateOccupancyTimer_(__FUNCTION__);
    CumulativeFunctionTimer::Invocation invoke(&updateOccupancyTimer_);

    auto [rxIdx, ryIdx] = getIdx(loc.x(), loc.y());

    for (size_t i = 0; i < points.size(); i += 3) {
      auto point = points[i];
      auto [pxIdx, pyIdx] = getIdx(point.x(), point.y());

      // Ray trace from robot position to point
      updateOccupancyTrace(rxIdx, ryIdx, pxIdx, pyIdx, [&](int x, int y) {
        if (x >= 0 && y >= 0 && x < mapWidth && y < mapHeight) {
          // Update grid cell as free along the ray, except for the last cell
          if ((x != pxIdx || y != pyIdx) && !static_grid[y][x]) {
            if (grid[y][x] > MIN_INT8 - LOG_ODDS_FREE) {
              grid[y][x] += LOG_ODDS_FREE;
            } else {
              grid[y][x] = MIN_INT8;
            }
          }
        }
      });

      // Update the cell containing the point as occupied
      if (pxIdx >= 0 && pyIdx >= 0 && pxIdx < mapWidth && pyIdx < mapHeight) {
        if (grid[pyIdx][pxIdx] < MAX_INT8 - LOG_ODDS_OCCUPIED) {
          grid[pyIdx][pxIdx] += LOG_ODDS_OCCUPIED;
        } else {
          grid[pyIdx][pxIdx] = MAX_INT8;
        }
      }
    }
  }

  std::pair<int, int> getIdx(float x, float y) const {
    return std::make_pair(static_cast<int>((x - minX) / resolution),
                          static_cast<int>((y - minY) / resolution));
  }

  Eigen::Vector2f getPose(int xIdx, int yIdx) const {
    return Eigen::Vector2f((xIdx + 0.5f) * resolution + minX,
                           (yIdx + 0.5f) * resolution + minY);
  }

  void addLine(const line2f& line) { drawLineAndInflate(line); }

  const std::vector<std::vector<int8_t>>& getGrid() const { return grid; }

  bool isOccupied(const Eigen::Vector2i& idx) const {
    if (idx.x() < 0 || idx.y() < 0 || idx.x() >= mapWidth || idx.y() >= mapHeight) {
      return true;
    }
    return grid[idx.y()][idx.x()] > 0;
  }

  bool isOccupied(const Eigen::Vector2f& loc) const {
    auto [xIdx, yIdx] = getIdx(loc.x(), loc.y());
    return isOccupied(Eigen::Vector2i(xIdx, yIdx));
  }

  int getWidth() const { return mapWidth; }
  int getHeight() const { return mapHeight; }

  void visualization(VisualizationMsg& global_msg) {
    // For each occupied cell, convert to xy coordinates and publish as a square
    // with same size as grid resolution

    for (size_t yIdx = 0; yIdx < grid.size(); ++yIdx) {
      for (size_t xIdx = 0; xIdx < grid[yIdx].size(); ++xIdx) {
        if (grid[yIdx][xIdx] <= 0) {
          continue;
        }

        const auto& pose = getPose(xIdx, yIdx);

        uint8_t c = 255 - (grid[yIdx][xIdx] + 128) * 255 / 255;
        // Assumes that the grid values are in the range [0, 255]
        uint32_t color = visualization::RGB2INT(c, c, c);
        visualization::DrawPoint(pose, color, global_msg);
      }
    }
  }

 private:
  void drawLineAndInflate(const line2f& line) {
    auto [xStart, yStart] = getIdx(line.p0.x(), line.p0.y());
    auto [xEnd, yEnd] = getIdx(line.p1.x(), line.p1.y());

    int dx = abs(xEnd - xStart), sx = xStart < xEnd ? 1 : -1;
    int dy = -abs(yEnd - yStart), sy = yStart < yEnd ? 1 : -1;
    int err = dx + dy, e2;

    while (true) {
      inflateCell(xStart, yStart);

      if (xStart == xEnd && yStart == yEnd) break;
      e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        xStart += sx;
      }
      if (e2 <= dx) {
        err += dx;
        yStart += sy;
      }
    }
  }

  void inflateCell(int x, int y) {
    for (int inflateY = y - robotRadiusCells; inflateY <= y + robotRadiusCells;
         ++inflateY) {
      for (int inflateX = x - robotRadiusCells; inflateX <= x + robotRadiusCells;
           ++inflateX) {
        if (inflateX < 0 || inflateY < 0 || inflateX >= mapWidth ||
            inflateY >= mapHeight)
          continue;
        if (grid[inflateY][inflateX]) continue;

        // Center cells at zero and compute radius
        float inflateRadiusCells = sqrtf(Sq(inflateX - x) + Sq(inflateY - y));
        if (inflateRadiusCells < robotRadiusCells) {
          grid[inflateY][inflateX] = MAX_INT8;
          static_grid[inflateY][inflateX] = true;
        }
      }
    }
  }

 private:
  NavigationParams params_;               //
  std::vector<std::vector<int8_t>> grid;  // -127, 128
  std::vector<std::vector<bool>> static_grid;
  float minX, minY, maxX, maxY;
  int mapWidth, mapHeight;
  float resolution;
  float robotRadius;
  int robotRadiusCells;
};

}  // namespace navigation

#endif  // MAP_H
