#pragma once

#include <map>
#include <memory>
#include <planning/PlanRequest.hpp>
#include <planning/paths/Path.hpp>

#include "SystemState.hpp"
#include "planning/DynamicObstacle.hpp"

namespace Planning {

/**
 * @brief Interface for Path Planners that plan paths for a set of robots.
 */
class MultiRobotPathPlanner {
public:
    virtual ~MultiRobotPathPlanner() = default;
    virtual std::map<int, std::unique_ptr<Path>> run(
        std::map<int, PlanRequest> requests) = 0;
};

}  // namespace Planning
