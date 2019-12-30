#include "RRTUtil.hpp"
#include <array>
#include <rrt/planning/Path.hpp>
#include "DebugDrawer.hpp"
#include "planning/MotionConstraints.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/PathSmoothing.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"

using namespace Geometry2d;

namespace Planning {
//todo(Ethan) fix this
REGISTER_CONFIGURABLE(RRTConfig)

ConfigBool* RRTConfig::EnableRRTDebugDrawing;
ConfigDouble* RRTConfig::StepSize;
ConfigDouble* RRTConfig::GoalBias;
ConfigDouble* RRTConfig::WaypointBias;

ConfigInt* RRTConfig::MinIterations;
ConfigInt* RRTConfig::MaxIterations;

using std::vector;
using Geometry2d::Point;

void RRTConfig::createConfiguration(Configuration* cfg) {
    EnableRRTDebugDrawing =
        new ConfigBool(cfg, "PathPlanner/RRT/EnableDebugDrawing", false);
    StepSize = new ConfigDouble(cfg, "PathPlanner/RRT/StepSize", 0.15);
    GoalBias = new ConfigDouble(
        cfg, "PathPlanner/RRT/GoalBias", 0.3,
        "Value from 0 to 1 that determines what proportion of the time the RRT "
        "will grow towards the goal rather than towards a random point");
    WaypointBias = new ConfigDouble(
        cfg, "PathPlanner/RRT/WayPointBias", 0.5,
        "Value from 0 to 1 that determines the portion of the time that the "
        "RRT will"
        " grow towards given waypoints rather than towards a random point");
    MinIterations = new ConfigInt(
            cfg, "PathPlanner/RRT/MinIterations", 100,
            "The minimum number of iterations for running RRT");
    MaxIterations = new ConfigInt(
            cfg, "PathPlanner/RRT/MaxIterations", 250,
            "The maximum number of iterations for running RRT");
}

ConfigBool EnableExpensiveRRTDebugDrawing();

void DrawRRT(const RRT::Tree<Point>& rrt, DebugDrawer* debug_drawer,
             unsigned shellID) {
    // Draw each robot's rrts in a different color
    // Note: feel free to change these, they're completely arbitrary
    static const std::array<QColor, 6> colors = {
        QColor("green"), QColor("blue"),   QColor("yellow"),
        QColor("red"),   QColor("purple"), QColor("orange")};
    QColor color = colors[shellID % colors.size()];

    for (auto& node : rrt.allNodes()) {
        if (node.parent()) {
            debug_drawer->drawLine(
                Segment(node.state(), node.parent()->state()), color,
                QString("RobotRRT%1").arg(shellID));
        }
    }
}

void DrawBiRRT(const RRT::BiRRT<Point>& biRRT, DebugDrawer* debug_drawer,
               unsigned shellID) {
    DrawRRT(biRRT.startTree(), debug_drawer, shellID);
    DrawRRT(biRRT.goalTree(), debug_drawer, shellID);
}

vector<Point> runRRTHelper(
        Point start,
        Point goal,
        const std::shared_ptr<RoboCupStateSpace>& state_space,
        const vector<Point>& waypoints,
        bool straightLine) {
    RRT::BiRRT<Point> biRRT(state_space, Point::hash, 2);
    biRRT.setStartState(start);
    biRRT.setGoalState(goal);

    if(straightLine) {
        // Set the step size to be the distance between the start and goal.
        biRRT.setStepSize(state_space->distance(start, goal));
        // Plan straight toward the goal.
        biRRT.setGoalBias(1);
        // Try up to five times. If unsuccessful after five tries, there
        // probably doesn't exist
        // a straight path.
        biRRT.setMinIterations(0);
        biRRT.setMaxIterations(5);
    } else {
        biRRT.setStepSize(*RRTConfig::StepSize);
        biRRT.setMinIterations(*RRTConfig::MinIterations);
        biRRT.setMaxIterations(*RRTConfig::MaxIterations);
        biRRT.setGoalBias(*RRTConfig::GoalBias);

        if (!waypoints.empty()) {
            biRRT.setWaypoints(waypoints);
            biRRT.setWaypointBias(*RRTConfig::WaypointBias);
        }
    }
    bool success = biRRT.run();
    if (!success) {
        return {};
    }
    vector<Point> points = biRRT.getPath();
    return std::move(points);
}

vector<Point> GenerateRRT(
        Point start,
        Point goal,
        const std::shared_ptr<RoboCupStateSpace>& state_space,
        const vector<Point>& waypoints) {
    // note: we could just use state_space.transitionValid() for the straight
    // line test, but this way is 1.5ms faster
    vector<Point> straight = runRRTHelper(start, goal, state_space, waypoints, true);
    if(!straight.empty()) {
        return std::move(straight);
    }
    return runRRTHelper(start, goal, state_space, waypoints, false);
}

Trajectory RRTTrajectory(const RobotInstant& start, const RobotInstant& goal, const MotionConstraints& motionConstraints, const Geometry2d::ShapeSet& static_obstacles, const vector<DynamicObstacle>& dynamic_obstacles, const vector<Point>& biasWaypoints) {
    Geometry2d::ShapeSet statics = static_obstacles;
    Trajectory path{{}};
    for(int i = 0; i < 10; i++) {
        printf("running RRT...\n");
        auto space = std::make_shared<RoboCupStateSpace>(
                Field_Dimensions::Current_Dimensions, statics);
        RJ::Time t0 = RJ::now();
        std::vector<Geometry2d::Point> points = GenerateRRT(
                start.pose.position(), goal.pose.position(),
                space, biasWaypoints);
        RRT::SmoothPath(points, *space);
        printf("dt: %.6f num_points: %d\n", RJ::Seconds(RJ::now()-t0).count(), (int)points.size());

        t0=RJ::now();
        BezierPath postBezier(points,
                              start.velocity.linear(),
                              goal.velocity.linear(),
                              motionConstraints);

        path = ProfileVelocity(postBezier,
                                         start.velocity.linear().mag(),
                                         goal.velocity.linear().mag(),
                                         motionConstraints,
                                         start.stamp);
        printf("dt: %.6f num_interpolations: %d\n", RJ::Seconds(RJ::now()-t0).count(), (int)path.num_instants());
        //todo(Ethan) delete this?
        if(path.num_instants() < 2) {
            break;
        }

        Geometry2d::Point hitPoint;
        if(path.intersects(dynamic_obstacles, path.begin_time(), &hitPoint, nullptr)) {
            statics.add(std::make_shared<Circle>(hitPoint, Robot_Radius * 1.5));
        } else {
            break;
        }
    }
    return std::move(path);
}

}
