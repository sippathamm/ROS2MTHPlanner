#ifndef MTH_PLANNER_HPP
#define MTH_PLANNER_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "BasePlanner.h"
#include "ABCPlanner.h"
#include "GWOPlanner.h"
#include "PSOPlanner.h"

namespace MTH
{
  class MTHPlanner : public nav2_core::GlobalPlanner
  {
    public:
      MTHPlanner() = default;

      ~MTHPlanner()
      {
          delete Planner_;
          Planner_ = nullptr;
      }

      void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
                     std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

      void cleanup() override;

      void activate() override;

      void deactivate() override;

      nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                     const geometry_msgs::msg::PoseStamped &goal) override;

    private:
      std::shared_ptr<tf2_ros::Buffer> tf_;
      nav2_util::LifecycleNode::SharedPtr node_;
      nav2_costmap_2d::Costmap2D *costmap_;
      std::string global_frame_, name_;

      ABasePlanner *Planner_ = nullptr;
      ABC::AABCPlanner *ABCPlanner_ = nullptr;
      GWO::AGWOPlanner *GWOPlanner_ = nullptr;
      PSO::APSOPlanner *PSOPlanner_ = nullptr;

      APoint PreviousGoal = APoint(-100, -100);

      // Global parameter
      APoint LowerBound_, UpperBound_;
      int MaximumIteration_, NPopulation_, NBreakpoint_, NInterpolationPoint_, NWaypoint_;
      double VelocityFactor_ = 0.5f;
      INITIAL_POSITION_TYPE InitialPositionType_ = MTH::INITIAL_POSITION::CIRCULAR;
      TRAJECTORY_TYPE TrajectoryType_ = MTH::TRAJECTORY::CUBIC_SPLINE;
      bool Log_ = true;

      // PSO planner parameter
      double SocialCoefficient_, CognitiveCoefficient_;
      double MaximumInertialWeight_, MinimumInertialWeight_;
      int VelocityConfinement_;

      // GWO planner parameter
      double Theta_, K_;
      double Maximum_a_ = 2.2f, Minimum_a_ = 0.02f;
      double MaximumWeight_ = 0.9f, MinimumWeight_ = 0.4f;
  };
}  // MTH

#endif  // MTH_PLANNER_HPP
