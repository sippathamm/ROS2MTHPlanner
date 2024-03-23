#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "MTHPlanner.hpp"

namespace MTH
{
    void MTHPlanner::configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
                               std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                               std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent;
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        /* Lowerbound and Upperbound */
        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + ".lowerbound_x", 
                                                     rclcpp::ParameterValue(0.0));
        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + ".lowerbound_y", 
                                                     rclcpp::ParameterValue(0.0));
        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + ".upperbound_x", 
                                                     rclcpp::ParameterValue(250.0));
        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + ".upperbound_y", 
                                                     rclcpp::ParameterValue(250.0));
        /* Lowerbound and Upperbound */

        double LowerboundX, LowerboundY;
        double UpperboundX, UpperboundY;
        node_->get_parameter(name_ + ".lowerbound_x", LowerboundX);
        node_->get_parameter(name_ + ".lowerbound_y", LowerboundY);
        node_->get_parameter(name_ + ".upperbound_x", UpperboundX);
        node_->get_parameter(name_ + ".upperbound_y", UpperboundY);

        this->LowerBound_ = APoint(LowerboundX, LowerboundY);
        this->UpperBound_ = APoint(UpperboundX, UpperboundY);

        /* Global parameter configuration */
        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + ".algorithm", 
                                                     rclcpp::ParameterValue("pso"));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + ".maximum_iteration", 
                                                     rclcpp::ParameterValue(100));
                                            
        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + ".n_population", 
                                                     rclcpp::ParameterValue(50));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + ".n_breakpoint", 
                                                     rclcpp::ParameterValue(3));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + ".n_interpolation_point", 
                                                     rclcpp::ParameterValue(30));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + ".initial_position", 
                                                     rclcpp::ParameterValue("circular"));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + ".trajectory", 
                                                     rclcpp::ParameterValue("cubic_spline"));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + ".log", 
                                                     rclcpp::ParameterValue(true));

        node_->get_parameter(name_ + ".maximum_iteration", this->MaximumIteration_);
        node_->get_parameter(name_ + ".n_population", this->NPopulation_);
        node_->get_parameter(name_ + ".n_breakpoint", this->NBreakpoint_);
        node_->get_parameter(name_ + ".n_interpolation_point", this->NInterpolationPoint_);

        std::string InitialPositionType;
        node_->get_parameter(name_ + ".initial_position", InitialPositionType);

        if (InitialPositionType.compare("distributed") == 0)
        {
            this->InitialPositionType_ = MTH::INITIAL_POSITION::DISTRIBUTED;
        }
        else if (InitialPositionType.compare("circular") == 0)
        {
            this->InitialPositionType_ = MTH::INITIAL_POSITION::CIRCULAR;         
        }
        else if (InitialPositionType.compare("linear") == 0)
        {
            this->InitialPositionType_ = MTH::INITIAL_POSITION::LINEAR;
        }
        else
        {
            this->InitialPositionType_ = MTH::INITIAL_POSITION::CIRCULAR;
        }

        std::string TrajectoryType;
        node_->get_parameter(name_ + ".trajectory", TrajectoryType);

        if (TrajectoryType.compare("linear") == 0)
        {
            this->TrajectoryType_ = MTH::TRAJECTORY::LINEAR;
        }
        else if (TrajectoryType.compare("cubic_spline") == 0)
        {
            this->TrajectoryType_ = MTH::TRAJECTORY::CUBIC_SPLINE;
        }
        else 
        {
            this->TrajectoryType_ = MTH::TRAJECTORY::CUBIC_SPLINE;   
        }

        bool Log;
        node_->get_parameter(name_ + ".log", Log);

        if (Log == true)
        {
            this->Log_ = true;
        }
        else if (Log == false)
        {
            this->Log_ = false;
        }
        else
        {
            this->Log_ = true;
        }

        this->NWaypoint_ = 1 + (this->NBreakpoint_ + 1) * NInterpolationPoint_;

        std::string Algorithm;
        node_->get_parameter(name_ + ".algorithm", Algorithm);
        /* Global parameter configuration */

        /* PSO planner parameter configuration */
        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + "pso" + ".social_coefficient", 
                                                     rclcpp::ParameterValue(2.0));
    
        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + "pso" + ".cognitive_coefficient", 
                                                     rclcpp::ParameterValue(1.3));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + "pso" + ".maximum_inertial_weight", 
                                                     rclcpp::ParameterValue(0.9));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + "pso" + ".minimum_inertial_weight", 
                                                     rclcpp::ParameterValue(0.4));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + "pso" + ".velocity_confinement", 
                                                     rclcpp::ParameterValue("hyperbolic"));
                                                    
        node_->get_parameter(name_ + "pso" + ".social_coefficient", this->SocialCoefficient_);
        node_->get_parameter(name_ + "pso" + ".cognitive_coefficient", this->CognitiveCoefficient_);
        node_->get_parameter(name_ + "pso" + ".maximum_inertial_weight", this->MaximumInertialWeight_);
        node_->get_parameter(name_ + "pso" + ".minimum_inertial_weight", this->MinimumInertialWeight_);

        std::string VelocityConfinement;
        node_->get_parameter(name + "pso" + ".velocity_confinement", VelocityConfinement);

        if (VelocityConfinement.compare("random_back") == 0)
        {
            this->VelocityConfinement_ = PSO::VELOCITY_CONFINEMENT::RANDOM_BACK;
        }
        else if (VelocityConfinement.compare("hyperbolic") == 0)
        {
            this->VelocityConfinement_ = PSO::VELOCITY_CONFINEMENT::HYPERBOLIC;
        }
        else if (VelocityConfinement.compare("mixed") == 0)
        {
            this->VelocityConfinement_ = PSO::VELOCITY_CONFINEMENT::MIXED;
        }
        else
        {
            this->VelocityConfinement_ = PSO::VELOCITY_CONFINEMENT::RANDOM_BACK;  
        }
        /* PSO planner parameter configuration */

        /* GWO planner parameter configuration */
        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + "gwo" + ".theta", 
                                                     rclcpp::ParameterValue(2.2));
    
        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + "gwo" + ".k", 
                                                     rclcpp::ParameterValue(1.5));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + "gwo" + ".maximum_a", 
                                                     rclcpp::ParameterValue(2.2));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + "gwo" + ".minimum_a", 
                                                     rclcpp::ParameterValue(0.02));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + "gwo" + ".maximum_weight", 
                                                     rclcpp::ParameterValue(0.9));

        nav2_util::declare_parameter_if_not_declared(node_, 
                                                     name_ + "gwo" + ".minimum_weight", 
                                                     rclcpp::ParameterValue(0.4));
                                                    
        node_->get_parameter(name_ + "gwo" + ".theta", this->Theta_);
        node_->get_parameter(name_ + "gwo" + ".k", this->K_);
        node_->get_parameter(name_ + "gwo" + ".maximum_a", this->Maximum_a_);
        node_->get_parameter(name_ + "gwo" + ".minimum_a", this->Minimum_a_);
        node_->get_parameter(name_ + "gwo" + ".maximum_weight", this->MaximumWeight_);
        node_->get_parameter(name_ + "gwo" + ".minimum_weight", this->MinimumWeight_);
        /* GWO planner parameter configuration */

        if (Algorithm.compare("pso") == 0)
        {
            this->PSOPlanner_ = new PSO::APSOPlanner(this->LowerBound_, this->UpperBound_,
                                                     this->MaximumIteration_, this->NPopulation_, this->NBreakpoint_, this->NWaypoint_,
                                                     this->SocialCoefficient_, this->CognitiveCoefficient_,
                                                     this->MaximumInertialWeight_, this->MinimumInertialWeight_,
                                                     this->VelocityConfinement_,
                                                     this->VelocityFactor_,
                                                     this->InitialPositionType_,
                                                     this->TrajectoryType_,
                                                     this->Log_);
            this->Planner_ = this->PSOPlanner_;
        }
        else if (Algorithm.compare("gwo") == 0)
        {
            this->GWOPlanner_ = new GWO::AGWOPlanner(this->LowerBound_, this->UpperBound_,
                                                     this->MaximumIteration_, this->NPopulation_, this->NBreakpoint_, this->NWaypoint_,
                                                     this->Theta_, this->K_,
                                                     this->Maximum_a_, this->Minimum_a_,
                                                     this->MaximumWeight_, this->MinimumWeight_,
                                                     this->VelocityFactor_,
                                                     this->InitialPositionType_,
                                                     this->TrajectoryType_,
                                                     this->Log_);
            this->Planner_ = this->GWOPlanner_;
        }
        else if (Algorithm.compare("abc") == 0)
        {
            this->ABCPlanner_ = new ABC::AABCPlanner(this->LowerBound_, this->UpperBound_,
                                                     this->MaximumIteration_, this->NPopulation_, this->NBreakpoint_, this->NWaypoint_,
                                                     this->InitialPositionType_,
                                                     this->TrajectoryType_,
                                                     this->Log_);
            this->Planner_ = this->ABCPlanner_;
        }
        else
        {
            this->PSOPlanner_ = new PSO::APSOPlanner(this->LowerBound_, this->UpperBound_,
                                                     this->MaximumIteration_, this->NPopulation_, this->NBreakpoint_, this->NWaypoint_,
                                                     this->SocialCoefficient_, this->CognitiveCoefficient_,
                                                     this->MaximumInertialWeight_, this->MinimumInertialWeight_,
                                                     this->VelocityConfinement_,
                                                     this->VelocityFactor_,
                                                     this->InitialPositionType_,
                                                     this->TrajectoryType_,
                                                     this->Log_);
            this->Planner_ = this->PSOPlanner_;
        }
    }

    void MTHPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s of type NavfnPlanner", name_.c_str());
    }

    void MTHPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type NavfnPlanner", name_.c_str());
    }

    void MTHPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner", name_.c_str());
    }

    nav_msgs::msg::Path MTHPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start, 
                                               const geometry_msgs::msg::PoseStamped &goal)
    {
        nav_msgs::msg::Path global_path;

        if (start.header.frame_id != global_frame_) {
            RCLCPP_ERROR(
            node_->get_logger(), "Planner will only except start position from %s frame",
            global_frame_.c_str());
            return global_path;
        }
 
        if (goal.header.frame_id != global_frame_) {
            RCLCPP_INFO(
            node_->get_logger(), "Planner will only except goal position from %s frame",
            global_frame_.c_str());
            return global_path;
        }

        APoint Start, Goal;
        unsigned int MapX, MapY;

        this->costmap_->worldToMap(start.pose.position.x, start.pose.position.y, MapX, MapY);
        Start = APoint(MapX, MapY);

        this->costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, MapX, MapY);
        Goal = APoint(MapX, MapY);

        if (this->PreviousGoal != Goal)
        {
            this->Planner_->Clear();
        }

        this->PreviousGoal = Goal;

        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        std::vector<APoint> Path;

        bool IsPathFound = this->Planner_->CreatePlan(this->costmap_->getCharMap(), Start, Goal, Path);

        if (IsPathFound)
        {
            double WorldX, WorldY;

            for (size_t i = 0; i < Path.size(); ++i) 
            {
                MapX = static_cast<unsigned int>(Path[i].X);
                MapY = static_cast<unsigned int>(Path[i].Y);

                this->costmap_->mapToWorld(MapX, MapY, WorldX, WorldY);

                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = WorldX;
                pose.pose.position.y = WorldY;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                pose.header.stamp = node_->now();
                pose.header.frame_id = global_frame_;

                global_path.poses.push_back(pose);
            }

            geometry_msgs::msg::PoseStamped goal_pose = goal;
            goal_pose.header.stamp = node_->now();
            goal_pose.header.frame_id = global_frame_;
            global_path.poses.push_back(goal_pose);
        }

        return global_path;
    }
}  // MTH

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MTH::MTHPlanner, nav2_core::GlobalPlanner)
