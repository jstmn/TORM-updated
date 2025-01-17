/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Modified by : Mincheul Kang */

#ifndef TORM_OPTIMIZER_H_
#define TORM_OPTIMIZER_H_

#include <torm/torm_parameters.h>
#include <torm/torm_trajectory.h>
#include <torm/torm_cost.h>
#include <torm/torm_ik_solver.h>
#include <torm/torm_debug.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_distance_field/collision_env_hybrid.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <trac_ik/trac_ik.hpp>
#include <random>
#include <utility>

namespace torm
{
    class TormOptimizer
    {
    public:
        TormOptimizer(TormTrajectory* trajectory, const planning_scene::PlanningSceneConstPtr& planning_scene,
                      const std::string& planning_group, TormParameters* parameters,
                      const moveit::core::RobotState& start_state,
                      const std::vector<KDL::Frame> &end_poses,
                      const std::vector<int> &simplified_points,
                      TormIKSolver& iksolver,
                      const bool start_flag,
                      const robot_model::JointBoundsVector& bounds);

        virtual ~TormOptimizer();

        bool localOptimizeTSGDforAdaptive(int maxiter);
        bool localOptimizeTSGD(int maxiter);
        bool adaptiveExploration();
        bool iterativeExploration();
        bool checkJointVelocityLimit();
        bool checkSingularity();

        inline void destroy()
        {
            // Nothing for now.
            delete hy_env_;
        }

        void updateStartConfiguration();
        void updateGoalConfiguration();
    private:
        inline double getPotential(double field_distance, double radius, double clearence)
        {
            double d = field_distance - radius;
            double potential = 0.0;

            // three cases below:
            if (d >= clearence)
            {
                potential = 0.0;
            }
            else if (d >= 0.0)
            {
                double diff = (d - clearence);
                double gradient_magnitude = diff * clearence;  // (diff / clearance)
                potential = 0.5 * gradient_magnitude * diff;
            }
            else  // if d < 0.0
            {
                potential = -d + 0.5 * clearence;
            }

            return potential;
        }
        template <typename Derived>
        void getJacobian(int trajectoryPoint, Eigen::Vector3d& collision_point_pos, std::string& jointName,
                         Eigen::MatrixBase<Derived>& jacobian) const;

        void setRobotStateFromPoint(TormTrajectory& group_trajectory, int i);

        int num_joints_;
        int num_vars_free_;
        int num_vars_all_;
        int num_collision_points_;
        int free_vars_start_;
        int free_vars_end_;
        int iteration_;
        ros::WallTime start_time_;
        int start_collision_;
        int end_collision_;
        int worst_collision_cost_state_;
        bool check_collision_;
        bool check_vel_limits_;

        TormTrajectory* full_trajectory_;
        const moveit::core::RobotModelConstPtr& kmodel_;
        std::string planning_group_;
        TormParameters* parameters_;
        TormTrajectory group_trajectory_;
        planning_scene::PlanningSceneConstPtr planning_scene_;
        moveit::core::RobotState state_;
        const moveit::core::JointModelGroup* joint_model_group_;

        collision_detection::CollisionEnvHybrid* hy_env_;
        
        std::vector<TormCost> joint_costs_;
        collision_detection::GroupStateRepresentationPtr gsr_;

        std::vector<std::vector<std::string> > collision_point_joint_names_;
        std::vector<EigenSTL::vector_Vector3d> collision_point_pos_eigen_;
        std::vector<EigenSTL::vector_Vector3d> collision_point_vel_eigen_;
        std::vector<EigenSTL::vector_Vector3d> collision_point_acc_eigen_;
        std::vector<std::vector<double> > collision_point_potential_;
        std::vector<std::vector<double> > collision_point_vel_mag_;
        std::vector<EigenSTL::vector_Vector3d> collision_point_potential_gradient_;
        std::vector<EigenSTL::vector_Vector3d> joint_axes_;
        std::vector<EigenSTL::vector_Vector3d> joint_positions_;
        Eigen::MatrixXd group_trajectory_backup_;
        Eigen::MatrixXd local_group_trajectory_;
        Eigen::MatrixXd best_trajectory_;
        double best_trajectory_backup_cost_;

        std::vector<double> vel_limit_;

        Eigen::MatrixXd smoothness_increments_;
        Eigen::MatrixXd collision_increments_;
        Eigen::MatrixXd endPose_increments_;
        Eigen::MatrixXd final_increments_;

        Eigen::VectorXd smoothness_derivative_;
        Eigen::MatrixXd jacobian_;
        Eigen::MatrixXd jacobian_pseudo_inverse_;
        Eigen::MatrixXd jacobian_jacobian_tranpose_;

        std::vector<std::string> joint_names_;
        std::map<std::string, std::map<std::string, bool> > joint_parent_map_;

        std::vector<KDL::Frame> endPoses_desired_;
        std::vector<KDL::Frame> endPoses_desired_intervals_;
        std::vector<KDL::Twist> delta_twist_;
        std::vector<int> simplified_points_;
        double value_for_endPoses_costs_;

        torm::TormIKSolver &iksolver_;
        bool start_flag_;
        collision_detection::AllowedCollisionMatrix acm_;
        std::vector<bool> joint_kinds_;
        std::default_random_engine generator_;

        inline bool isParent(const std::string& childLink, const std::string& parentLink) const
        {
            if (childLink == parentLink)
            {
                return true;
            }

            if (joint_parent_map_.find(childLink) == joint_parent_map_.end())
            {
                return false;
            }
            const std::map<std::string, bool>& parents = joint_parent_map_.at(childLink);
            return (parents.find(parentLink) != parents.end() && parents.at(parentLink));
        }

        void registerParents(const moveit::core::JointModel* model);
        void initialize();
        void calculateSmoothnessIncrements();
        void calculateCollisionIncrements();
        void calculateEndPoseIncrements();
        void calculateFeasibleIncrements();
        void performForwardKinematics();
        void performForwardKinematics(int start, int end);
        void addIncrementsToTrajectory();
        void updateFullTrajectory();
        void handleJointLimits();
        void getNewTrajectory();
        double getEndPoseCost(bool grad);
        double getEndPoseCost(int start, int end);
        void printTrajectoryEvaluation(const Eigen::MatrixXd& trajectory, double cost);
        std::pair<double, double> maxEndPosePositionalAndRotationalError(const Eigen::MatrixXd& trajectory) const;
        void fillInLinearInterpolation(int s, int g);
        double getCollisionCost(int start, int end);
        double getCollisionCostWithWorst(int start, int end);
        void calculatePseudoInverse();
        void computeJointProperties(int trajectoryPoint);
        bool isCurrentTrajectoryCollisionFree() const;
    };
}

#endif /* Torm_OPTIMIZER_H_ */
