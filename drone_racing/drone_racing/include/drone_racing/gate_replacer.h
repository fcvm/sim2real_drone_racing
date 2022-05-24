#pragma once

#include <cstring>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace gate_replacer { class GateReplacer 
{


public: // Con- and destructor

    GateReplacer (
        const ros::NodeHandle &nh,
        const ros::NodeHandle &nh_private
    );

    GateReplacer() : GateReplacer(ros::NodeHandle(), ros::NodeHandle("~")) {}

    ~GateReplacer() = default;


public: // Methods

    // 1) set_start_time_ = false;
    // 2) replaceGates()
    void replaceGatesExplicit();

    // Returns gates_positions_temp_ with gate_height_ added to the z-component of each element
    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > getGoalPositions();
    
    // Return only_gates_
    std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > getGatePositions();

    // 1) If set_start_time_==false
    //      start_time_ = now
    //      set_start_time_ = true
    // 2) for every gate in model_states_msg_
    //      publish gate as gazebo_msgs::ModelState to "/gazebo/set_model_state"
    //        - only change position with time: max_dynamic_amp_* sin( speed_ * (now-start_time_) + random_phases_... )
    void makeGatesMove();




private: // Methods

    void gazeboModelStatesSub( const gazebo_msgs::ModelStates::ConstPtr &msg );

    void replaceGatesCallback( const std_msgs::EmptyConstPtr &msg );

    // for every gate in model_states_msg_
    //    publish gate as gazebo_msgs::ModelState to "/gazebo/set_model_state"
    //        - only change position: randomly displaced with maximal  max_static_amp_ in each direction
    //    Set gates_positions_temp_ with randomly displaced positions
    void replaceGates();

    void loadParameters();

    void loadGoalPositions();



private: // ROS member variables
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher replace_gate_pub_;
    ros::Subscriber gazebo_model_states_sub_;
    ros::Subscriber replace_gate_sub_;



private: // Member variables

    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > gates_positions_original_;
    
    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > gates_positions_temp_;
    
    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > random_phases_;

    std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > only_gates_;
    
    std::vector< double > gates_orientations_original_;
    
    std::vector< double > gates_orientations_temp_;

    gazebo_msgs::ModelStates model_states_msg_;

    bool found_all_gates_ = false;
    
    int total_number_of_gates_;

    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > gates_velocities_;

    double max_dynamic_amp_;
    
    double max_static_amp_;
    
    double max_dynamic_acc_;
    
    double gate_height_;
    
    double speed_;
    
    ros::WallTime start_time_;
    
    bool set_start_time_ = false;


};
}






/* DESCRIPTION

When an instance of the class GlobalTrajectory is created the following happens:
    
    1)  Several member variables are set
            (ros::NodeHandle)   nh_                         =   const ros::NodeHandle &nh (input arg)
            (ros::NodeHandle)   pnh_                        =   const ros::NodeHandle &nh_private (input arg)
            (int)               total_number_of_gates_      =   {"num_of_gates", 1}
            (double)            max_dynamic_amp_            =   {"gates_dyn_amplitude", 10}
            (double)            max_static_amp_             =   {"gates_static_amplitude", 1}
            (double)            speed_                      =   {"speed_moving_gates", 1.0}
            (double)            gate_height_                =   {"gate_height", 1.0}

            std::vector<Eigen::Vector3d>  gates_positions_original_     <- "goal_positions"
            std::vector<double>           gates_orientations_original_  <- "goal_orientations"
            std::vector<Eigen::Vector3d>  gates_velocities_             =  all elements: Eigen::Vector3d::Zero()

            gates_positions_temp_ = gates_positions_original_
            gates_orientations_temp_ = gates_orientations_original_

            std::vector<Eigen::Vector4d>  only_gates_ <- "goal_positions" and "goal_orientations"

    2)  A Publisher is initialized to the ROS topic
            "/gazebo/set_model_state"
    3)  Subscribers are initialized to the ROS topics
            A)  "/gazebo/model_states"
            B)  "/replace_gates"
        Their callbacks do the following:
            A)  if found_all_gates_==false
                  model_states_msg_ = *msg
                  gates_found_so_far = 0
                  iterate i through msg->name
                    if found_all_gates_==false && i not contains "gate"
                      gates_found_so_far++;
                        if gates_found_so_far == total_number_of_gates_
                          found_all_gates_ = true;
            B)  calls replaceGates():
                  for every gate in model_states_msg_
                    publish gazebo_msgs::ModelState to "/gazebo/set_model_state"
                    - position: randomly displaced with maximal  max_static_amp_ in each direction
                    gates_positions_temp_ with randomly displaced positions
    4)  Initialize std::vector<Eigen::Vector3d> random_phases_






The following public methods are provided:

    A)  void replaceGatesExplicit()

            1)  set_start_time_ = false;
            2)  replaceGates():
                    for every gate in model_states_msg_
                        publish gazebo_msgs::ModelState to "/gazebo/set_model_state"
                            - position: randomly displaced with maximal  max_static_amp_ in each direction
                        set gates_positions_temp_ with randomly displaced positions

    B)  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > getGoalPositions()
        
            1)  Returns gates_positions_temp_ with gate_height_ added to the z-component of each element

    C)  std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > getGatePositions()

            1)  Return only_gates_
    
    D)  void makeGatesMove()

            1)  If set_start_time_==false
                    start_time_ := now
                    set_start_time_ = true
            2)  for every gate in model_states_msg_
                    publish gate as gazebo_msgs::ModelState to "/gazebo/set_model_state"
                        - with time-dependent position: max_dynamic_amp_* sin( speed_ * (now-start_time_) + random_phases_... )
    
    
*/




/* NOTES

gates_velocities_ not used

*/