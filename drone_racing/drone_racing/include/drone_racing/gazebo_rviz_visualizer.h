#pragma once

#include <string.h>

#include <Eigen/Dense>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>




namespace gazebo_rviz_visualizer { class GazeboRvizVisualizer 
{


    public: // Con- and destructor
        GazeboRvizVisualizer (
            const ros::NodeHandle& nh,
            const ros::NodeHandle& nh_private
        );

        GazeboRvizVisualizer() : GazeboRvizVisualizer(ros::NodeHandle(), ros::NodeHandle("~")) {}

        ~GazeboRvizVisualizer();

    
    public: // Methods

        void visualizeGates();


    private: // Methods

        void gazeboModelStatesSub(const gazebo_msgs::ModelStates::ConstPtr& msg);

        void loadParameters();

    
    private: // ROS member variables

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Subscriber gazebo_model_states_sub_;
        
        ros::Publisher gate_publisher_;
    

    private: // Member variables

        std::vector<Eigen::Vector3d> gates_positions_;
        std::vector<Eigen::Quaterniond> gates_orientations_;
        std::vector<std::string> gates_meshes_;
        int total_number_of_gates_;
        gazebo_msgs::ModelStates model_states_;


};
}












/* DESCRIPTION

When an instance of the class GlobalTrajectory is created the following happens:
    
    1)  Several member variables are set
            (ros::NodeHandle)   nh_                         =   const ros::NodeHandle &nh (input arg)
            (ros::NodeHandle)   pnh_                        =   const ros::NodeHandle &nh_private (input arg)
            (int)               total_number_of_gates_      =   {"num_of_gates", 1}

    2)  A Publisher is initialized to the ROS topic
            "rviz_gates"

    3)  A Subscriber is initialized to the ROS topic
            "/gazebo/model_states"
        Its callback do the following:
            model_states_ = *msg

    4)  For each gate:
            gates_meshes_.emplace_back("package://drone_racing/resources/race_track/real_world/gate/meshes/gate.dae")






The following public methods are provided:

    A)  void GazeboRvizVisualizer::visualizeGates()

            1)  Clear gates_positions_ and gates_orientations_
            2)  Declare next_position and next_orientation
            3)  model_states_temp = model_states_
            4)  for each model in model_states:
                    if model is a gate
                        push back gate position to gates_positions_
                        push back gate orientation to gates_orientations_
            5)  Declare visualization_msgs::MarkerArray msg
                For each gate in gates_positions_:
                    visualization_msgs::Marker gate_marker
                    gate_marker.pose.position =  gate position
                    gate_marker.pose.orientation = gate orientation
                    gate_marker.id = gate index
                    msg.markers.push_back( gate_marker )
            6)  Publish msg to "rviz_gates"
*/




/* NOTES



*/