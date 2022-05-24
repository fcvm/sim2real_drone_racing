#pragma once

#include <ros/ros.h>

#include <quadrotor_common/trajectory_point.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

#include "rapid_trajectories/Vec3.h"



namespace drone_racing{ class Visualizer 
{
  
    public: // Con- and destructors

        Visualizer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

        Visualizer() : Visualizer(ros::NodeHandle(), ros::NodeHandle("~")) {}

        virtual ~Visualizer();



    public: // Enums

        enum class Color 
        {
          kRed, kGreen, kBlue, kYellow, kPurple, kWhite, kBlack
        };



    public: // Methods

        // Input: vector of positions (std::vector of Eigen::Vector3d)
        // For each position, publish a marker (visualization_msgs::Marker) to "goal_marker"
        // - position = position
        // - sphere
        // - yellow
        void displayGoalMarker
        (
            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& goal_positions
        );

        // Input: position, id, color
        // Publishs a marker (visualization_msgs::Marker) to "debug"
        // - position = position
        // - id = id
        // - cube
        // - color = color
        void displayDebug
        (
            const double& goal_x, 
            const double& goal_y, 
            const double& goal_z, 
            int id, 
            Color color
        );

        // Input:
        // - vector of trajectory points (std::vector of quadrotor_common::TrajectoryPoint)
        // - bool is_model_based_traj
        // - color
        // - orientation_index
        // 
        // Publish a marker (visualization_msgs::Marker)
        // - msg.id = show_all_trajectories_? ++traj_marker_id_ : 1.0
        // - points = trajectory points ->> positions and velocity vectors /20
        // - line
        // - color = color
        // to topics exclusively depending on input args
        // - "trajectories_cnn" if is_model_based_traj==false
        // - "trajectories_model_based" if is_model_based_traj==true and orientation_index==0
        // - "global_trajectory" if is_model_based_traj==true and orientation_index==3
        void visualizeTrajectory
        (
            std::vector<quadrotor_common::TrajectoryPoint> trajectory,
            bool is_model_based_traj, 
            Color color,
            int orientation_index
        );

        // Publish *vehicle_marker_ (visualization_msgs::MarkerArray) to "vehicle_marker"
        void displayQuadrotor();



    private: // Methods

        // Load parameters from ROS parameter server
        // - child_frame_id_
        // - show_all_trajectories_
        void loadParameters();


        // Creates vehicle_marker_ (shared ptr to visualization_msgs::MarkerArray) according to input.
        void create_vehicle_markers
        (
            int num_rotors, 
            float arm_len, 
            float body_width, 
            float body_height
        );



    private: // ROS member variables

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Publisher goal_marker_pub_;
        ros::Publisher debug_marker_pub_;
        ros::Publisher vehicle_marker_pub_;
        ros::Publisher trajectory_cnn_viz_pub_;
        ros::Publisher trajectory_center_viz_pub_;
        ros::Publisher trajectory_global_viz_pub_;



    private: // Member variables

        int traj_marker_id_;
        std::shared_ptr<visualization_msgs::MarkerArray> vehicle_marker_;
        std::string child_frame_id_;
        double marker_scale_;
        bool show_all_trajectories_;


         
};
}




/* DESCRIPTION

When an instance of the class Visualizer is created the following happens:

    1)  All member variables are set to hard coded values, fetched from ROS parameter server or calculated from hard coded values.
    2)  Publishers are initialized to the ROS topics
            "goal_marker",             
            "debug",                   
            "vehicle_marker",          
            "trajectories_cnn",        
            "trajectories_model_based",
            "global_trajectory",       





The following public methods are provided:

    A)  void displayGoalMarker
    
        1)  Input a vector of positions (std::vector of Eigen::Vector3d)
        2)  For each position, publish a marker (visualization_msgs::Marker) to "goal_marker"
                - position = position
                - sphere
                - yellow


    B)  void displayDebug

        1)  Input position, id, color
        2)  Publish a marker (visualization_msgs::Marker) to "debug"
                - position = position
                - id = id
                - cube
                - color = color
        

    C)  void visualizeTrajectory
        
        1)  Input
                - vector of trajectory points (std::vector of quadrotor_common::TrajectoryPoint)
                - bool is_model_based_traj
                - color
                - orientation_index
        2)  Publish a marker (visualization_msgs::Marker)
                - msg.id = show_all_trajectories_? ++traj_marker_id_ : 1.0
                - points = trajectory points ->> positions and velocity vectors/20
                - line
                - color = color
            to topics exclusively depending on input args
                - is_model_based_traj==false                                : "trajectories_cnn"
                - is_model_based_traj==true     && orientation_index==0     : "trajectories_model_based"
                - is_model_based_traj==true     && orientation_index==3     : "global_trajectory"
        

    D)  void displayQuadrotor

        1)  Publish *vehicle_marker_ (visualization_msgs::MarkerArray) to "vehicle_marker"
        
        
*/



/* NOTES

dont use shared ptrs, try to create all on the stack


*/