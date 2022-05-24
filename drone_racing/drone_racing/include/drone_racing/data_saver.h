#pragma once

#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>

#include <Eigen/Dense>


namespace drone_racing{ class DataSaver 
{

    public: // Con- and destructors
  
        DataSaver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

        DataSaver() : DataSaver(ros::NodeHandle(), ros::NodeHandle("~")) {}

        virtual ~DataSaver();



    public: // Structs

        struct trajectory_info 
        {
            double idx_right;
            double idx_up;
            double curr_vel;
            double max_vel;
        };



    public: // Methods

        // If record_data_==true:
        //     1) Save image_ to file "$(root_dir_)/Run_$(run_idx_)/images/frame_center_$(frame_counter_).jpg"
        //     2) Increments frame_counter_
        //     3) Add the line 
        //            "$(traj_coor.idx_right);$(traj_coor.idx_up);$(traj_coor.curr_vel);$(traj_coor.max_vel);$(new_dagger_batch)" 
        //        to file "$(root_dir_)/Run_$(run_idx_)/labels.txt" 
        void saveImage
        (
            const DataSaver::trajectory_info goal_coor,
            const int model_based_frame
        );

        // Add the line "$(run_idx_)" to file "$(root_dir_)/fails.txt" 
        void labelAsBadData();

        // If created_directory_==false:
        //     1) Create directory "$(root_dir_)/Run_$(run_idx_)/images"
        //     2) Set frame_counter=0
        //     3) Set created_directory_==true
        bool createDirectory();

        // Set model_based_prediction_=prediction
        void setModelBasedPrediction(Eigen::Vector3d prediction);
        
        // Set network_selection_=selection
        void setNetworkSelection(Eigen::Vector3d selection);

        // Set record_data_=true
        void startRecording();
        // Set record_data_=false
        void stopRecording();



    private: // Callbacks
  
        // 1) Save messages from topic "image_rgb" to image_
        // 2) Call addPredictionAndRepublish()
        void imageCallback(const sensor_msgs::ImageConstPtr &msg);

        // 1) Save messages from topic "run_idx" to run_idx_
        // 2) Set created_directory_=false
        void setRunIdxCallback(const std_msgs::Int16ConstPtr &msg);


    
    private: // Methods

        // Publish an edited version of image_ to topic "image_with_prediction"
        // - shrink image
        // - add grid
        // - add coordinate and velocity info of network_selection_ and model_based_prediction_
        void addPredictionAndRepublish();

        // Load parameters from ROS parameter server
        // - root_dir_
        // - camera_fov_deg_yaw_
        // - camera_fov_deg_pitch_
        void loadParameters();



    private: // Member variables
        
        // Relatively resolved node handle
        ros::NodeHandle nh_;
        // Privately resolved node handle
        ros::NodeHandle pnh_;
        // ROS subscribers & publishers
        ros::Subscriber run_idx_sub_;
        ros::Subscriber image_sub_;
        ros::Publisher image_pub_;

        // locks access to image_
        std::mutex mtx_image_save_;
        
        // Image from onboard camera
        cv::Mat image_;
        // ?
        Eigen::Vector3d model_based_prediction_;
        // ?
        Eigen::Vector3d network_selection_;
        // Index of run
        int run_idx_;
        // Index of frame ?
        int frame_counter_;
        // Root dir to save files
        std::string root_dir_;
        bool created_directory_;
        bool record_data_;
        double camera_fov_deg_yaw_;
        double camera_fov_deg_pitch_;



};
}




/* DESCRIPTION

When an instance of the class DataSaver is created happens the following:

    1)  Several member variables are set to hard coded values or fetched from ROS parameter server
    2)  A Publisher is initialized to the ROS topic
            "image_with_prediction"
    3)  Subscribers are initialized to the ROS topics
            A)  "image_rgb"
            B)  "run_idx"
        Their callbacks do the following:
            A)  i) Save messages from topic "image_rgb" to image_
                ii) Publish an edited version of image_ to topic "image_with_prediction"
                        - shrink image
                        - add grid
                        - add coordinate and velocity info of network_selection_ and model_based_prediction_
            B)  i) Save messages from topic "run_idx" to run_idx_
                ii) Set created_directory_=false






The following public methods are provided:

    1)  void saveImage
        (
            const DataSaver::trajectory_info goal_coor,
            const int model_based_frame
        )
            
            If record_data_==true:
                1)  Save image_ to file "$(root_dir_)/Run_$(run_idx_)/images/frame_center_$(frame_counter_).jpg"
                2)  Increments frame_counter_
                3)  Add the line 
                        "$(traj_coor.idx_right);$(traj_coor.idx_up);$(traj_coor.curr_vel);$(traj_coor.max_vel);$(new_dagger_batch)" 
                    to file 
                        "$(root_dir_)/Run_$(run_idx_)/labels.txt" 


    2)  void labelAsBadData()

            1)  Add the line "$(run_idx_)" to file "$(root_dir_)/fails.txt" 
        

    3)  bool createDirectory()
        
            If created_directory_==false:
                1)  Create directory "$(root_dir_)/Run_$(run_idx_)/images"
                2)  Set frame_counter=0
                3)  Set created_directory_==true
        

    4)  void setModelBasedPrediction    (Eigen::Vector3d prediction)
        void setNetworkSelection        (Eigen::Vector3d selection);
            
            Set model_based_prediction_=prediction
            Set network_selection_=selection
        
        
    5)  void startRecording() 
        void stopRecording()
    
            Set record_data_=true 
            Set record_data_=false
        

*/