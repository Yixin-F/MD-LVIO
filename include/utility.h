#pragma once

#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/esf.h>   

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <unordered_map>
#include <utility>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <random>
#include <filesystem> // requires gcc version >= 8
#include <yaml-cpp/yaml.h> // yaml

#include "assert.h"
#include "tictoc.h"

namespace fs = std::filesystem; // file-process

// pose-point cloud from lio-sam
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                  

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))
typedef PointXYZIRPYT  Pose;

// point use here
struct PointAPRI{
    float x, y, z;
    float range;
    float angle;
    float azimuth;
    float intensity = 0.f;
    int range_idx = -1;
    int sector_idx = -1 ;
    int azimuth_idx = -1;
    int voxel_idx = -1;  
};

// curved voxel in hash table
struct Voxel{
    int range_idx;
    int sector_idx;
    int azimuth_idx;
    int name = -1;
    pcl::PointXYZI voxCenter;   // the point center's intensity is its id in voxel cloud
    std::vector<int> ptIdx;  // the vector of id in noground cloud
    float intensity_av = 0.f;
    float intensity_cov = 0.f;
};

// feature values
struct FeatureValue{
    FeatureValue() = delete;
    FeatureValue(std::string value_name_, double value_) : name(value_name_), value(value_) {}
    ~FeatureValue() {}

    std::string name = "";
    double value = 0.0;
};

// one type of feature
struct Feature{
    Feature() = delete;
    Feature(std::string feature_name): name(feature_name) {}
    ~Feature() {}

    std::string name = "";
    std::vector<FeatureValue> feature_values;
};

// one Object
struct Object{
    Object() = delete;
    Object(std::vector<PointAPRI>){
        // TODO:
    }
    ~Object();
    int label = -1;  // building, tree, car
    int state = -1;   //  dynamic 1, static 0 
    float color[3];
    std::pair<pcl::PointXYZI, pcl::PointXYZI> bounding_box;
    std::vector<int> occupy_pts;  
    std::vector<int> occupy_voxels; 
    Eigen::VectorXd feature_vec;
};

// one frame
struct Frame{
    int id = -1;
    std::vector<PointAPRI> point_use;
    pcl::PointCloud<pcl::PointXYZI>::Ptr vox_cloud;  // voxel cloud -> intensity = voxel_idx
    std::unordered_map<int, Voxel> hash_cloud;
    std::unordered_map<int, Object> object_set;
};

class Utility{
public:
    // common
    std::string out_path;
    int kNumOmpCores;
    bool save_bool;
    bool gpu_bool;
    bool bag_bool;
    bool bin_bool;
    int skip_num;
    bool ri_bool;
    bool tc_bool;
    std::vector<float> tr_v;
    Eigen::Matrix4f tr;

    // session
    std::string data_path;
    std::string label_path;
    std::string pose_path;
    int start_id;
    int end_id;

    // ssc
    std::string ori_path;
    std::string seg_path;
    std::string map_path;
    std::string evaluate_path;

    float min_range;
    float max_range;
    float min_angle;
    float max_angle;
    float min_azimuth;
    float max_azimuth;

    float range_res;
    float angle_res;
    float azimuth_res;

    int toBeClass;

    int iteration;
    float intensity_max;
    float intensity_diff;
    float intensity_cov;

    // feature
    float sensor_height;
    float min_z;

    double kOneThird;
    double kLinearityMax;
    double kPlanarityMax;
    double kScatteringMax;
    double kOmnivarianceMax;
    double kAnisotropyMax;
    double kEigenEntropyMax;
    double kChangeOfCurvatureMax;
    double kNPointsMax;

    float car_height;
    float car_quare;

    int building;
    int tree;
    int car;

    // dynamic
    std::vector<int> dynamic_label;
    float occupancy;
    float respond_score;

    ros::NodeHandle nh;

    ~Utility(){}
    Utility(){
        // common
        nh.param<std::string>("common/out_path_", out_path,  " ");
        nh.param<bool>("common/gpu_bool_", gpu_bool, false);
        nh.param<bool>("common/save_bool_", save_bool, false);
        nh.param<bool>("common/gpu_bool_", gpu_bool, false);
        nh.param<bool>("common/bag_bool_", bag_bool, false);
        nh.param<bool>("common/bin_bool_", bin_bool, true);
        nh.param<bool>("common/ri_bool_", ri_bool, false);
        nh.param<bool>("common/tc_bool_", tc_bool, false);
        nh.param<int>("common/skip_num_", skip_num, 1);
        nh.param<std::vector<float>>("common/tr_v_", tr_v, std::vector<float>());
        tr = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(tr_v.data(), 4, 4);

        // session
        nh.param<std::string>("session/data_path_", data_path,  " ");
        nh.param<std::string>("session/label_path_", label_path,  " ");
        nh.param<std::string>("session/pose_path_", pose_path,  " ");
        nh.param<int>("session/start_id_", start_id, -1);
        nh.param<int>("session/end_id_", end_id, -1);

        // ssc
        nh.param<std::string>("ssc/ori_path_", ori_path,  " ");
        nh.param<std::string>("ssc/seg_path_", seg_path,  " ");
        nh.param<std::string>("ssc/map_path_", map_path,  " ");
        nh.param<std::string>("ssc/evaluate_path_", evaluate_path,  " ");

        nh.param<float>("ssc/min_range_", min_range,  2.0);
        nh.param<float>("ssc/max_range_", max_range,  50.0);
        nh.param<float>("ssc/min_angle_", min_angle,  0.0);
        nh.param<float>("ssc/max_angle_", max_angle,  360.0);
        nh.param<float>("ssc/min_azimuth_", min_azimuth,  -30.0);
        nh.param<float>("ssc/max_azimuth_", max_azimuth,  60.0);

        nh.param<float>("ssc/range_res_", range_res,  0.2);
        nh.param<float>("ssc/angle_res_", angle_res,  1.2);
        nh.param<float>("ssc/azimuth_res_", azimuth_res,  2.0);

        nh.param<int>("ssc/toBeClass_", toBeClass, 10);

        nh.param<int>("ssc/iteration_", iteration, 3);
        nh.param<float>("ssc/intensity_max_", intensity_max,  200.0);
        nh.param<float>("ssc/intensity_diff_", intensity_diff,  3.0);
        nh.param<float>("ssc/intensity_cov_", intensity_cov,  1.5);

        // feature
        nh.param<float>("feature/sensor_height_", sensor_height,  1.73);
        nh.param<float>("feature/min_z_", min_z,  -1.5);

        nh.param<double>("feature/kOneThird_", kOneThird, 0.333);
        nh.param<double>("feature/kLinearityMax_",  kLinearityMax, 740.0);
        nh.param<double>("feature/kPlanarityMax_", kPlanarityMax, 959.0);
        nh.param<double>("feature/kScatteringMax_", kScatteringMax, 1248.0);
        nh.param<double>("feature/kOmnivarianceMax_", kOmnivarianceMax, 0.278636);
        nh.param<double>("feature/kAnisotropyMax_", kAnisotropyMax, 1248.0);
        nh.param<double>("feature/kEigenEntropyMax_", kEigenEntropyMax, 0.956129);
        nh.param<double>("feature/kChangeOfCurvatureMax_", kChangeOfCurvatureMax, 0.99702);
        nh.param<double>("feature/kNPointsMax_", kNPointsMax, 13200.0);

        nh.param<float>("feature/car_height_", car_height,  1.0);
        nh.param<float>("feature/car_quare_", car_quare,  15.0);

        nh.param<int>("feature/building_", building, 0);
        nh.param<int>("feature/tree_", tree, 1);
        nh.param<int>("feature/car_", car, 2);

        // dynamic
        nh.param<std::vector<int>>("dynamic/dynamic_label_", dynamic_label, std::vector<int>());
        nh.param<float>("dynamic/occupancy_", occupancy,  0.5);
        nh.param<float>("dynamic/respond_score_", respond_score,  0.6);
    }

    void fsmkdir(std::string path_){
        if(fs::exists(path_)){
            fs::remove_directory(path_);  // remove old
        }
        else if(!fs::is_directory(path_) || !fs::exists(path_)){
            fs::create_directories(path_);   // create new
        }
        else{
            ROS_ERROR("Directory processing error ...");
            ros::shutdown();
        }
    } 

    template<typename CloudT> 
    sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, CloudT thisCloud, ros::Time thisStamp, std::string thisFrame){
        sensor_msgs::PointCloud2 tmp_cloud;
        pcl::toROSMsg(*thisCloud, tmp_cloud);
        tmp_cloud.header.stamp = thisStamp;
        tmp_cloud.header.frame_id = thisFrame;
        if(thisPub->getNumSubscribers() != 0){
            thisPub->publish(tmp_cloud);
        }
        return tmp_cloud;
    }

    template<typename T> 
    float rad2deg(const T& radians){
        return (float)radians * 180.0 / M_PI; 
    }

    template<typename T> 
    float deg2rad(const T& degrees){
        return (float)degrees * M_PI / 180.0; 
    }
    
    template<typename PointT> 
    float pointDistance3d(const PointT& p1, const PointT& p2){
        return (float)sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
    }

    template<typename PointT> 
    float pointDistance2d(const PointT& p1, const PointT& p2){
        return (float)sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
    }

    template<typename PointT> 
    float pointDistance3d(const PointT& p1){
        return (float)sqrt((p1.x)*(p1.x) + (p1.y)*(p1.y) + (p1.z)*(p1.z));
    }

    template<typename PointT> 
    float pointDistance2d(const PointT& p1){
        return (float)sqrt((p1.x)*(p1.x) + (p1.y)*(p1.y));
    }

    template<typename PointT> 
    float getPolarAngle(const PointT& p){
        if(p.x == 0 && p.y == 0){
            return 0.f;
        }
        else if(p.y >= 0){
            return (float)rad2deg((float)atan2(p.y, p.x));
        }
        else if(p.y < 0){
            return (float)rad2deg((float)atan2(p.y, p.x) + 2*M_PI);
        }
    }

    template<typename PointT> 
    float getAzimuth(const PointT& p){
        return (float)rad2deg((float)atan2(p.z, (float)pointDistance2d(p)));
    }

    template<typename CloudTPtr> 
    void transformCloud(const CloudTPtr& cloudIn_, const Eigen::Affine3f& transCur_, CloudTPtr& cloudOut_){
        int cloudSize = cloudIn_->points.size();
        cloudOut_->points.resize(cloudSize);
    
        #pragma omp parallel for num_threads(kNumOmpCores)
        for(int i = 0; i < cloudSize; i++){
            cloudOut_->points[i].x = transCur_(0,0) * cloudIn_->points[i].x + transCur_(0,1) * cloudIn_->points[i].y + transCur_(0,2) * cloudIn_->points[i].z + transCur_(0,3);
            cloudOut_->points[i].y = transCur_(1,0) * cloudIn_->points[i].x + transCur_(1,1) * cloudIn_->points[i].y + transCur_(1,2) * cloudIn_->points[i].z + transCur_(1,3);
            cloudOut_->points[i].z = transCur_(2,0) * cloudIn_->points[i].x + transCur_(2,1) * cloudIn_->points[i].y + transCur_(2,2) * cloudIn_->points[i].z + transCur_(2,3);
            cloudOut_->points[i].intensity = cloudIn_->points[i].intensity;
        }
    }

    template<typename CloudTPtr>
    void loadCloud(CloudTPtr& cloud_, const std::string& path_){
        if(pcl::io::loadPCDFile(path_, *cloud_) == -1){
            ROS_WARN("pose file %s load error", path_.c_str());
            ROS_BREAK();
        }
        else{
            ROS_DEBUG("cloud load: %s load success, pt_num: %d", path_.c_str(), (int)cloud_->points.size());
        }
    }

    template<typename CloudTPtr>
    void saveCloud(const CloudTPtr& cloud_, const std::string& path_){  // root path
        cloud_->height = 1;
        cloud_->width = cloud_->points.size();
        if(save){
            if(cloud_->points.size() == 0 || pcl::io::savePCDFile(path_, *cloud_) == -1){
                ROS_WARN("%s save error ", (path_).c_str());
            }
            ROS_DEBUG("cloud save: %s save success, pt_num: %d", path_.c_str(), (int)cloud_->points.size());
        }
    }

    template<typename CloudTPtr>
    void getCloudByVec(const CloudTPtr& cloud_, const std::vector<int>& vec_, CloudTPtr& cloud_out_){
        for(auto& it : vec_){
            cloud_out_->points.push_back(cloud_->points[it]);
        }
    }

    template<typename T>
    void addVec(std::vector<T>& vec_central_, const std::vector<T>& vec_add_){
        vec_central_.insert(vec_central_.end(), vec_add_.begin(), vec_add_.end());
    }

    template<typename T>
    void reduceVec(std::vector<T>& vec_central_, const std::vector<T>& vec_reduce_){
        for(auto& it = vec_reduce_.begin(); it != vec_reduce_.end(); it++){
            vec_central_.erase(std::remove(vec_central_.begin(), vec_central_.end(), *it), vec_central_.end());
        }
    }

    template<typename T>
    void sampleVec(std::vector<T>& vec_central_){
        std::sort(vec_central_.begin(), vec_central_.end());
        vec_central_.erase(std::unique(vec_central_.begin(), vec_central_.end()), vec_central_.end());
    }

    bool findNameInVec(const int& name_, const std::vector<int>& vec_){
        if(std::count(vec_.begin(), vec_.end(), name_)){
            return true;
        }
        else{
            return false;
        }
    }

    bool findNameInVec(const std::vector<int>& vec1_, const std::vector<int>& vec2_){
        for(auto& i : vec2_){
            if(findNameInVec(i, vec1_)){
                return true;
            }
            else{
                continue;
            }
        }
        return false;
    }

    Eigen::MatrixXd turnVec2Matrix(const std::vector<FeatureValue>& vec_){
        int size = vec_.size();
        Eigen::MatrixXd m(1, size);
        for(int i = 0; i < vec_.size(); i++){   
            m(0, i) = vec_[i].value;   // row add
        }
        return m;
    }

    Eigen::Vector3f rotationMatrixToEulerAngles(Eigen::Matrix3f &R){
        float sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
        bool singular = sy < 1e-6;
        float x, y, z;
        if (!singular)
        {
            x = atan2( R(2,1), R(2,2));
            y = atan2(-R(2,0), sy);
            z = atan2( R(1,0), R(0,0));
        }
        else
        {
            x = atan2(-R(1,2), R(1,1));
            y = atan2(-R(2,0), sy);
            z = 0;
        }
        return {x, y, z};
    }
};


#endif