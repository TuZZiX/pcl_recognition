//
// Created by sxt437 on 4/25/16.
//

#ifndef PCL_RECOGNITION_OBJECT_RECOGNIZER_H
#define PCL_RECOGNITION_OBJECT_RECOGNIZER_H

#include <pcl_recognition/pcl_recognition.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

class object_recognizer {
public:

    bool show_correspondences;
    bool show_keypoints;

    object_recognizer(ros::NodeHandle& nodehandle);

    geometry_msgs::Quaternion rotation2quat(Eigen::Matrix3f rotation);

    void set_model_cloud(pcl::PointCloud<PointType>::Ptr in_cloud) { pcl::copyPointCloud(*in_cloud, *model); have_model = true;}
    bool set_model_cloud(std::string filename);
    void set_scene_cloud(pcl::PointCloud<PointType>::Ptr in_cloud) { pcl::copyPointCloud(*in_cloud, *scene); have_scene = true;}
    bool set_scene_cloud(std::string filename);
    void use_kinect_scene();

    void set_model_ss(double model_ss) {model_ss_ = model_ss;}
    void set_scene_ss(double scene_ss) {scene_ss_ = scene_ss;}
    void set_rf_rad(double rf_rad) {rf_rad_ = rf_rad;}
    void set_descr_rad(double descr_rad) {descr_rad_ = descr_rad;}
    void set_cg_size(double cg_size) {cg_size_ = cg_size;}
    void set_cg_thresh(double cg_thresh) {cg_thresh_ = cg_thresh;}

    bool recognize(std::vector<Eigen::Matrix3f> &rotation, std::vector<Eigen::Vector3f> &translation, std::vector<pcl::Correspondences> &correspondences);
    bool recognize(std::vector<Eigen::Matrix3f> &rotation, std::vector<Eigen::Vector3f> &translation);
    bool find_best(Eigen::Matrix3f &rotation, Eigen::Vector3f &translation);
    bool find_best(geometry_msgs::Pose &object_pose);

    void pcl_visualize();

private:
    ros::NodeHandle nh_;

    pcl::PointCloud<PointType>::Ptr model;
    pcl::PointCloud<PointType>::Ptr model_keypoints;
    pcl::PointCloud<PointType>::Ptr scene;
    pcl::PointCloud<PointType>::Ptr scene_keypoints;
    pcl::PointCloud<NormalType>::Ptr model_normals;
    pcl::PointCloud<NormalType>::Ptr scene_normals;
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors;
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors;

    pcl::PointCloud<PointType>::Ptr pclKinect_ptr_;
    pcl::PointCloud<PointType>::Ptr rotated_model;

    std::vector<pcl::Correspondences> correspondences;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;

    ros::Timer timer;

    ros::Publisher scene_publisher_;
    ros::Publisher model_publisher_;
    ros::Publisher scene_keypoints_publisher_;
    ros::Publisher model_keypoints_publisher_;
    ros::Publisher rotated_model_publisher_;

    ros::Subscriber pointcloud_subscriber_;
    ros::Subscriber real_kinect_subscriber_;

    sensor_msgs::PointCloud2 scene_cloud;
    sensor_msgs::PointCloud2 model_cloud;
    sensor_msgs::PointCloud2 scene_keypoints_cloud;
    sensor_msgs::PointCloud2 model_keypoints_cloud;
    sensor_msgs::PointCloud2 rotated_model_cloud;

    double model_ss_;
    double scene_ss_;
    double rf_rad_;
    double descr_rad_;
    double cg_size_;
    double cg_thresh_;

    bool have_model;
    bool have_scene;

    bool got_kinect_cloud_;

    void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void timerCB(const ros::TimerEvent&);

    void initialize_publishers();
    void initialize_subscribers();
    
};


#endif //PCL_RECOGNITION_OBJECT_RECOGNIZER_H
