//* take snapshot selest points, dele tpoints *****************//

#include <pcl_recognition/pcl_recognition.h>

const double DISTANCE_TOLERANCE = pow(0.002, 2);

bool display_kinect = false;
sensor_msgs::PointCloud2 ros_cloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr pclSelected_ptr(new pcl::PointCloud<pcl::PointXYZ>); //pointer for color version of pointcloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclEditing_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
ros::Publisher pubCloud;

void timerCallback(const ros::TimerEvent&)
{
    if (display_kinect) {
        pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud);
        pubCloud.publish(ros_cloud);
    } else {
        pclEditing_ptr->header.frame_id = "camera_depth_optical_frame";
        pcl::toROSMsg(*pclEditing_ptr, ros_cloud);
        pubCloud.publish(ros_cloud);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_edit_tool"); //node name
    ros::NodeHandle nh;
    if (argc < 2) {
        ROS_INFO("Please input the pcd name!");
        return -1;
    }

    PclUtils utils(&nh);
    pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("editing_cloud", 1);
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), timerCallback);

    //spin until obtain a snapshot
    ROS_INFO("waiting for kinect data...");
    while (!utils.got_kinect_cloud()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    utils.get_kinect_points(pclKinect_clr_ptr);
    display_kinect = true;
    ros::spinOnce();
    ROS_INFO("Please select point cloud in \'/camera/depth_registered/points\' topic using \'Publish Selected points\'");

    double x, y, z;
    while (!utils.got_selected_points()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    utils.reset_got_selected_points();
    utils.get_selected_points(pclSelected_ptr);
    for (int i = 0; i < pclSelected_ptr->points.size(); ++i) {
        for (int j = 0; j < pclKinect_clr_ptr->points.size(); ++j) {
            x = pow(pclKinect_clr_ptr->points[j].x - pclSelected_ptr->points[i].x, 2);
            y = pow(pclKinect_clr_ptr->points[j].y - pclSelected_ptr->points[i].y, 2);
            z = pow(pclKinect_clr_ptr->points[j].z - pclSelected_ptr->points[i].z, 2);
            if (sqrt(x + y +z ) < DISTANCE_TOLERANCE) {
                pclEditing_ptr->points.push_back(pclKinect_clr_ptr->points[j]);
                break;
            }
        }
    }
    ROS_INFO("snapshot with points %d; saving to file %s", (int)pclEditing_ptr->points.size(), argv[1]);
    pcl::io::savePCDFile(argv[1], *pclEditing_ptr, true);
    display_kinect = false;
    ros::spinOnce();

    string input;
    while (ros::ok()) {
        cout<<"+ to add points to cloud, - to delete cloud, r to reset cloud, q to exit: ";
        cin >> input;
        if (input.compare("+") == 0) {
        	ROS_INFO("Please select point cloud in \'/camera/depth_registered/points\' topic using \'Publish Selected points\'");
            while (!utils.got_selected_points()) {
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }
            utils.reset_got_selected_points();
            utils.get_selected_points(pclSelected_ptr);
            temp_ptr->points.clear();
            for (int i = 0; i < pclSelected_ptr->points.size(); ++i) {
                for (int j = 0; j < pclKinect_clr_ptr->points.size(); ++j) {
                    x = pow(pclKinect_clr_ptr->points[j].x - pclSelected_ptr->points[i].x, 2);
                    y = pow(pclKinect_clr_ptr->points[j].y - pclSelected_ptr->points[i].y, 2);
                    z = pow(pclKinect_clr_ptr->points[j].z - pclSelected_ptr->points[i].z, 2);
                    if (x + y +z < DISTANCE_TOLERANCE) {
                        temp_ptr->points.push_back(pclKinect_clr_ptr->points[j]);
                        break;
                    }
                }
            }
            bool in_cloud = false;
            for (int i = 0; i < temp_ptr->points.size(); ++i) {
                for (int j = 0; j < pclEditing_ptr->points.size(); ++j) {
                    x = pow(pclEditing_ptr->points[j].x - temp_ptr->points[i].x, 2);
                    y = pow(pclEditing_ptr->points[j].y - temp_ptr->points[i].y, 2);
                    z = pow(pclEditing_ptr->points[j].z - temp_ptr->points[i].z, 2);
                    if (x + y +z < DISTANCE_TOLERANCE) {
                        in_cloud = true;
                        break;
                    }
                }
                if (!in_cloud) {
                    pclEditing_ptr->points.push_back(temp_ptr->points[i]);
                }
                in_cloud = false;
            }
            ROS_INFO("snapshot with points %d; saving to file %s", (int)pclEditing_ptr->points.size(), argv[1]);
            pcl::io::savePCDFile(argv[1], *pclEditing_ptr, true);
        } else if (input.compare("-") == 0) {
        	ROS_INFO("Please select point cloud in \'/editing_cloud\' topic using \'Publish Selected points\'");
            while (!utils.got_selected_points()) {
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }
            utils.reset_got_selected_points();
            utils.get_selected_points(pclSelected_ptr);
            bool in_cloud = false;
            temp_ptr->points.clear();
            for (int i = 0; i < pclEditing_ptr->points.size(); ++i) {
                for (int j = 0; j < pclSelected_ptr->points.size(); ++j) {
                    x = pow(pclEditing_ptr->points[i].x - pclSelected_ptr->points[j].x, 2);
                    y = pow(pclEditing_ptr->points[i].y - pclSelected_ptr->points[j].y, 2);
                    z = pow(pclEditing_ptr->points[i].z - pclSelected_ptr->points[j].z, 2);
                    if (x + y + z < DISTANCE_TOLERANCE) {
                        in_cloud = true;
                        break;
                    }
                }
                if (!in_cloud) {
                    temp_ptr->points.push_back(pclEditing_ptr->points[i]);
                }
                in_cloud = false;
            }
            pcl::copyPointCloud(*temp_ptr, *pclEditing_ptr);
            ROS_INFO("snapshot with points %d; saving to file %s", (int)pclEditing_ptr->points.size(), argv[1]);
            pcl::io::savePCDFile(argv[1], *pclEditing_ptr, true);
        } else if (input.compare("r") == 0 || input.compare("R") == 0) {
            ROS_INFO("waiting for kinect data...");
            
            utils.reset_got_kinect_cloud();
            while (!utils.got_kinect_cloud()) {
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }
            utils.get_kinect_points(pclKinect_clr_ptr);
            display_kinect = true;
            ros::spinOnce();

            //pcl::copyPointCloud(*pclKinect_clr_ptr, *pclEditing_ptr);
            ROS_INFO("Please select point cloud in \'/camera/depth_registered/points\' topic using \'Publish Selected points\'");
            while (!utils.got_selected_points()) {
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }
            utils.reset_got_selected_points();
            utils.get_selected_points(pclSelected_ptr);
            pclEditing_ptr->points.clear();
            for (int i = 0; i < pclSelected_ptr->points.size(); ++i) {
                for (int j = 0; j < pclKinect_clr_ptr->points.size(); ++j) {
                    x = pow(pclKinect_clr_ptr->points[j].x - pclSelected_ptr->points[i].x, 2);
                    y = pow(pclKinect_clr_ptr->points[j].y - pclSelected_ptr->points[i].y, 2);
                    z = pow(pclKinect_clr_ptr->points[j].z - pclSelected_ptr->points[i].z, 2);
                    if (sqrt(x + y +z ) < DISTANCE_TOLERANCE) {
                        pclEditing_ptr->points.push_back(pclKinect_clr_ptr->points[j]);
                        break;
                    }
                }
            }
            ROS_INFO("snapshot with points %d; saving to file %s", (int)pclEditing_ptr->points.size(), argv[1]);
            pcl::io::savePCDFile(argv[1], *pclEditing_ptr, true);
            display_kinect = false;
            ros::spinOnce();
        } else if (input.compare("q") == 0 || input.compare("Q") == 0) {
            ROS_INFO("Quiting...");
            ros::spinOnce();
            return 0;   
        }
        ros::spinOnce();
    }
    return 0;
}