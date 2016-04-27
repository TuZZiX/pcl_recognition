//
// Created by sxt437 on 4/27/16.
//

#include <pcl_recognition/pcl_recognition.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/TableArray.h>
#include <visualization_msgs/MarkerArray.h>

std::vector<double> euler(double mData[4]) {
    std::vector<double> euler(3);
    const static double PI_OVER_2 = M_PI * 0.5;
    const static double EPSILON = 1e-10;
    double sqw, sqx, sqy, sqz;

    // quick conversion to Euler angles to give tilt to user
    sqw = mData[3]*mData[3];
    sqx = mData[0]*mData[0];
    sqy = mData[1]*mData[1];
    sqz = mData[2]*mData[2];

    euler[1] = asin(2.0 * (mData[3]*mData[1] - mData[0]*mData[2]));
    if (PI_OVER_2 - fabs(euler[1]) > EPSILON) {
        euler[2] = atan2(2.0 * (mData[0]*mData[1] + mData[3]*mData[2]),
                         sqx - sqy - sqz + sqw);
        euler[0] = atan2(2.0 * (mData[3]*mData[0] + mData[1]*mData[2]),
                         sqw - sqx - sqy + sqz);
    } else {
        // compute heading from local 'down' vector
        euler[2] = atan2(2*mData[1]*mData[2] - 2*mData[0]*mData[3],
                         2*mData[0]*mData[2] + 2*mData[1]*mData[3]);
        euler[0] = 0.0;

        // If facing down, reverse yaw
        if (euler[1] < 0)
            euler[2] = M_PI - euler[2];
    }
    return euler;
}

void objectCallback(const object_recognition_msgs::RecognizedObjectArray objects_msg) {
    ROS_WARN("Total %d objects found", (int)objects_msg.objects.size());
    for (int i = 0; i < objects_msg.objects.size(); ++i) {
        double quat[4];
        quat[0] = objects_msg.objects[i].pose.pose.pose.orientation.x;
        quat[1] = objects_msg.objects[i].pose.pose.pose.orientation.y;
        quat[2] = objects_msg.objects[i].pose.pose.pose.orientation.z;
        quat[3] = objects_msg.objects[i].pose.pose.pose.orientation.w;
        std::vector<double> eul = euler(quat);
        ROS_INFO("Object %d: ID in CouchDB is %s", i+1, objects_msg.objects[i].type.key.c_str());
        ROS_INFO("Object %d: Similarity of the object to coke is: %f ", i+1, objects_msg.objects[i].confidence);
        ROS_INFO("Object %d: object pose x is: %f", i+1, objects_msg.objects[i].pose.pose.pose.position.x);
        ROS_INFO("Object %d: object pose y is: %f", i+1, objects_msg.objects[i].pose.pose.pose.position.y);
        ROS_INFO("Object %d: object pose z is: %f", i+1, objects_msg.objects[i].pose.pose.pose.position.z);
        ROS_INFO("Object %d: object pose roll is: %f", i+1, eul[0]);
        ROS_INFO("Object %d: object pose pitch is: %f", i+1, eul[1]);
        ROS_INFO("Object %d: object pose yaw is: %f", i+1, eul[2]);
        ROS_INFO("---------------------------------");
    }
}

void planeCallback(const object_recognition_msgs::TableArray plane_msg) {
    ROS_WARN("Total %d planes found", (int)plane_msg.tables.size());
    for (int i = 0; i < plane_msg.tables.size(); ++i) {
        double quat[4];
        quat[0] = plane_msg.tables[i].pose.orientation.x;
        quat[1] = plane_msg.tables[i].pose.orientation.y;
        quat[2] = plane_msg.tables[i].pose.orientation.z;
        quat[3] = plane_msg.tables[i].pose.orientation.w;
        std::vector<double> eul = euler(quat);
        ROS_INFO("Plane %d: plane pose x is: %f", i+1, plane_msg.tables[i].pose.position.x);
        ROS_INFO("Plane %d: plane pose y is: %f", i+1, plane_msg.tables[i].pose.position.y);
        ROS_INFO("Plane %d: plane pose z is: %f", i+1, plane_msg.tables[i].pose.position.z);
        ROS_INFO("Plane %d: plane pose roll is: %f", i+1, eul[0]);
        ROS_INFO("Plane %d: plane pose pitch is: %f", i+1, eul[1]);
        ROS_INFO("Plane %d: plane pose yaw is: %f", i+1, eul[2]);
        ROS_INFO("---------------------------------");
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_recognition_kitchen"); //node name
    ros::NodeHandle nh;
    ros::Subscriber object_sub = nh.subscribe("/recognized_object_array", 1, &objectCallback);
    ros::Subscriber plane_sub = nh.subscribe("/table_array", 1, &planeCallback);
    ros::Duration loop_timer(2.5);
    while (ros::ok()) {
        ros::spinOnce();
        loop_timer.sleep();
    }
    return 0;
}