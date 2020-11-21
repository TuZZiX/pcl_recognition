/* created 25 April 2016 by Shipei Tian and Ran Hao, 
   for object recognition in cwru robotics lab
   from pcl corresponding_group.cpp using "hough"
   this is a test main, parameters should be tuned in differnt cases
   */

#include <pcl_recognition/ObjectRecognizer.h>
#include <ros/package.h> 

string test_model;
string test_scene;

std::string find_in_package(std::string filename) {
	string pkg_path = ros::package::getPath("pcl_recognition");
	string append = "/pcd/";
	string full_path = pkg_path+append+filename;
	return full_path;
}
void load_coke1_parameters(ObjectRecognizer& object)
{
    // the paramters for coke_bad and coke_scene
    test_model = "coke_bad.pcd";
    test_scene = "coke_scene.pcd";

    object.set_model_ss(0.01);
    object.set_scene_ss(0.01);
    object.set_cg_size(0.09);
    object.set_cg_thresh(2.9);
}


void load_coke2_parameters(ObjectRecognizer& object)
{
    //the parameter for coke_2 and new_coke
    test_model = "new_coke.pcd";
    test_scene = "coke_2.pcd";

    object.set_model_ss(0.01);
    object.set_scene_ss(0.01);
    object.set_cg_size(0.13);
    object.set_cg_thresh(45);
}

void load_milk_parameters(ObjectRecognizer& object)
{
    //the parameter for milk from PCL example
    test_model = "milk.pcd";
    test_scene = "milk_cartoon_all_small_clorox.pcd";

    object.set_model_ss(0.01);
    object.set_scene_ss(0.01);
    object.set_cg_size(0.13);
    object.set_cg_thresh(45);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ObjectRecognizer"); //node name
    ros::NodeHandle nh;
    
    ObjectRecognizer object(nh);

    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;

    if (argc >= 2)
    {
    	if (string(argv[1]) == "1") {
    		load_coke1_parameters(object);
    	} else if (string(argv[1]) == "2") {
    		load_coke2_parameters(object);
    	} else {
            load_milk_parameters(object);
    	}
    } else {
        // use kinect camera
        load_coke2_parameters(object);
        test_scene = "";
    }

    if (!object.set_model_cloud(find_in_package(test_model)))
    {
    	ROS_INFO("NO MODEL PCD FILE FOUND!");
    	return -1;
    }

    if (!object.set_scene_cloud(find_in_package(test_scene)))
    {
    	ROS_INFO("NO SCENE PCD FILE FOUND! USE INPUT FROM KINECT CAMERA INSTEAD");
        ROS_INFO("Please make sure kinect camera is published to /kinect/depth/points or /camera/depth_registered/points");
        object.use_kinect_scene();
        ROS_INFO("Got scene from kinect camera");
    }

    ROS_INFO("In rviz, please select fixed frame = \'camera_depth_optical_frame\'");
 
	//get best correspondence rotation and translation  
    if (object.find_best(rotation, translation))
    {
    	printf ("Best rotation matrix and translation vector are: \n");
	    printf ("\n");
	    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
	    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
	    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
	    printf ("\n");
	    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
        object.pcl_visualize();
    } else {
    	ROS_INFO("No object found!");
    }

    ros::spin();

}