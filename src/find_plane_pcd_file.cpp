//find_plane_pcd_file.cpp
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
// can select a patch; then computes a plane containing that patch, which is published on topic "planar_pts"
// illustrates use of PCL methods: computePointNormal(), transformPointCloud(), 
// pcl::PassThrough methods setInputCloud(), setFilterFieldName(), setFilterLimits, filter()
// pcl::io::loadPCDFile() 
// pcl::toROSMsg() for converting PCL pointcloud to ROS message
// voxel-grid filtering: pcl::VoxelGrid,  setInputCloud(), setLeafSize(), filter()
//wsn March 2016

#include <pcl_recognition/pcl_recognition.h>  //a local library with some utility fncs

#define sq(a)   (pow(a,2))

PclUtils *g_pcl_utils_ptr;


void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr,
                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr, vector<int> &indices) {

    float curvature;
    Eigen::Vector4f plane_parameters;

    pcl::computePointNormal(*patch_cloud_ptr, plane_parameters, curvature); //pcl fnc to compute plane fit to point cloud
    cout << "PCL: plane params of patch: " << plane_parameters.transpose() << endl;

    //next, define a coordinate frame on the plane fitted to the patch.
    // choose the z-axis of this frame to be the plane normal--but enforce that the normal
    // must point towards the camera
    Eigen::Affine3f A_plane_wrt_camera;
    // here, use a utility function in pclUtils to construct a frame on the computed plane
    A_plane_wrt_camera = g_pcl_utils_ptr->make_affine_from_plane_params(plane_parameters);
    cout << "A_plane_wrt_camera rotation:" << endl;
    cout << A_plane_wrt_camera.linear() << endl;
    cout << "origin: " << A_plane_wrt_camera.translation().transpose() << endl;

    //next, transform all points in input_cloud into the plane frame.
    //the result of this is, points that are members of the plane of interest should have z-coordinates
    // nearly 0, and thus these points will be easy to find
    cout << "transforming all points to plane coordinates..." << endl;
    //Transform each point in the given point cloud according to the given transformation.
    //pcl fnc: pass in ptrs to input cloud, holder for transformed cloud, and desired transform
    //note that if A contains a description of the frame on the plane, we want to xform with inverse(A)
    pcl::transformPointCloud(*input_cloud_ptr, *transformed_cloud_ptr, A_plane_wrt_camera.inverse());

    //now we'll use some functions from the pcl filter library;
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object
    pass.setInputCloud(transformed_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(-0.02, 0.02); //here is the range: z value near zero, -0.02<z<0.02
    pass.filter(indices); //  this will return the indices of the points in   transformed_cloud_ptr that pass our test
    cout << "number of points passing the filter = " << indices.size() << endl;
    //This fnc populates the reference arg "indices", so the calling fnc gets the list of interesting points
}

void sphare_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr,
                    vector<int> &indices, vector<int> &filtered_indices, double &centroid_indices, double filter_range) {
    // find largest segment in the indices
    double sq_range = sq(filter_range);
    double best_indices = -1;
    int best_cnt = -1;
    double x, y, z;

    Eigen::Vector3f centroid = Eigen::MatrixXf::Zero(3, 1);
    for (int i = 0; i < patch_cloud_ptr->points.size(); ++i) {
        centroid += patch_cloud_ptr->points[i].getVector3fMap();
    }
    centroid /= patch_cloud_ptr->points.size();

    for (int i = 0; i < indices.size(); ++i) {
        int current_cnt = 0;
        x = sq(centroid[0] - input_cloud_ptr->points[indices[i]].x);
        y = sq(centroid[1] - input_cloud_ptr->points[indices[i]].y);
        z = sq(centroid[2] - input_cloud_ptr->points[indices[i]].z);
        if ((x + y + z) <= sq_range) {
            for (int j = 0; j < indices.size(); ++j) {
                x = sq(input_cloud_ptr->points[indices[i]].x - input_cloud_ptr->points[indices[j]].x);
                y = sq(input_cloud_ptr->points[indices[i]].y - input_cloud_ptr->points[indices[j]].y);
                z = sq(input_cloud_ptr->points[indices[i]].z - input_cloud_ptr->points[indices[j]].z);
                // cout << "x:" << x << ", y:" << y << ", z:" << z << ", sum:" << x+y+z << endl;
                if ((x + y + z) <= sq_range) {
                    current_cnt++;
                }
            }
            if (current_cnt > best_cnt) {
                best_indices = indices[i];
                best_cnt = current_cnt;
            }
        }
    }
    //cout << "the best indices is: " << best_indices << endl;
    filtered_indices.clear();
    for (int k = 0; k < indices.size(); ++k) {
        //x = sq(transformed_cloud_ptr->points[best_indices].x - transformed_cloud_ptr->points[indices[k]].x);
        //y = sq(transformed_cloud_ptr->points[best_indices].y - transformed_cloud_ptr->points[indices[k]].y);
        x = sq(input_cloud_ptr->points[best_indices].x - input_cloud_ptr->points[indices[k]].x);
        y = sq(input_cloud_ptr->points[best_indices].y - input_cloud_ptr->points[indices[k]].y);
        z = sq(input_cloud_ptr->points[best_indices].z - input_cloud_ptr->points[indices[k]].z);
        if ((x + y) <= sq_range) {
            filtered_indices.push_back(indices[k]);
        }
    }
    cout << "number of points after sphare filter = " << filtered_indices.size() << endl;
    centroid_indices = best_indices;
}

void find_normal_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, vector<int> &indices,
                    Eigen::Vector3f &plane_normal, Eigen::Vector3f &plane_centroid, double &plane_distance) {
    float curvature;
    Eigen::Vector4f plane_parameters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

    pcl::copyPointCloud(*input_cloud_ptr, indices, *temp_cloud_ptr); //extract these pts into new cloud
    pcl::computePointNormal(*temp_cloud_ptr, plane_parameters, curvature); //pcl fnc to compute plane fit to point cloud
    plane_normal[0] = plane_parameters[0];
    plane_normal[1] = plane_parameters[1];
    plane_normal[2] = plane_parameters[2];
    plane_distance = plane_parameters[3];

    Eigen::Affine3f A_plane_wrt_camera;
    // here, use a utility function in pclUtils to construct a frame on the computed plane
    A_plane_wrt_camera = g_pcl_utils_ptr->make_affine_from_plane_params(plane_parameters);
    Eigen::Vector3f camera_pos = Eigen::MatrixXf::Zero(3, 1);
    camera_pos = A_plane_wrt_camera.inverse() * camera_pos;

    plane_centroid = Eigen::MatrixXf::Zero(3, 1);
    for (int i = 0; i < temp_cloud_ptr->points.size(); ++i) {
        plane_centroid += temp_cloud_ptr->points[i].getVector3fMap();
    }
    plane_centroid /= temp_cloud_ptr->points.size();

    cout << "Normal of plane: " << plane_normal.transpose() << endl;
    cout << "Centroid of plane: " << plane_centroid.transpose() << endl;
    cout << "Camera height related to plane: " << fabs(plane_distance) << endl;

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_finder"); //node name
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for pointcloud of planar points found
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //ptr to selected pts from Rvis tool
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to filtered cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

    vector<int> indices, filterd_indices;
    Eigen::Vector3f plane_normal;
    Eigen::Vector3f plane_centroid;
    double plane_distance;
    double center_indices;
    double filter_range = 0.195;
    double voxel_size = 0.01f;
    string fname;

    int selection;

    if (argc < 2) {
        cout << "Please input the filename of PCD file" << fname << endl;
        return 0;
    } else {
        fname = argv[1];
        cout << "Load PCD file: " << fname <<endl;
    }
    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclKinect_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubPlane = nh.advertise<sensor_msgs::PointCloud2> ("planar_pts", 1);
    ros::Publisher pubFilter = nh.advertise<sensor_msgs::PointCloud2> ("filtered_planar_pts", 1);
    ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);

    sensor_msgs::PointCloud2 ros_cloud, ros_planar_cloud, downsampled_cloud, filtered_cloud; //here are ROS-compatible messages
    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud); //convert from PCL cloud to ROS message this way
    cout << "num bytes in original cloud data = " << pclKinect_clr_ptr->points.size() << endl;

    //use voxel filtering to downsample the original cloud:
    cout << "voxel filtering to size: "<< voxel_size << endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(pclKinect_clr_ptr);
    vox.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox.filter(*downsampled_kinect_ptr);

    cout << "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() << endl; // ->data.size()<<endl;    
    pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud); //convert to ros message for publication and display

    PclUtils pclUtils(&nh); //instantiate a PclUtils object--a local library w/ some handy fncs
    g_pcl_utils_ptr = &pclUtils; // make this object shared globally, so above fnc can use it too

    cout << "Load profile: say 1 to search for stool, and say 2 to search for coke can" << endl << "Profile: ";
    cin >> selection;
    if (selection == 1) {
        filter_range = 0.195;
    } else if (selection == 2) {
        filter_range = 0.1;
    } else {
        cout << "Wrong input!" <<endl;
        return 0;
    }
    cout << " select a patch of points to find corresponding plane..." << endl; //prompt user action

    //loop to test for new selected-points inputs and compute and display corresponding planar fits 
    while (ros::ok()) {
        if (pclUtils.got_selected_points()) { //here if user selected a new patch of points
            pclUtils.reset_got_selected_points(); // reset for a future trigger
            pclUtils.get_copy_selected_points(selected_pts_cloud_ptr); //get a copy of the selected points
            cout << "got new patch with number of selected pts = " << selected_pts_cloud_ptr->points.size() << endl;

            //find pts coplanar w/ selected patch, using PCL methods in above-defined function
            //"indices" will get filled with indices of points that are approx co-planar with the selected patch
            // can extract indices from original cloud, or from voxel-filtered (down-sampled) cloud
            //find_indices_of_plane_from_patch(pclKinect_clr_ptr, selected_pts_cloud_ptr, indices);
            find_indices_of_plane_from_patch(downsampled_kinect_ptr, selected_pts_cloud_ptr, transformed_cloud_ptr, indices);
            sphare_filter(downsampled_kinect_ptr, selected_pts_cloud_ptr, indices, filterd_indices, center_indices, filter_range);
            find_normal_centroid(downsampled_kinect_ptr, filterd_indices,  plane_normal, plane_centroid, plane_distance);

            pcl::copyPointCloud(*downsampled_kinect_ptr, indices, *plane_pts_ptr); //extract these pts into new cloud
            //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
            pcl::toROSMsg(*plane_pts_ptr, ros_planar_cloud); //convert to ros message for publication and display

            pcl::copyPointCloud(*downsampled_kinect_ptr, filterd_indices, *filtered_pts_ptr); //extract these pts into new cloud
            pcl::toROSMsg(*filtered_pts_ptr, filtered_cloud); //convert to ros message for publication and display
        }
        pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
        pubPlane.publish(ros_planar_cloud); // display the set of points computed to be coplanar w/ selection
        pubDnSamp.publish(downsampled_cloud); //can directly publish a pcl::PointCloud2!!
        pubFilter.publish(filtered_cloud);
        ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        ros::Duration(0.1).sleep();
    }

    return 0;
}
