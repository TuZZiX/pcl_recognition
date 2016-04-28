/*
 *
 * Created by sxt437 on 4/25/16.
 *
 */

#include <pcl_recognition/object_recognizer.h>


object_recognizer::object_recognizer( ros::NodeHandle &nodehandle ) : nh_( nodehandle ),
    model( new pcl::PointCloud<PointType> ), model_keypoints( new pcl::PointCloud<PointType> ),
    scene( new pcl::PointCloud<PointType> ), scene_keypoints( new pcl::PointCloud<PointType> ),
    model_normals( new pcl::PointCloud<NormalType> ), scene_normals( new pcl::PointCloud<NormalType> ),
    model_descriptors( new pcl::PointCloud<DescriptorType> ), scene_descriptors( new pcl::PointCloud<DescriptorType> ),
    pclKinect_ptr_( new pcl::PointCloud<PointType> ), rotated_model( new pcl::PointCloud<PointType> )
{
    model_ss_   = 0.01f;
    scene_ss_   = 0.03f;
    rf_rad_     = 0.015f;
    descr_rad_  = 0.02f;
    cg_size_    = 0.01f;
    cg_thresh_  = 5.0f;
    have_model  = false;
    have_scene  = false;

    got_kinect_cloud_  = false;
    show_keypoints     = true;
    show_correspondences = true;
    
    initialize_publishers();
    initialize_subscribers();
    timer = nh_.createTimer(ros::Duration(0.2), &object_recognizer::timerCB, this );

}


void object_recognizer::initialize_subscribers()
{
    pointcloud_subscriber_  = nh_.subscribe( "/kinect/depth/points", 1, &object_recognizer::kinectCB, this );
    real_kinect_subscriber_ = nh_.subscribe( "/camera/depth_registered/points", 1, &object_recognizer::kinectCB, this );
}


void object_recognizer::initialize_publishers()
{
    scene_publisher_    = nh_.advertise<sensor_msgs::PointCloud2>( "/scene", 1, true );
    model_publisher_    = nh_.advertise<sensor_msgs::PointCloud2>( "/model", 1, true );
    model_keypoints_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>( "/model_keypoints", 1, true );
    scene_keypoints_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>( "/scene_keypoints", 1, true );    
    rotated_model_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>( "/rotated_model", 1, true );
}


bool object_recognizer::set_model_cloud( std::string filename )
{
    if ( pcl::io::loadPCDFile<PointType> ( filename, *model ) == -1 ) /* * load the file */
    {
        ROS_ERROR( "Couldn't read file \n" );
        return(false);
    }
    have_model = true;
    return(true);
}


bool object_recognizer::set_scene_cloud( std::string filename )
{
    if ( pcl::io::loadPCDFile<PointType> ( filename, *scene ) == -1 ) /* * load the file */
    {
        ROS_ERROR( "Couldn't read file \n" );
        return(false);
    }
    have_scene = true;
    return(true);
}


void object_recognizer::use_kinect_scene()
{
    got_kinect_cloud_ = false;
    while(!got_kinect_cloud_) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    have_scene = true;
}

bool object_recognizer::recognize( std::vector<Eigen::Matrix3f> &rotation, std::vector<Eigen::Vector3f> &translation )
{
    if ( have_scene && have_model )
    {
        pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
        norm_est.setKSearch( 10 );
        norm_est.setInputCloud( model );
        norm_est.compute( *model_normals );

        norm_est.setInputCloud( scene );
        norm_est.compute( *scene_normals );

        /*
         *
         *  Downsample Clouds to Extract keypoints
         *
         */
        pcl::UniformSampling<PointType> uniform_sampling;
        uniform_sampling.setInputCloud( model );
        uniform_sampling.setRadiusSearch( model_ss_ );
        /* uniform_sampling.filter (*model_keypoints); */
        pcl::PointCloud<int> keypointIndices1;
        uniform_sampling.compute( keypointIndices1 );
        pcl::copyPointCloud( *model, keypointIndices1.points, *model_keypoints );
        std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;


        uniform_sampling.setInputCloud( scene );
        uniform_sampling.setRadiusSearch( scene_ss_ );
        /* uniform_sampling.filter (*scene_keypoints); */
        pcl::PointCloud<int> keypointIndices2;
        uniform_sampling.compute( keypointIndices2 );
        pcl::copyPointCloud( *scene, keypointIndices2.points, *scene_keypoints );
        std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

        /*
         *
         *  Compute Descriptor for keypoints
         *
         */
        pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
        descr_est.setRadiusSearch( descr_rad_ );

        descr_est.setInputCloud( model_keypoints );
        descr_est.setInputNormals( model_normals );
        descr_est.setSearchSurface( model );
        descr_est.compute( *model_descriptors );

        descr_est.setInputCloud( scene_keypoints );
        descr_est.setInputNormals( scene_normals );
        descr_est.setSearchSurface( scene );
        descr_est.compute( *scene_descriptors );

        /*
         *
         *  Find Model-Scene Correspondences with KdTree
         *
         */
        pcl::CorrespondencesPtr model_scene_corrs( new pcl::Correspondences() );

        pcl::KdTreeFLANN<DescriptorType> match_search;
        match_search.setInputCloud( model_descriptors );

        /*  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector. */
        for ( size_t i = 0; i < scene_descriptors->size(); ++i )
        {
            std::vector<int>    neigh_indices( 1 );
            std::vector<float>  neigh_sqr_dists( 1 );
            if ( !pcl_isfinite( scene_descriptors->at( i ).descriptor[0] ) )        /* skipping NaNs */
            {
                continue;
            }
            int found_neighs = match_search.nearestKSearch( scene_descriptors->at( i ), 1, neigh_indices, neigh_sqr_dists );
            if ( found_neighs == 1 && neigh_sqr_dists[0] < 0.25f )                  /*  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design) */
            {
                pcl::Correspondence corr( neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0] );
                model_scene_corrs->push_back( corr );
            }
        }
        std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

        /*
         *
         *  Actual Clustering
         *
         */

        /* std::vector<pcl::Correspondences> correspondences; */
        correspondences.clear();
        rototranslations.clear();
        /*
         *  Compute (Keypoints) Reference Frames only for Hough
         *
         */
        pcl::PointCloud<RFType>::Ptr    model_rf( new pcl::PointCloud<RFType> () );
        pcl::PointCloud<RFType>::Ptr    scene_rf( new pcl::PointCloud<RFType> () );

        pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
        rf_est.setFindHoles( true );
        rf_est.setRadiusSearch( rf_rad_ );

        rf_est.setInputCloud( model_keypoints );
        rf_est.setInputNormals( model_normals );
        rf_est.setSearchSurface( model );
        rf_est.compute( *model_rf );

        rf_est.setInputCloud( scene_keypoints );
        rf_est.setInputNormals( scene_normals );
        rf_est.setSearchSurface( scene );
        rf_est.compute( *scene_rf );

        /*  Clustering */
        pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
        clusterer.setHoughBinSize( cg_size_ );
        clusterer.setHoughThreshold( cg_thresh_ );
        clusterer.setUseInterpolation( true );
        clusterer.setUseDistanceWeight( false );

        clusterer.setInputCloud( model_keypoints );
        clusterer.setInputRf( model_rf );
        clusterer.setSceneCloud( scene_keypoints );
        clusterer.setSceneRf( scene_rf );
        clusterer.setModelSceneCorrespondences( model_scene_corrs );

        /* clusterer.cluster (correspondences); */
        clusterer.recognize( rototranslations, correspondences );

        rotated_model->points.clear();
        pcl::PointCloud<PointType>::Ptr temp_rotated_ptr( new pcl::PointCloud<PointType> );

        rotation.resize( rototranslations.size() );
        translation.resize( rototranslations.size() );
        std::cout << "Model instances found: " << rototranslations.size () << std::endl;
        if (rototranslations.size() == 0) {
            return(false);
        }
        for ( size_t i = 0; i < rototranslations.size(); ++i )
        {
            std::cout << "\nInstance " << i + 1 << ":" << std::endl;
            std::cout << "Correspondences belonging to this instance: " << correspondences[i].size () << std::endl;
            /* Print the rotation matrix and translation vector */
            rotation[i] = rototranslations[i].block<3, 3>( 0, 0 );
            translation[i]  = rototranslations[i].block<3, 1>( 0, 3 );
            pcl::transformPointCloud( *model, *temp_rotated_ptr, rototranslations[i] );
            for (int i = 0; i < temp_rotated_ptr->points.size(); ++i)
            {
                rotated_model->points.push_back(temp_rotated_ptr->points[i]);
            }
        }
    } else {
        ROS_INFO( "Please set model and scene first" );
        return(false);
    }
    return(true);
}


bool object_recognizer::recognize( std::vector<Eigen::Matrix3f> &rotation, std::vector<Eigen::Vector3f> &translation,
                   std::vector<pcl::Correspondences> &correspondences )
{
    if ( recognize( rotation, translation ) )
    {
        correspondences.resize( this->correspondences.size() );
        for ( int i = 0; i < correspondences.size(); ++i )
        {
            correspondences[i].resize( this->correspondences[i].size() );
            for ( int j = 0; j < this->correspondences[i].size(); ++j )
            {
                correspondences[i][j] = this->correspondences[i][j];
            }
        }
        return(true);
    } else {
        return(false);
    }
}


bool object_recognizer::find_best( Eigen::Matrix3f &rotation, Eigen::Vector3f &translation )
{
    std::vector<Eigen::Matrix3f>    temp_rotation;
    std::vector<Eigen::Vector3f>    temp_translation;

    int     index       = -1;
    double  best_norm   = 0.0;
    double  temp_norm   = 0.0;

    if ( recognize( temp_rotation, temp_translation ) ) {
        for ( int i = 0; i < correspondences.size(); ++i )
        {
            temp_norm = 0.0;
            for ( int j = 0; j < correspondences[i].size(); ++j )
            {
                temp_norm += pow( correspondences[i][j].weight, 2 );
            }
            temp_norm = sqrt( temp_norm );
            if ( best_norm <= temp_norm )
            {
                index       = i;
                best_norm   = temp_norm;
            }
        }
        if (index >= 0)
        {
            rotation    = temp_rotation[index];
            translation = temp_translation[index];
        }
        return(true);
    } else {
        return(false);
    }
}

bool object_recognizer::find_best( geometry_msgs::Pose &object_pose )
{
    Eigen::Matrix3f    temp_rotation;
    Eigen::Vector3f    temp_translation;
    if (find_best(temp_rotation, temp_translation)) {
        object_pose.orientation = rotation2quat(temp_rotation);
        object_pose.position.x = temp_translation[0];
        object_pose.position.y = temp_translation[1];
        object_pose.position.z = temp_translation[2];
    } else {
        return(false);
    }
}


void object_recognizer::pcl_visualize()
{
    pcl::visualization::PCLVisualizer viewer( "Correspondence Grouping" );
    viewer.addPointCloud( scene, "scene_cloud" );

    pcl::PointCloud<PointType>::Ptr off_scene_model( new pcl::PointCloud<PointType> () );
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints( new pcl::PointCloud<PointType> () );

    if ( show_correspondences || show_keypoints )
    {
/*  We are translating the model so that it doesn't end in the middle of the scene representation */
        pcl::transformPointCloud( *model, *off_scene_model, Eigen::Vector3f( -1, 0, 0 ), Eigen::Quaternionf( 1, 0, 0, 0 ) );
        pcl::transformPointCloud( *model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f( -1, 0, 0 ), Eigen::Quaternionf( 1, 0, 0, 0 ) );

        pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler( off_scene_model, 255, 255, 128 );
        viewer.addPointCloud( off_scene_model, off_scene_model_color_handler, "off_scene_model" );
    }

    if ( show_keypoints )
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler( scene_keypoints, 0, 0, 255 );
        viewer.addPointCloud( scene_keypoints, scene_keypoints_color_handler, "scene_keypoints" );
        viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints" );

        pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler( off_scene_model_keypoints, 0, 0, 255 );
        viewer.addPointCloud( off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints" );
        viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints" );
    }

    for ( size_t i = 0; i < rototranslations.size(); ++i )
    {
        pcl::PointCloud<PointType>::Ptr rotated_model( new pcl::PointCloud<PointType> () );
        pcl::transformPointCloud( *model, *rotated_model, rototranslations[i] );

        std::stringstream ss_cloud;
        ss_cloud << "instance" << i;

        pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler( rotated_model, 255, 0, 0 );
        viewer.addPointCloud( rotated_model, rotated_model_color_handler, ss_cloud.str() );

        if ( show_correspondences )
        {
            for ( size_t j = 0; j < correspondences[i].size(); ++j )
            {
                std::stringstream ss_line;
                ss_line << "correspondence_line" << i << "_" << j;
                PointType & model_point = off_scene_model_keypoints->at( correspondences[i][j].index_query );
                PointType & scene_point = scene_keypoints->at( correspondences[i][j].index_match );

                /*  We are drawing a line for each pair of clustered correspondences found between the model and the scene */
                viewer.addLine<PointType, PointType> ( model_point, scene_point, 0, 255, 0, ss_line.str() );
            }
        }
    }

    while ( !viewer.wasStopped() )
    {
        viewer.spinOnce();
        ros::spinOnce();
    }
}


void object_recognizer::kinectCB( const sensor_msgs::PointCloud2ConstPtr & cloud )
{
    if ( !got_kinect_cloud_ )
    {
        pcl::fromROSMsg( *cloud, *pclKinect_ptr_ );
        got_kinect_cloud_ = true; /* cue to "main" that callback received and saved a pointcloud */
    }
}


void object_recognizer::timerCB( const ros::TimerEvent & )
{
    if (have_scene)
    {
        pcl::toROSMsg(*scene, scene_cloud);
        scene_cloud.header.frame_id = "camera_depth_optical_frame";
        scene_cloud.header.stamp = ros::Time::now();
        scene_publisher_.publish(scene_cloud);
    }

    if (have_model)
    {
        pcl::toROSMsg(*model, model_cloud);
        model_cloud.header.frame_id = "camera_depth_optical_frame";
        model_cloud.header.stamp = ros::Time::now();
        model_publisher_.publish(model_cloud);
        //cout<<model->points.size()<<endl;
    }
    
    if (show_keypoints)
    {
        pcl::toROSMsg(*scene_keypoints, scene_keypoints_cloud);
        scene_keypoints_cloud.header.frame_id = "camera_depth_optical_frame";
        scene_keypoints_cloud.header.stamp = ros::Time::now();
        scene_keypoints_publisher_.publish(scene_keypoints_cloud);

        pcl::toROSMsg(*model_keypoints, model_keypoints_cloud);
        model_keypoints_cloud.header.frame_id = "camera_depth_optical_frame";
        model_keypoints_cloud.header.stamp = ros::Time::now();
        model_keypoints_publisher_.publish(model_keypoints_cloud);
    }

    if (rototranslations.size() > 0)
    {
        pcl::toROSMsg(*rotated_model, rotated_model_cloud);
        rotated_model_cloud.header.frame_id = "camera_depth_optical_frame";
        rotated_model_cloud.header.stamp = ros::Time::now();
        rotated_model_publisher_.publish(rotated_model_cloud);
    }

}

geometry_msgs::Quaternion object_recognizer::rotation2quat(Eigen::Matrix3f rotation) {
    // Output quaternion
    geometry_msgs::Quaternion quaternion;
    float w,x,y,z;
    // Determine which of w,x,y, or z has the largest absolute value
    float fourWSquaredMinus1 = rotation(0,0) + rotation(1,1) + rotation(2,2);
    float fourXSquaredMinus1 = rotation(0,0) - rotation(1,1) - rotation(2,2);
    float fourYSquaredMinus1 = rotation(1,1) - rotation(0,0) - rotation(2,2);
    float fourZSquaredMinus1 = rotation(2,2) - rotation(0,0) - rotation(1,1);

    int biggestIndex = 0;
    float fourBiggestSquaredMinus1 = fourWSquaredMinus1;

    if(fourXSquaredMinus1 > fourBiggestSquaredMinus1) {
        fourBiggestSquaredMinus1 = fourXSquaredMinus1;
        biggestIndex = 1;
    }
    if (fourYSquaredMinus1 > fourBiggestSquaredMinus1) {
        fourBiggestSquaredMinus1 = fourYSquaredMinus1;
        biggestIndex = 2;
    }
    if (fourZSquaredMinus1 > fourBiggestSquaredMinus1) {
        fourBiggestSquaredMinus1 = fourZSquaredMinus1;
        biggestIndex = 3;
    }
    // Per form square root and division
    float biggestVal = sqrt (fourBiggestSquaredMinus1 + 1.0f ) * 0.5f;
    float mult = 0.25f / biggestVal;

    // Apply table to compute quaternion values
    switch (biggestIndex) {
        case 0:
            w = biggestVal;
            x = (rotation(1,2) - rotation(2,1)) * mult;
            y = (rotation(2,0) - rotation(0,2)) * mult;
            z = (rotation(0,1) - rotation(1,0)) * mult;
            break;
        case 1:
            x = biggestVal;
            w = (rotation(1,2) - rotation(2,1)) * mult;
            y = (rotation(0,1) + rotation(1,0)) * mult;
            z = (rotation(2,0) + rotation(0,2)) * mult;
            break;
        case 2:
            y = biggestVal;
            w = (rotation(2,0) - rotation(0,2)) * mult;
            x = (rotation(0,1) + rotation(1,0)) * mult;
            z = (rotation(1,2) + rotation(2,1)) * mult;
            break;
        case 3:
            z = biggestVal;
            w = (rotation(0,1) - rotation(1,0)) * mult;
            x = (rotation(2,0) + rotation(0,2)) * mult;
            y = (rotation(1,2) + rotation(2,1)) * mult;
            break;
    }

    quaternion.x = x;
    quaternion.y = y;
    quaternion.z = z;
    quaternion.w = w;

    return quaternion;
}