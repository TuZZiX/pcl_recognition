//
// Created by sxt437 on 4/25/16.
//

#include <pcl_recognition/object_recognizer.h>


object_recognizer::object_recognizer(ros::NodeHandle &nodehandle) : nh_(nodehandle),
                                                                    model (new pcl::PointCloud<PointType>), model_keypoints (new pcl::PointCloud<PointType>),
                                                                    scene (new pcl::PointCloud<PointType>), scene_keypoints (new pcl::PointCloud<PointType>),
                                                                    model_normals (new pcl::PointCloud<NormalType>), scene_normals (new pcl::PointCloud<NormalType>),
                                                                    model_descriptors (new pcl::PointCloud<DescriptorType>), scene_descriptors (new pcl::PointCloud<DescriptorType>) {
    model_ss_ =  0.01f;
    scene_ss_ = 0.03f;
    rf_rad_ = 0.015f;
    descr_rad_ = 0.02f;
    cg_size_ = 0.01f;
    cg_thresh_ = 5.0f;
    have_model = false;
    have_scene = false;
}

bool object_recognizer::set_model_cloud(std::string filename) {
    if (pcl::io::loadPCDFile<PointType> (filename, *model) == -1) //* load the file
    {
        ROS_ERROR ("Couldn't read file \n");
        return false;
    }
    have_model = true;
    return true;
}

bool object_recognizer::set_scene_cloud(std::string filename) {
    if (pcl::io::loadPCDFile<PointType> (filename, *scene) == -1) //* load the file
    {
        ROS_ERROR ("Couldn't read file \n");
        return false;
    }
    have_scene = true;
    return true;
}

void object_recognizer::use_kinect_scene() {
    have_scene = true;
}

void object_recognizer::load_coke1_parameters() {
    model_ss_ =  0.01f;
    scene_ss_ = 0.03f;
    rf_rad_ = 0.015f;
    descr_rad_ = 0.02f;
    cg_size_ = 0.01f;
    cg_thresh_ = 5.0f;
}

void object_recognizer::load_coke2_parameters() {
    model_ss_ =  0.01f;
    scene_ss_ = 0.03f;
    rf_rad_ = 0.015f;
    descr_rad_ = 0.02f;
    cg_size_ = 0.01f;
    cg_thresh_ = 5.0f;
}

bool object_recognizer::recognize(std::vector<Eigen::Matrix3f> rotation, std::vector<Eigen::Vector3f> translation,
                                  std::vector<pcl::Correspondences> correspondences) {
    if (have_scene && have_model) {
        pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
        norm_est.setKSearch (10);
        norm_est.setInputCloud (model);
        norm_est.compute (*model_normals);

        norm_est.setInputCloud (scene);
        norm_est.compute (*scene_normals);

        //
        //  Downsample Clouds to Extract keypoints
        //
        pcl::UniformSampling<PointType> uniform_sampling;
        uniform_sampling.setInputCloud (model);
        uniform_sampling.setRadiusSearch (model_ss_);
        //uniform_sampling.filter (*model_keypoints);
        pcl::PointCloud<int> keypointIndices1;
        uniform_sampling.compute(keypointIndices1);
        pcl::copyPointCloud(*model, keypointIndices1.points, *model_keypoints);
        std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;


        uniform_sampling.setInputCloud (scene);
        uniform_sampling.setRadiusSearch (scene_ss_);
        //uniform_sampling.filter (*scene_keypoints);
        pcl::PointCloud<int> keypointIndices2;
        uniform_sampling.compute(keypointIndices2);
        pcl::copyPointCloud(*scene, keypointIndices2.points, *scene_keypoints);
        std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

        //
        //  Compute Descriptor for keypoints
        //
        pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
        descr_est.setRadiusSearch (descr_rad_);

        descr_est.setInputCloud (model_keypoints);
        descr_est.setInputNormals (model_normals);
        descr_est.setSearchSurface (model);
        descr_est.compute (*model_descriptors);

        descr_est.setInputCloud (scene_keypoints);
        descr_est.setInputNormals (scene_normals);
        descr_est.setSearchSurface (scene);
        descr_est.compute (*scene_descriptors);

        //
        //  Find Model-Scene Correspondences with KdTree
        //
        pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

        pcl::KdTreeFLANN<DescriptorType> match_search;
        match_search.setInputCloud (model_descriptors);

        //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
        for (size_t i = 0; i < scene_descriptors->size (); ++i)
        {
            std::vector<int> neigh_indices (1);
            std::vector<float> neigh_sqr_dists (1);
            if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
            {
                continue;
            }
            int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
            if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
            {
                pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
                model_scene_corrs->push_back (corr);
            }
        }
        std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

        //
        //  Actual Clustering
        //
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
        //std::vector<pcl::Correspondences> clustered_corrs;
        correspondences.clear();
        //  Compute (Keypoints) Reference Frames only for Hough
        //
        pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
        pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

        pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
        rf_est.setFindHoles (true);
        rf_est.setRadiusSearch (rf_rad_);

        rf_est.setInputCloud (model_keypoints);
        rf_est.setInputNormals (model_normals);
        rf_est.setSearchSurface (model);
        rf_est.compute (*model_rf);

        rf_est.setInputCloud (scene_keypoints);
        rf_est.setInputNormals (scene_normals);
        rf_est.setSearchSurface (scene);
        rf_est.compute (*scene_rf);

        //  Clustering
        pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
        clusterer.setHoughBinSize (cg_size_);
        clusterer.setHoughThreshold (cg_thresh_);
        clusterer.setUseInterpolation (true);
        clusterer.setUseDistanceWeight (false);

        clusterer.setInputCloud (model_keypoints);
        clusterer.setInputRf (model_rf);
        clusterer.setSceneCloud (scene_keypoints);
        clusterer.setSceneRf (scene_rf);
        clusterer.setModelSceneCorrespondences (model_scene_corrs);

        //clusterer.cluster (clustered_corrs);
        clusterer.recognize (rototranslations, correspondences);

        rotation.resize(rototranslations.size ());
        translation.resize(rototranslations.size ());
        for (size_t i = 0; i < rototranslations.size (); ++i)
        {
            // Print the rotation matrix and translation vector
            rotation[i] = rototranslations[i].block<3,3>(0, 0);
            translation[i] = rototranslations[i].block<3,1>(0, 3);
        }

    } else {
        ROS_INFO("Please set model and scene first");
        return false;
    }
    return true;
}

bool object_recognizer::find_best(Eigen::Matrix3f rotation, Eigen::Vector3f translation) {
    if (have_scene && have_model) {

    } else {
        ROS_INFO("Please set model and scene first");
        return false;
    }
    return true;
}

void object_recognizer::visualize_correspondences(pcl::PointCloud<PointType>::Ptr in_cloud,
                                                  pcl::PointCloud<PointType>::Ptr out_cloud) {

}

void object_recognizer::visualize_keypoints(pcl::PointCloud<PointType>::Ptr in_cloud,
                                            pcl::PointCloud<PointType>::Ptr out_cloud) {

}