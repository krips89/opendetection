//
// Created by sarkar on 10.08.15.
//

#ifndef OPENDETECTION_ODCADDETECTOR3DGLOBAL_HPP
#define OPENDETECTION_ODCADDETECTOR3DGLOBAL_HPP


namespace od
{
  namespace g3d
  {

    template<typename PointT>
    void ODCADDetector3DGlobal<PointT>::init()
    {

      boost::shared_ptr<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > mesh_source(new pcl::rec_3d_framework::MeshSource<pcl::PointXYZ>);
      mesh_source->setPath(this->training_input_location_);
      std::string training_dir_specific =  this->getSpecificTrainingDataLocation();
      mesh_source->setModelScale (1.f);
      mesh_source->generate(training_dir_specific);

      boost::shared_ptr<pcl::rec_3d_framework::Source<pcl::PointXYZ> > cast_source;
      cast_source = boost::static_pointer_cast<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> >(mesh_source);

      boost::shared_ptr<pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal> > normal_estimator;
      normal_estimator.reset(new pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal>);
      normal_estimator->setCMR(true);
      normal_estimator->setDoVoxelGrid(true);
      normal_estimator->setRemoveOutliers(true);
      normal_estimator->setFactorsForCMR(3, 7);

      if(desc_name.compare("vfh") == 0)
      {
        boost::shared_ptr<pcl::rec_3d_framework::VFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> > vfh_estimator;
        vfh_estimator.reset(new pcl::rec_3d_framework::VFHEstimation<pcl::PointXYZ, pcl::VFHSignature308>);
        vfh_estimator->setNormalEstimator(normal_estimator);

        boost::shared_ptr<pcl::rec_3d_framework::GlobalEstimator<pcl::PointXYZ, pcl::VFHSignature308> > cast_estimator;
        cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::VFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> >(
            vfh_estimator);

        boost::shared_ptr<pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::VFHSignature308> > global(
            new pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::VFHSignature308>());
        global->setDataSource(cast_source);
        global->setTrainingDir(training_dir_specific);
        global->setDescriptorName(desc_name);
        global->setNN(NN);
        global->setFeatureEstimator(cast_estimator);
        global->initialize(false);
        this->global_ = global;

      } else if(desc_name.compare("cvfh") == 0)
      {
        boost::shared_ptr<pcl::rec_3d_framework::CVFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> > vfh_estimator;
        vfh_estimator.reset(new pcl::rec_3d_framework::CVFHEstimation<pcl::PointXYZ, pcl::VFHSignature308>);
        vfh_estimator->setNormalEstimator(normal_estimator);

        boost::shared_ptr<pcl::rec_3d_framework::GlobalEstimator<pcl::PointXYZ, pcl::VFHSignature308> > cast_estimator;
        cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::CVFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> >(
            vfh_estimator);

        boost::shared_ptr<pcl::rec_3d_framework::GlobalNNPipeline<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::VFHSignature308> > global(
            new pcl::rec_3d_framework::GlobalNNPipeline<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::VFHSignature308>());
        global->setDataSource(cast_source);
        global->setTrainingDir(training_dir_specific);
        global->setDescriptorName(desc_name);
        global->setFeatureEstimator(cast_estimator);
        global->setNN(NN);
        global->initialize(false);
        this->global_ = global;
      } else if(desc_name.compare("esf") == 0)
      {
        boost::shared_ptr<pcl::rec_3d_framework::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> > estimator;
        estimator.reset(new pcl::rec_3d_framework::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640>);

        boost::shared_ptr<pcl::rec_3d_framework::GlobalEstimator<pcl::PointXYZ, pcl::ESFSignature640> > cast_estimator;
        cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> >(
            estimator);

        boost::shared_ptr<pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::ESFSignature640> > global(
            new pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::ESFSignature640>());
        global->setDataSource(cast_source);
        global->setTrainingDir(training_dir_specific);
        global->setDescriptorName(desc_name);
        global->setFeatureEstimator(cast_estimator);
        global->setNN(NN);
        global->initialize(false);
        this->global_ = global;
      } else
      {
        std::cout << "FATAL: descriptor type not available.";
      }

    }

    template<typename PointT>
    ODDetections3D* ODCADDetector3DGlobal<PointT>::detectOmni(ODScenePointCloud<PointT> *scene)
    {
      ODDetections3D *detections = new ODDetections3D;


      typename pcl::PointCloud<PointT>::Ptr frame;
      float Z_DIST_ = 1.25f;
      float text_scale = 0.015f;

      frame = scene->getPointCloud();
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*frame, *xyz_points);


      //Step 1 -> Segment
      pcl::apps::DominantPlaneSegmentation<pcl::PointXYZ> dps;
      dps.setInputCloud(xyz_points);
      dps.setMaxZBounds(Z_DIST_);
      dps.setObjectMinHeight(0.005);
      dps.setMinClusterSize(1000);
      dps.setWSize(9);
      dps.setDistanceBetweenClusters(0.1f);

      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
      std::vector<pcl::PointIndices> indices;
      dps.setDownsamplingSize(0.02f);
      dps.compute_fast(clusters);
      dps.getIndicesClusters(indices);
      Eigen::Vector4f table_plane_;
      Eigen::Vector3f normal_plane_ = Eigen::Vector3f(table_plane_[0], table_plane_[1], table_plane_[2]);
      dps.getTableCoefficients(table_plane_);


      float dist_ = 0.03f;

      for(size_t i = 0; i < clusters.size(); i++)
      {

        global_->setInputCloud(xyz_points);
        global_->setIndices(indices[i].indices);
        global_->classify();

        std::vector<std::string> categories;
        std::vector<float> conf;
        global_->getCategory(categories);
        global_->getConfidence(conf);
        //detection done!

        std::string category = categories[0];
        Eigen::Vector4d centroid;
        pcl::compute3DCentroid(*xyz_points, indices[i].indices, centroid);
        //position at 3D identified!

        //now fill up the detection:
        ODDetection3D *detection = new ODDetection3D;
        detection->setType(ODDetection::OD_DETECTION_CLASS);
        detection->setId(categories[0]);
        detection->setLocation(centroid);
        detection->setMetainfoCluster(clusters[i]);
        detections->push_back(detection);
      }

      return detections;
    }

    template<typename PointT>
    ODDetections* ODCADDetector3DGlobal<PointT>::detect(ODScenePointCloud<PointT> *scene)
    {
      ODDetections *detections = new ODDetections;


      typename pcl::PointCloud<PointT>::Ptr frame;
      float Z_DIST_ = 1.25f;
      float text_scale = 0.015f;

      frame = scene->getPointCloud();
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*frame, *xyz_points);

      global_->setInputCloud(xyz_points);
      //no indices set, so it would try to classify the entire PC
      global_->classify();

      std::vector<std::string> categories;
      std::vector<float> conf;
      global_->getCategory(categories);
      global_->getConfidence(conf);
      //detection done!

      std::string category = categories[0];

      //now fill up the detection:
      ODDetection *detection = new ODDetection3D(ODDetection::OD_DETECTION_CLASS, categories[0], conf[0]);
      detections->push_back(detection);

      return detections;
    }
  }
}


#endif //OPENDETECTION_ODCADDETECTOR3DGLOBAL_HPP
