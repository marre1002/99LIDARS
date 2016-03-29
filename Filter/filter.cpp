#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>
#include "include/util.h"
#include <string>
#include <omp.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>

int 
main (int argc, char** argv)
{
 
 

  // Timer object for each individual part
  pcl::console::TicToc tt;
  // Timer object for running the pipeline procedure (splitting not included)
  pcl::console::TicToc to;
  // Timer object for running everything. 5 files after each other (splitting included)
  pcl::console::TicToc pT;

  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  pT.tic();
  for(int i=0; i < 5; i++){
    

    std::string str = std::string("../../PCDdataFiles/data0") + to_string(i) + ".pcd";

    reader.read (str, *cloud);
    std::cout << str << " PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

    tt.tic();

    //pcl::PointCloud<pcl::PointXYZ> cloud1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud6 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud7 (new pcl::PointCloud<pcl::PointXYZ>);

    double zero = 0.0000000;
    int iii;

    omp_lock_t sector0, sector1, sector2, sector3, sector4, sector5, sector6, sector7;
    omp_init_lock(&sector0);
    omp_init_lock(&sector1);
    omp_init_lock(&sector2);
    omp_init_lock(&sector3);
    omp_init_lock(&sector4);
    omp_init_lock(&sector5);
    omp_init_lock(&sector6);
    omp_init_lock(&sector7);

    #pragma omp parallel for
    for (iii = 0; iii < static_cast<int> (cloud->size ()); ++iii){ 
        if(cloud->points[iii].x > zero){
            if(cloud->points[iii].y > zero){
                if(cloud->points[iii].y > cloud->points[iii].x){
                    omp_set_lock(&sector0);
                    cloud0->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
                    omp_unset_lock(&sector0);
                }else{
                    omp_set_lock(&sector1);
                    cloud1->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
                    omp_unset_lock(&sector1);
                }
            }else{
                if((abs(cloud->points[iii].y)) > cloud->points[iii].x){
                    omp_set_lock(&sector2);
                    cloud2->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
                    omp_unset_lock(&sector2);
                }else{
                    omp_set_lock(&sector3);
                    cloud3->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
                    omp_unset_lock(&sector3);
                }
            }    
        }else{
            if(cloud->points[iii].y > zero){
                if(cloud->points[iii].y > (abs(cloud->points[iii].x))){
                        omp_set_lock(&sector4);
                        cloud4->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
                        omp_unset_lock(&sector4);
                    }else{
                        omp_set_lock(&sector5);
                        cloud5->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
                        omp_unset_lock(&sector5);
                    }
                }else{
                    if(cloud->points[iii].y > cloud->points[iii].x){
                        omp_set_lock(&sector6);
                        cloud6->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
                        omp_unset_lock(&sector6);
                    }else{
                        omp_set_lock(&sector7);
                        cloud7->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
                        omp_unset_lock(&sector7);
                    }
            }
        }
    }

    //pcl::io::savePCDFileASCII ("cloud1.pcd", cloud1);

    //std::cout << "1st sector points: " << one << endl;
    std::cout << str << " 2st sector points: " << cloud1->points.size() << endl;
    //std::cout << "3st sector points: " << three << endl;
    //std::cout << "4st sector points: " << four << endl;
    //std::cout << "5st sector points: " << five << endl;
    //std::cout << "6st sector points: " << six << endl;
    //std::cout << "7st sector points: " << seven << endl;
    //std::cout << "8st sector points: " << eight << endl;
    
    std::cout << str << " Splitting data in: " << tt.toc() << " ms." << endl;

    // Create the pass through filtering object
    // COMMENT BELOW SEGMENT TO REMOVE PASSTHROUGH FILTERING
    to.tic();
    std::cerr << str << " Starting VoxelGrid downsampling\n",tt.tic ();
    // Create the filtering object: downsample the dataset using a leaf size of 7cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud1);
    vg.setLeafSize (0.07f, 0.07f, 0.07f);
    vg.filter (*cloud1);
    std::cerr << str << " >> Voxelgrid Done: " << tt.toc () << " ms\n";
    std::cout << str << " PointCloud after filtering has: " << cloud1->points.size ()  << " data points." << std::endl;
    // Create the segmentation object for the planar model and set all the parameters
    std::cerr << str << " Starting Planar Segmentation",tt.tic ();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.2);
    seg.setInputCloud (cloud1);
    seg.segment (*inliers, *coefficients);
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud1);
    extract.setIndices (inliers);
    extract.setNegative (false);
    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << str << " PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;

    std::cerr << str << " >>Planar segmentation Done: " << tt.toc () << " ms\n";
    std::cerr << str << " >> Done: " << tt.toc () << " ms\n";

    std::cerr << str << " Building kdTree and finding all clusters (Euclidian cluster extraction)\n",tt.tic ();
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.25); // 0.02 = 2cm
    ec.setMinClusterSize (150);
    ec.setMaxClusterSize (5000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);
  
      // ----------------------------------------------------------------------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //viewer->setBackgroundColor (0, 0, 0);
    //viewer->addPointCloud<pcl::PointXYZ> (cloud_filtered, "Cloud with boxes");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud with boxes");
    //viewer->addCoordinateSystem (1.0);
    //viewer->initCameraParameters ();
    //------------------------------------------------------------------------------------------------------------
    
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
      /*
      pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
      feature_extractor.setInputCloud (cloud_cluster);
      feature_extractor.compute ();

      std::vector <float> moment_of_inertia;
      std::vector <float> eccentricity;
      pcl::PointXYZ min_point_OBB;
      pcl::PointXYZ max_point_OBB;
      pcl::PointXYZ position_OBB;
      Eigen::Matrix3f rotational_matrix_OBB;
      float major_value, middle_value, minor_value;
      Eigen::Vector3f major_vector, middle_vector, minor_vector;
      Eigen::Vector3f mass_center;

      feature_extractor.getMomentOfInertia (moment_of_inertia);
      feature_extractor.getEccentricity (eccentricity);
      feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
      feature_extractor.getEigenValues (major_value, middle_value, minor_value);
      feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
      feature_extractor.getMassCenter (mass_center);

      Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
      Eigen::Quaternionf quat (rotational_matrix_OBB);
      viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, ("id" + j));
      //viewer->addText3D ("Yoda" +j, position_OBB, 1.0, 1.0, 1.0,1.0, "id" + j ,0);
      //viewer->addText3D ("ID:"+j, position_OBB, 1.0, 1.0, 1.0, 1.0);
      
      //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      //std::stringstream ss;
      //ss << "cloud_cluster_" << j << ".pcd";
      //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);*/ 
      j++;
    }

    std::cout << str << " found: " << j << " clusters." << endl;
    std::cerr << str << " >>Clustering Done: " << tt.toc () << " ms\n";
    std::cout << str << " Pipeline execution ms: " << to.toc() << "\n";
  }

  std::cout << "Complete runtime ms: " << pT.toc() << "\n";

  /*while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }*/
  
  return (0);
}