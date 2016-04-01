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
#include <string>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/console/parse.h>


int main (int argc, char** argv)
{

  bool visualization = false;
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if(pcl::console::find_argument (argc, argv, "-v") >= 0)
  {
  	// Run with -v to start the visualizer. Default is without
  	visualization = true;
  }
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_main (new pcl::PointCloud<pcl::PointXYZ>);

  reader.read ("../../PCDdataFiles/data02.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Timer object
  pcl::console::TicToc tt;

  tt.tic();


  pcl::PointCloud<pcl::PointXYZ> cloud_yo;


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud6 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud7 (new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > v;
  v.push_back(cloud0);
  v.push_back(cloud1);
  v.push_back(cloud2);
  v.push_back(cloud3);
  v.push_back(cloud4);
  v.push_back(cloud5);
  v.push_back(cloud6);
  v.push_back(cloud7);

  double zero = 0.0000000;
  for (int iii = 0; iii < static_cast<int> (cloud->size ()); ++iii){ 
      if((iii%2) == 0)
      {

      }else{
	      if(cloud->points[iii].x > zero){
	          if(cloud->points[iii].y > zero){
	              if(cloud->points[iii].y > cloud->points[iii].x){
	                  cloud0->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
	              }else{
	                  cloud1->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
	              }
	          }else{
	              if((abs(cloud->points[iii].y)) > cloud->points[iii].x){
	                  cloud2->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
	              }else{
	                  cloud3->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
	              }
	          }    
	      }else{
	          if(cloud->points[iii].y > zero){
	              if(cloud->points[iii].y > (abs(cloud->points[iii].x))){
	                      cloud4->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
	                  }else{
	                      cloud5->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
	                  }
	              }else{
	                  if(cloud->points[iii].y > cloud->points[iii].x){
	                      cloud6->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
	                  }else{
	                      cloud7->points.push_back (pcl::PointXYZ (cloud->points[iii].x,cloud->points[iii].y,cloud->points[iii].z));
	                  }
	          }
	      }
  		}
  }

  //pcl::io::savePCDFileASCII ("cloud1.pcd", cloud1);

  int total_points = 0;
  for(int k = 0 ; k < v.size(); k++){
  	total_points += v.at(k)->points.size();
  	std::cout << "Sector " << k <<  " have: " << v.at(k)->points.size() << " points" << endl;	
  }
  std::cout << "All sectors:" << total_points << " points" << endl;

  
  std::cout << "Splitting data in: " << tt.toc() << " ms." << endl;

  // Create the pass through filtering object
  // COMMENT BELOW SEGMENT TO REMOVE PASSTHROUGH FILTERING

  int clusters = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);

  for(int ii = 0 ; ii < v.size(); ii++){
  
	  //std::cerr << "Starting VoxelGrid downsampling\n",tt.tic ();
	  // Create the filtering object: downsample the dataset using a leaf size of 7cm
	  //pcl::VoxelGrid<pcl::PointXYZ> vg;
	  //vg.setInputCloud (v.at(ii));
	  //vg.setLeafSize (0.07f, 0.07f, 0.07f);
	  //vg.filter (*cloud_voxel);
	  //std::cerr << ">> Done: " << tt.toc () << " ms\n";
	  //std::cout << "PointCloud after filtering has: " << cloud0->points.size ()  << " data points." << std::endl; 
	  // Create the segmentation object for the planar model and set all the parameters
	  std::cerr << "Starting Planar Segmentation",tt.tic ();

	  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);

	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::SACSegmentation<pcl::PointXYZ> seg;
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	  pcl::PCDWriter writer;
	  seg.setEpsAngle(  30.0f * (M_PI/180.0f) );
	  seg.setAxis(axis);
	  seg.setOptimizeCoefficients (true);
	  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (100);
	  seg.setDistanceThreshold (0.30);
	  seg.setInputCloud (v.at(ii));
	  seg.segment (*inliers, *coefficients);
	  // Extract the planar inliers from the input cloud
	  pcl::ExtractIndices<pcl::PointXYZ> extract;
	  extract.setInputCloud (v.at(ii));
	  extract.setIndices (inliers);
	  extract.setNegative (false);
	  // Get the points associated with the planar surface
	  extract.filter (*cloud_plane);
	  //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
	  // Remove the planar inliers, extract the rest
	  extract.setNegative (true);
	  extract.filter (*cloud_f);
	  *cloud_filtered = *cloud_f;
	  std::cerr << ">> Done: " << tt.toc () << " ms\n";

	 
	  
	  tt.tic ();
	  // Creating the KdTree object for the search method of the extraction
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud (cloud_filtered);
	  std::vector<pcl::PointIndices> cluster_indices;
	  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	  ec.setClusterTolerance (0.40); // 0.02 = 2cm
	  ec.setMinClusterSize (50);
	  ec.setMaxClusterSize (6000); // with voxel it should be aroud 5000
	  ec.setSearchMethod (tree);
	  ec.setInputCloud (cloud_filtered);
	  ec.extract (cluster_indices);
	  
	  int j = 0;
	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
	    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	    clusters++;
	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	      cloud_main->points.push_back (cloud_filtered->points[*pit]); //*
	      cloud_main->width = cloud_main->points.size ();
	      cloud_main->height = 1;
	      cloud_main->is_dense = true;
	    
	    /*pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
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
	    viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, ("id" + j + ii));
	    */ //viewer->addText3D ("Yoda" +j, position_OBB, 1.0, 1.0, 1.0,1.0, "id" + j ,0);
	    //viewer->addText3D ("ID:"+j, position_OBB, 1.0, 1.0, 1.0, 1.0);
	    
	    //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
	    //std::stringstream ss;
	    //ss << "cloud_cluster_" << j << ".pcd";
	    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);*/ 
	    j++;
	  }

	  std::cout << "found: " << j << " clusters." << endl;
	  std::cerr << ">> Done: " << tt.toc () << " ms\n";

	  cloud_filtered->points.clear();
	  cloud_f->points.clear();
	  cloud_voxel->points.clear();
  }

  if(visualization){
	  // ----------------------------------------------------------------------------------------------------------
	  // -----Open 3D viewer and add point cloud-----
	  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> grey(cloud,204, 204, 179);
	  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(cloud, 255, 0, 0);

	  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->setBackgroundColor (0, 0, 0);
	  viewer->addPointCloud<pcl::PointXYZ> (cloud, "source");
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.2f, 0.2f, 0.2f, "source");
	  viewer->addPointCloud<pcl::PointXYZ> (cloud_main, "main");
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "main");  
	  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud with boxes");
	  viewer->addCoordinateSystem (1.0);
	  viewer->initCameraParameters ();
	  //------------------------------------------------------------------------------------------------------------

	  int z = -1.5;
	  viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(35,0,z), "line0");
	  viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(0,30,z), "line1");
	  viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(-35,0,z), "line2");
	  viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(0,-30,z), "line3");
	  viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(20,20,z), "line4");
	  viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(-20,20,z), "line5");
	  viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(20,-20,z), "line6");
	  viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(-20,-20,z), "line7");

	   std::cout << "Found a total of: " << clusters << " clusters." << endl;

	  while (!viewer->wasStopped ())
	  {
	    viewer->spinOnce (100);
	    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  }
  	}else{
  		std::cout << "Found a total of: " << clusters << " clusters." << endl;
  	}

  return (0);
}