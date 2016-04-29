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
#include <pcl/common/common.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/console/parse.h>
#include <sstream>
#include <fstream>
#include <iostream>

int main (int argc, char** argv)
{

  bool visualization = false;
  bool lines = false;
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if(pcl::console::find_argument (argc, argv, "-v") >= 0)
  {
  	// Run with -v to start the visualizer. Default is without
  	visualization = true;
  }
  if(pcl::console::find_argument(argc,argv, "-l") >= 0)
  {
  	lines = true;
  }
  pcl::console::TicToc tt;

  tt.tic();
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_main (new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<pcl::PointXYZ> points;

 std::string infile = "../../BinAndTxt/0000000013.bin";

	// load point cloud
	fstream input(infile.c_str(), ios::in | ios::binary);
	if(!input.good()){
		cerr << "Could not read file: " << infile << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);

	//pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);

	float ignore;
	int i;
	for (i=0; input.good() && !input.eof(); i++) {
		pcl::PointXYZ point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &ignore, sizeof(float));
		if(i%5 == 0)cloud->points.push_back(point);
	}
	input.close();

	cout << "Read KTTI point cloud with " << (i/5) << " points in " << tt.toc() << " ms." << endl;



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

  // Devide the dataset and keep every n:th point (setting it to 1 will include all points)
  //int nth_point = 3;
  double zero = 0.0000000;
  for (int iii = 0; iii < static_cast<int> (cloud->size ()); ++iii){ 
   // if((iii%nth_point) == 0){
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
    //}else{
    	//Ignore this point
    //}
  }

  //pcl::io::savePCDFileASCII ("cloud1.pcd", cloud1);

  int total_points = 0;
  for(int k = 0 ; k < v.size(); k++){
  	total_points += v.at(k)->points.size();
  	std::cout << "Sector " << k <<  " have: " << v.at(k)->points.size() << " points" << endl;	
  }
  std::cout << "All sectors:" << total_points << " points" << endl;

  
  //std::cout << "Splitting data in: " << tt.toc() << " ms." << endl;

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

  	  tt.tic();
  	  cout << "Clustering.... ";

	  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);

	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::SACSegmentation<pcl::PointXYZ> seg;
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	  pcl::PCDWriter writer;
	  seg.setEpsAngle( 15.0f * (M_PI/180.0f) ); // Perfect value! 
	  seg.setAxis(axis);
	  seg.setOptimizeCoefficients (true);
	  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (100);
	  seg.setDistanceThreshold (0.2); // 0.3
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

	 
	  
	  
	  // Creating the KdTree object for the search method of the extraction
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud (cloud_filtered);
	  std::vector<pcl::PointIndices> cluster_indices;
	  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	  ec.setClusterTolerance (0.7); // 0.02 = 2cm
	  ec.setMinClusterSize (10);
	  ec.setMaxClusterSize (3500); // with voxel it should be aroud 5000
	  ec.setSearchMethod (tree);
	  ec.setInputCloud (cloud_filtered);
	  ec.extract (cluster_indices);
	  
	  

	  int j = 0;
	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	    clusters++;
	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
	      cloud_cluster->width = cloud_cluster->points.size ();
	      cloud_cluster->height = 1;
	      cloud_cluster->is_dense = true;
	    
	    
	    pcl::PointXYZ minPt, maxPt;
 		pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
 		points.push_back(minPt);
 		points.push_back(maxPt);
 		j++;
	    cloud_cluster->points.clear();
	  }


	  cout << "Done in " << tt.toc() << " ms.\t";
	  cout << j << " clusters." << endl;

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
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.6f, 0.6f, 0.6f, "source");
	  //viewer->addPointCloud<pcl::PointXYZ> (cloud_main, "main");
	  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "main");  
	  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud with boxes");
	  viewer->addCoordinateSystem (1.0);
	  viewer->initCameraParameters ();

	  	// Draw boxes around clusters and give then id
        std::stringstream ss;
        int counts = 0;
	   for(int h = 0 ; h < points.size(); h++)
	   {
  		    if(h%2 == 0)
  		    {
  		    	ss << "id" << h << "test";
    			std::string str = ss.str();
  		    	pcl::PointXYZ a,b, middle;
  		    	a = points.at(h);
  		    	b = points.at(h+1);
  		    	viewer->addCube(a.x, b.x, a.y, b.y, a.z, b.z, 1.0,0.0,0.0, str ,0);
  		    	ss.str("");
    			ss << counts;
    			middle = pcl::PointXYZ(((a.x+b.x)/2),((a.y+b.y)/2),((a.z+b.z)/2));
    			viewer->addText3D(ss.str(),middle, 0.5,1.0,1.0,1.0,ss.str(),0);
    			counts++;
  			}
 	   }
	  //------------------------------------------------------------------------------------------------------------

	  if(lines){
	  	int z = -1.9;
	  	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(55,0,z),0.0f,8.0f,0.0f, "aline");
	  	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(0,55,z),0.0f,8.0f,0.0f, "bline");
	  	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(-55,0,z),0.0f,8.0f,0.0f, "cline");
	  	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(0,-55,z),0.0f,8.0f,0.0f, "dline");
	  	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(55,55,z),0.0f,8.0f,0.0f, "eline");
	  	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(-55,55,z),0.0f,8.0f,0.0f, "fline");
	  	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(55,-55,z),0.0f,8.0f,0.0f, "gline");
	  	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(-55,-55,z),0.0f,8.0f,0.0f, "hline");
	  }

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