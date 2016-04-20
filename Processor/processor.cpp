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

#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/console/parse.h>

#include <fstream>
#include <sstream>
#include <iostream>

//************************************
// 			DBSCAN includes
#include "dbscan/dbscan.h"
#include "dbscan/utils.h"
#include "dbscan/kdtree2.hpp"
//************************************

using namespace pcl;
using namespace std;



int main(int argc, char *argv[]){

	string infile = argv[1];

	int minPts; // minimal amout of points in order to be considered a cluster
	double eps; // distance between points
	
	
	minPts = 30; // defaults
	eps = 0.5; // defaults
	
	cout << "min size: " << minPts << "  eps: " << eps << endl;

	// load point cloud
	fstream input(infile.c_str(), ios::in | ios::binary);
	if(!input.good()){
		cerr << "Could not read file: " << infile << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);

	pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);

	float ignore;
	int i;
	for (i=0; input.good() && !input.eof(); i++) {
		PointXYZ point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &ignore, sizeof(float));
		if(i%3 == 0)cloud->push_back(point);
	}
	input.close();


	  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::SACSegmentation<pcl::PointXYZ> seg;
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

	  seg.setEpsAngle( 20.0f * (M_PI/180.0f) );
	  seg.setAxis(axis);
	  seg.setOptimizeCoefficients (true);
	  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (100);
	  seg.setDistanceThreshold (0.25);
	  seg.setInputCloud (cloud);
	  seg.segment (*inliers, *coefficients);
	  // Extract the planar inliers from the input cloud
	  pcl::ExtractIndices<pcl::PointXYZ> extract;
	  extract.setInputCloud (cloud);
	  extract.setIndices (inliers);
	  extract.setNegative (false);
	  // Get the points associated with the planar surface
	  extract.filter (*cloud_plane);
	  //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
	  // Remove the planar inliers, extract the rest
	  extract.setNegative (true);
	  extract.filter (*cloud_f);
	  *cloud_filtered = *cloud_f;
	  //cout << ">> Planar Segmentation Done: " << tt.toc () << " ms\n"; 

	int num_threads = 4;
	
	omp_set_num_threads(num_threads); // Use 4 threads for clustering on the odroid

	NWUClustering::ClusteringAlgo dbs;
	dbs.set_dbscan_params(eps, minPts);

	double start = omp_get_wtime();
	//cout << "DBSCAN reading points.."<< endl;
	dbs.read_cloud(cloud_filtered);	

	cout << "Reading input data file took " << omp_get_wtime() - start << " seconds." << endl;

	// build kdtree for the points
	start = omp_get_wtime();
	dbs.build_kdtree();
	cout << "Build kdtree took " << omp_get_wtime() - start << " seconds." << endl;

	start = omp_get_wtime();
	//run_dbscan_algo(dbs);
	run_dbscan_algo_uf(dbs);
	cout << "DBSCAN (total) took " << omp_get_wtime() - start << " seconds." << endl;


	// Calculate boxes from all the clusters found
	float c_buff [2000];
	int buffer_size = dbs.writeClusters_uf(c_buff);

	string outfile;
	string txt = "txt";

	outfile = infile.substr(0, infile.size()-3);
	outfile = outfile + txt;
	ofstream myfile (outfile.c_str());
	  if (myfile.is_open())
	  {
	    
	    for(int count = 0; count < buffer_size; ++count){
	        myfile << c_buff[count] << endl;
	    }
	    myfile.close();
	  }
	  else cout << "Unable to open file";

	
	


return 0;
}