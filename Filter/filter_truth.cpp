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
#include <algorithm>

//************************************
// 			DBSCAN includes
#include "dbscan/dbscan.h"
#include "dbscan/utils.h"
#include "dbscan/kdtree2.hpp"
//************************************

#define OFFSET 0.02

struct object{
		pcl::PointXYZ minPt;
		pcl::PointXYZ maxPt;
		bool remove;
		bool merged;
};

bool DoObjectsIntersect(object a, object b) {
  	if(a.minPt.x > (OFFSET+b.maxPt.x)) return false;
  	if((a.maxPt.x+OFFSET) < b.minPt.x) return false;
  	if(a.minPt.y > (OFFSET+b.maxPt.y)) return false;
  	if((a.maxPt.y+OFFSET) < b.minPt.y) return false;
  	return true;
}

object MergeObjects(object a, object b){
	object c;
	float LeftCornerX = std::min(a.minPt.x, b.minPt.x);
	float LeftCornerY = std::min(a.minPt.y, b.minPt.y);
	float LeftCornerZ = std::min(a.minPt.z, b.minPt.z);

	float RightCornerX = std::max(a.maxPt.x, b.maxPt.x);
	float RightCornerY = std::max(a.maxPt.y, b.maxPt.y);
	float RightCornerZ = std::max(a.maxPt.z, b.maxPt.z);


	c.minPt = pcl::PointXYZ(LeftCornerX, LeftCornerY, LeftCornerZ);
	c.maxPt = pcl::PointXYZ(RightCornerX, RightCornerY, RightCornerZ);
	c.remove = false;
	c.merged = true;


	return c;
}

int main (int argc, char** argv)
{

  bool visualization = false;
  bool merge = false;
  bool lines = false;
  bool dbscan = false;
  bool read_binary = true;
  int nth_point = 5; // five is default
  double eps = 0.5; // epsilon for clustering default 0.6 for the
  int minCl = 50;

  std::vector<object> objects;

  std::string infile = "../../Dataframes/";
  std::string file = "default";

  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------

  for (int i = 1; i < argc; i++) { /* We will iterate over argv[] to get the parameters stored inside.
                                          * Note that we're starting on 1 because we don't need to know the 
                                          * path of the program, which is stored in argv[0] */
            if (i != argc){ // Check that we haven't finished parsing already
                if(std::strcmp(argv[i], "-v") == 0) {
             		visualization = true;
                } else if(std::strcmp(argv[i], "-l") == 0) {
                    lines = true;
                } else if(std::strcmp(argv[i], "-d") == 0) {
                    dbscan = true;
                } else if(std::strcmp(argv[i], "-n") == 0){
 					sscanf(argv[i+1], "%i", &nth_point);
				} else if(std::strcmp(argv[i], "-e") == 0){
					eps = atof(argv[i+1]);
				} else if(std::strcmp(argv[i], "-m") == 0){
					minCl = atoi(argv[i+1]);
				} else if(std::strcmp(argv[i], "-i") == 0){
					file.assign(argv[i+1]);
				} else if(std::strcmp(argv[i], "-t") == 0){
					read_binary = false;
				} else if(std::strcmp(argv[i], "-x") == 0){
					merge = true;
				}                                    
                           
            }
            //std::cout << argv[i] << " ";
        }

  //cout << endl << "Arg, n: " << nth_point << " eps: " << eps << " minCl: " << minCl << endl;

  pcl::console::TicToc tt;

  infile.append(file);
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_main (new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<pcl::PointXYZ> points;


	if(read_binary){ // Binary
		// load point cloud
		fstream input(infile.c_str(), ios::in | ios::binary);
		if(!input.good()){
			cerr << "Could not read file: " << infile << endl;
			exit(EXIT_FAILURE);
		}

		input.seekg(0, ios::beg);

		int i;
		for (i=0; input.good() && !input.eof(); i++) {
			pcl::PointXYZ point;
			input.read((char *) &point.x, 3*sizeof(float));
			if(i%nth_point == 0)cloud->points.push_back(point);
		}
		input.close();

		float percent = ((float)(i/nth_point))/i;

		//cout << "File have " << i << " points, " << "after filtering: " << (i/nth_point) << "  (" << percent << ") "<< endl;
	}else{
		//  READ TXT INSTEAD OF BIN =======================================================================================
		FILE* f = fopen(infile.c_str(), "r");
		if (NULL == f) {
		   cerr << "Could not read file: " << infile << endl;
		   return 0;
		}
		
		int i = 0;
		float intensity;
		pcl::PointXYZ p;
		while(fscanf(f,"%f %f %f %f\n", &p.x, &p.y, &p.z, &intensity) == 4) {
			if(i%nth_point == 0) cloud->points.push_back(p);
			i++;
		}
		//cout << "file have " << i << " points" << endl;
		fclose(f);
	}          


  	  tt.tic(); // Start clock 

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
	  seg.setDistanceThreshold (0.25); // 0.3
	  seg.setInputCloud (cloud); 					//change input cloud later 
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

	  // END OF RANSAC
	  
	  int db_numberOfClusters = 0;
	  if(!dbscan){
		  // Creating the KdTree object for the search method of the extraction
		  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		  tree->setInputCloud (cloud_filtered);
		  std::vector<pcl::PointIndices> cluster_indices;
		  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		  ec.setClusterTolerance (eps); // 0.02 = 2cm
		  ec.setMinClusterSize (minCl);
		  //ec.setMaxClusterSize (); // with voxel it should be aroud 5000
		  ec.setSearchMethod (tree);
		  ec.setInputCloud (cloud_filtered);
		  ec.extract (cluster_indices);
		  

		  int j = 0;
		  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		  {
		    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		      cloud_cluster->width = cloud_cluster->points.size ();
		      cloud_cluster->height = 1;
		      cloud_cluster->is_dense = true;
		    
		    
		    object obj;
		    obj.remove = false;
	 		pcl::getMinMax3D (*cloud_cluster, obj.minPt, obj.maxPt);
	 		objects.push_back(obj);

	 	
	 		j++;
		    cloud_cluster->points.clear();
		  }

		  
		}else{ // DBSCAN CODE

			int num_threads = 4;
			//int minPts = 30; // minimal amout of points in order to be considered a cluster
			eps = 0.5; // Dbscan needs another default value
			minCl = 20;

			omp_set_num_threads(num_threads); // Use 4 threads for clustering on the odroid

			NWUClustering::ClusteringAlgo dbs;
			dbs.set_dbscan_params(eps, minCl);

			double start = omp_get_wtime();
			//cout << "DBSCAN reading points.."<< endl;
			dbs.read_cloud(cloud_filtered);	

			//cout << "Reading input data file took " << omp_get_wtime() - start << " seconds." << endl;

			// build kdtree for the points
			start = omp_get_wtime();
			dbs.build_kdtree();
			//cout << "Build kdtree took " float c_buff [200];<< omp_get_wtime() - start << " seconds." << endl;

			start = omp_get_wtime();
			//run_dbscan_algo(dbs);
			run_dbscan_algo_uf(dbs);
			//cout << "DBSCAN (total) took " << omp_get_wtime() - start << " seconds." << endl;


			// Calculate boxes from all the clusters found
			std::vector<pcl::PointXYZ> cluster_vector;
			
			dbs.writeClusters_uf(&cluster_vector);
			for (int i = 0; i < cluster_vector.size(); ++i)
			{
				if(i%2 == 0)
				{
					object obj;
					obj.minPt = cluster_vector.at(i);
					obj.maxPt = cluster_vector.at(i+1);
					obj.remove = false;
					objects.push_back(obj);
					db_numberOfClusters++;
				}
			}
			cluster_vector.clear();
		  	//cout << db_numberOfClusters << "\t";
		}

		std::vector<object> objv;

  

	if(merge){
	  	for (int i = 0; i < objects.size(); ++i)
	  	{	
			object a = objects.at(i);
			
			for (int ii = i; ii < objects.size(); ++ii)
			{
				
				object b = objects.at(ii);

				if(DoObjectsIntersect(a,b) && i != ii){

					object merge = MergeObjects(a,b);
					objv.push_back(a);
					objv.push_back(b);
					objects.at(ii) = merge;
					objects.at(i).remove = true;
				}
	  		}
	  	}
  	}

  	int count = 0;
  	for (int i = 0; i < objects.size(); ++i)
	{
	  		object obj = objects.at(i);
	  			if(!obj.remove) count++;
	 }
	
	cout << tt.toc() << "\t"; // time taken
	cout << count << endl;
	

  return (0);
}