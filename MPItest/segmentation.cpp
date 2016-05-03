#include "segmentation.h"



Segmentation::Segmentation()
{
	// Empty constructor
}

int Segmentation::build_cloud(float *f, int size)
{

	for(int i=0; i < size; i++) {
		if(i%3 == 0)
		{
			pcl::PointXYZ point;
			point.x = f[i];
			point.y = f[i+1];
			point.z = f[i+2];
			cloud.push_back(point);
		}
	}

	return(0);
}
int Segmentation::ransac(double threshold, int iterations)
{

	  cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>()); 

	  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
	  pcl::SACSegmentation<pcl::PointXYZ> seg;

	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

	  seg.setEpsAngle( 15.0f * (M_PI/180.0f) );
	  seg.setAxis(axis);
	  seg.setOptimizeCoefficients (true);
	  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (iterations);
	  seg.setDistanceThreshold (threshold);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb (new pcl::PointCloud<pcl::PointXYZ> (cloud));
	  seg.setInputCloud (cloudb);
	  seg.segment (*inliers, *coefficients);
	  // Extract the planar inliers from the input cloud
	  pcl::ExtractIndices<pcl::PointXYZ> extract;
	  extract.setInputCloud (cloudb);
	  extract.setIndices (inliers);
	  extract.setNegative (false);
	  // Get the points associated with the planar surface
	  extract.filter (*cloud_plane);
	  // Remove the planar inliers, extract the rest
	  extract.setNegative (true);
	  extract.filter (*cloud_f);
	  *cloud_filtered = *cloud_f;

	
	return 0;		
}

/*
*	Returns the size of the float array containing the clusters
*/
int Segmentation::euclidian(float *f, double eps, int minCl)
{

	 //Creating the KdTree object for the search method of the extraction
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud (cloud_filtered);
	  std::vector<pcl::PointIndices> cluster_indices;
	  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	  ec.setClusterTolerance (eps); // 0.02 = 2cm
	  ec.setMinClusterSize (minCl);
	  //ec.setMaxClusterSize (3500); // with voxel it should be aroud 5000
	  ec.setSearchMethod (tree);
	  ec.setInputCloud (cloud_filtered);
	  ec.extract (cluster_indices);
			  
	 
	  int nn = 0;
	  int j = 0;

	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
	      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
	      cloud_cluster->width = cloud_cluster->points.size ();
	      cloud_cluster->height = 1;
	      cloud_cluster->is_dense = true;
	  }
	    
	    
	    pcl::PointXYZ minPt, maxPt;
 		pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);

 		f[nn++] = minPt.x;
 		f[nn++] = minPt.y;
 		f[nn++] = minPt.z;
 		f[nn++] = maxPt.x;
 		f[nn++] = maxPt.y;
 		f[nn++] = maxPt.z;

	    j++;
	    cloud_cluster->points.clear();
	  }
	
	return nn; 	// Size of flaot buffer		
}

int Segmentation::dbscan(std::vector<std::vector<float> > *floats)
{

	int num_threads = 4;
	int minPts = 30; // minimal amout of points in order to be considered a cluster
	double eps = 0.5; // distance between points


	omp_set_num_threads(num_threads); // Use 4 threads for clustering on the odroid

	NWUClustering::ClusteringAlgo dbs;
	dbs.set_dbscan_params(eps, minPts);

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
	float c_buff [200];
	int buffer_size = dbs.writeClusters_uf(c_buff);
	
	return 0;		
}
