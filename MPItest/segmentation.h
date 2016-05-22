
#ifndef _SEGMENTATION_
#define _SEGMENTATION_

#include <pcl/point_types.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//************************************
// 			DBSCAN includes
#include "dbscan/dbscan.h"
#include "dbscan/utils.h"
#include "dbscan/kdtree2.hpp"
//************************************


using namespace std;
using namespace pcl;


class Segmentation
{
	

public:
	Segmentation();

	int   build_cloud(float *f, int size);
	int   build_cloud_two(float* fs, int size);
	int   ransac(double threshold, int iterations);
	int   euclidian(float *f, double eps, int minCl);
	int   dbscan(float *floats, double eps, int minCl, int threads);

public:
	//Class point cloud
	pcl::PointCloud<pcl::PointXYZ> cloud;	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
	
};

#endif
