
#ifndef _FILTER_
#define _FILTER_

#include <pcl/point_types.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/common_headers.h>
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;


class Filters
{

public:
	//Filters():m_pts(NULL),m_kdtree(NULL){ }
	Filters();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	int     read_file(std::string str, int nth_point);
	int     filter_and_slice(std::vector<std::vector<float> > *floats);
	
};

#endif