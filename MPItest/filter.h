
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

	pcl::PointCloud<pcl::PointXYZ> cloud;
	//std::vector<std::vector<float> > floats ( 8, std::vector<float> ( 8, 0 ) );
	std::vector<std::vector<float> > floats;

	public:
		//Filters():m_pts(NULL),m_kdtree(NULL){ }
		Filters();

		int     read_file(std::string str, int nth_point);
		int     filter_and_slice();
		
};

#endif
