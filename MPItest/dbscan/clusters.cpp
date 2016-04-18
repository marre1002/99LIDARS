/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*   Files: omp_main.cpp clusters.cpp  clusters.h utils.h utils.cpp          */
/*   			dbscan.cpp dbscan.h kdtree2.cpp kdtree2.hpp          */
/*		    						             */
/*   Description: an openmp implementation of dbscan clustering algorithm    */
/*				using the disjoint set data structure        */
/*                                                                           */
/*   Author:  Md. Mostofa Ali Patwary                                        */
/*            EECS Department, Northwestern University                       */
/*            email: mpatwary@eecs.northwestern.edu                          */
/*                                                                           */
/*   Copyright, 2012, Northwestern University                                */
/*   See COPYRIGHT notice in top-level directory.                            */
/*                                                                           */
/*   Please cite the following publication if you use this package 	     */
/* 									     */
/*   Md. Mostofa Ali Patwary, Diana Palsetia, Ankit Agrawal, Wei-keng Liao,  */
/*   Fredrik Manne, and Alok Choudhary, "A New Scalable Parallel DBSCAN      */
/*   Algorithm Using the Disjoint Set Data Structure", Proceedings of the    */
/*   International Conference on High Performance Computing, Networking,     */
/*   Storage and Analysis (Supercomputing, SC'12), pp.62:1-62:11, 2012.	     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



#include "clusters.h"


namespace NWUClustering
{
	Clusters::~Clusters()
	{
		if(m_pts)
		{
			m_pts->m_points.clear();
			delete m_pts;
			m_pts = NULL;
		}

		if(m_kdtree)
		{
			delete m_kdtree;
			m_kdtree = NULL;
		}
	}

	int Clusters::read_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{

		cout << "number of points, from cluster: " << cloud->points.size() << endl;

		int num_points = cloud->points.size();
		int dims = 3;

		m_pts->m_i_dims = dims;
        m_pts->m_i_num_points = num_points;

                               
		// allocate memory

		m_pts = new Points;
		m_pts->m_points.resize(num_points);
		
        for(int ll = 0; ll < num_points; ll++)
                m_pts->m_points[ll].resize(dims);
		
		for(int i = 0; i < num_points; i++){
			m_pts->m_points[i][0] = cloud->points[i].x;
			m_pts->m_points[i][1] = cloud->points[i].y;
			m_pts->m_points[i][2] = cloud->points[i].z;
		} 
		
		return 0;		
	}

	int Clusters::build_kdtree()
	{
		if(m_pts == NULL)
		{
			cout << "Point set is empty" << endl;
			return -1;
		}

		m_kdtree = new kdtree2(m_pts->m_points, false);
		
		if(m_kdtree == NULL)
		{
			cout << "Falied to allocate new kd tree" << endl;
			return -1;
		}
		
		return 0;		
	} 
}
