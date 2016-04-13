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
#include <mpi.h>

using namespace pcl;
using namespace std;

static int numprocs;

int main(int argc, char **argv) {
int my_rank = 0;
// MPI initializations
MPI_Status status;
MPI_Init (&argc, &argv);
MPI_Comm_size (MPI_COMM_WORLD, &numprocs);
MPI_Comm_rank (MPI_COMM_WORLD, &my_rank);

//************************************************************************************************************


if(my_rank == 0){ // I'm master and handle the splitting

  // Timer object
  pcl::console::TicToc tt;

    tt.tic();

	std::string infile = "../../PCDdataFiles/002.bin";

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

	cout << "Read KTTI point cloud with " << (i/3) << " points in " << tt.toc() << " ms." << endl;

	 pcl::PCDWriter writer;

    // Save DoN features
     writer.write<PointXYZ> ("slice.pcd", *cloud, false);


  tt.tic();

  int m_tag = 0; // MPI message tag
  float aa [20000];
  float bb [20000];
  float cc [20000];
  float dd [20000];
  float ee [20000];
  float ff [20000];
  float gg [20000];
  float hh [20000];
  int count0,count1,count2,count3,count4,count5,count6,count7 = 0;
  
  double zero = 0.0000000;
  for (int iii = 0; iii < static_cast<int> (cloud->size()); ++iii){ 
    	if(cloud->points[iii].x > zero){
	          if(cloud->points[iii].y > zero){
	              if(cloud->points[iii].y > cloud->points[iii].x){
	              	  aa[count0++] = cloud->points[iii].x;
	              	  aa[count0++] = cloud->points[iii].y;
	              	  aa[count0++] = cloud->points[iii].z;

	              }else{
	                  bb[++count1] = cloud->points[iii].x;
	              	  bb[++count1] = cloud->points[iii].y;
	              	  bb[++count1] = cloud->points[iii].z;
	              }
	          }else{
	              if((abs(cloud->points[iii].y)) > cloud->points[iii].x){
	                  cc[++count2] = cloud->points[iii].x;
	              	  cc[++count2] = cloud->points[iii].y;
	              	  cc[++count2] = cloud->points[iii].z;
	              }else{
	                  dd[++count3] = cloud->points[iii].x;
	              	  dd[++count3] = cloud->points[iii].y;
	              	  dd[++count3] = cloud->points[iii].z;
	              }
	          }    
	      }else{
	          if(cloud->points[iii].y > zero){
	              if(cloud->points[iii].y > (abs(cloud->points[iii].x))){
	              	  ee[++count4] = cloud->points[iii].x;
	              	  ee[++count4] = cloud->points[iii].y;
	              	  ee[++count4] = cloud->points[iii].z;
	               }else{
	                  ff[++count5] = cloud->points[iii].x;
	              	  ff[++count5] = cloud->points[iii].y;
	              	  ff[++count5] = cloud->points[iii].z;
	               }
	           }else{
	               if(cloud->points[iii].y > cloud->points[iii].x){
	                  gg[++count6] = cloud->points[iii].x;
	              	  gg[++count6] = cloud->points[iii].y;
	              	  gg[++count6] = cloud->points[iii].z;
	                }else{
	                  hh[++count7] = cloud->points[iii].x;
	              	  hh[++count7] = cloud->points[iii].y;
	              	  hh[++count7] = cloud->points[iii].z;
	                }
	          }
	      }
   
  }

   count0++; // This is needed because of the 0-index array

   cout << "Splitting data in: " << tt.toc() << " ms" << endl;
   tt.tic();
   // Send the number of floats to send
   MPI_Send(&count0, 1, MPI_INT, 1, m_tag, MPI_COMM_WORLD);
   // Send the float buffer (first sector)
   MPI_Send(&aa, count0, MPI_FLOAT, 1, m_tag, MPI_COMM_WORLD);

   cout << "Sending data in: " << tt.toc() << " ms" << endl;

   cout << "Waiting to get data back..." << endl;

   //MPI_Recv(); // Receive data from the workers, sync problem?

   int n;
   for(n = 0; n < 9; n++){
   	cout << cloud->points[n].x << "\t" << cloud->points[n].y << "\t" <<cloud->points[n].z << endl;
   }

   
   for(n = 0; n < (3*9); n++){
   	cout << aa[n] << "\t";
   	if(n%3 == 0) 
   		cout << endl;
   }
    
    int number_amount;
    MPI_Status status;
    MPI_Probe(MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &status);
    MPI_Get_count(&status, MPI_INT, &number_amount);

    // Allocate a buffer to hold the incoming numbers
    int* number_buf = (int*)malloc(sizeof(int) * number_amount);

    // Now receive the message with the allocated buffer
    MPI_Recv(number_buf, number_amount, MPI_INT, status.MPI_SOURCE, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

   	cout << "Node 0 (master) received " << number_amount << " from " << status.MPI_SOURCE << endl;
    free(number_buf);

   /*int buf[32];
   MPI_Status status;
   // receive message from any source
   MPI_recv(buf, 32, MPI_INT, MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &status);
   int replybuf[];
   // send reply back to sender of the message received above
   MPI_send(buf, 32, MPI_INT, status.MPI_SOURCE, tag, MPI_COMM_WORLD);*/
   

}else if(my_rank == 1){ // Worker1 runs this code

	pcl::console::TicToc tt;
	tt.tic();

	int count;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

 	float buff [20000];
 
    MPI_Recv(&count, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

 	MPI_Recv(&buff, count, MPI_FLOAT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

 	pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
 	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

 	int i;
	for(i=0; i < count ; i=i+3) {
		PointXYZ point;
		point.x = buff[i];
		point.y = buff[i+1];
		point.z = buff[i+2];
		cloud->push_back(point);
	}

 	cout << "------------ worker1 -------------------" << endl;
	cout << "Count is: " << count  << " count/3 = " << (count/3) << endl;
 	cout << "Received: " << cloud->points.size() << " points." << endl;
 	cout << "Time elapsed: " << tt.toc() << "ms" << endl;

    int n;
    for(n = 0; n < 9; n++){
   		cout << cloud->points[n].x << "\t" << cloud->points[n].y << "\t" <<cloud->points[n].z << endl;
    }



  	 pcl::PCDWriter writer;

    // Save DoN features
     writer.write<PointXYZ> ("slice.pcd", *cloud, false);

 	// Start processing data-slice --> RANSAC

 	/*
	  // Create the segmentation object for the planar model and set all the parameters
	  std::cerr << "Starting Planar Segmentation\n",tt.tic ();

	  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);

	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::SACSegmentation<pcl::PointXYZ> seg;
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	  pcl::PCDWriter writer;
	  seg.setEpsAngle( 30.0f * (M_PI/180.0f) );
	  seg.setAxis(axis);
	  seg.setOptimizeCoefficients (true);
	  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (100);
	  seg.setDistanceThreshold (0.30);
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
	  cout << ">> Planar Segmentation Done: " << tt.toc () << " ms\n";
	  */


	  int root = 0;
	  int buf [30] = {1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5};

	  MPI_Send(&buf, 21, MPI_INT, root, 0, MPI_COMM_WORLD);

	  cout << "worker1 done!" << endl;
}




//*****************************************************************************************************************

// End MPI
MPI_Finalize ();
return 0;
}