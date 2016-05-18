#include <pcl/console/time.h>
#include <string>
#include <vector>
#include <sstream>
#include <mpi.h>
#include <algorithm>    // std::sort

#include "filter.h"
#include "segmentation.h"

#define OFFSET 0.08

using namespace pcl;
using namespace std;

static int numprocs;

bool less_vectors(const vector<float>& a,const vector<float>& b) {
   return a.size() > b.size();
}

struct object{
		pcl::PointXYZ minPt;
		pcl::PointXYZ maxPt;
		bool remove;
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

	return c;
}

/*************************************************************************************
*		Main
**************************************************************************************/
int main(int argc, char **argv) {
	int my_rank = 0;

	bool dbscan = false;
	int nth_point = 5; // five is default
	double eps = 0.6; // epsilon for clustering default 0.6 for the
	int minCl = 30;
	
	std::string infile = "../../BinAndTxt/";
	std::string file = "0000000021.bin";

	for (int i = 1; i < argc; i++) { 
        if (i + 1 != argc) // Check that we haven't finished parsing already
            if(std::strcmp(argv[i], "-d") == 0) {
                dbscan = true;
            } else if(std::strcmp(argv[i], "-n") == 0){
					//nth_point = atoi(argv[i+1]);
					sscanf(argv[i+1], "%i", &nth_point);
			} else if(std::strcmp(argv[i], "-e") == 0){
				eps = atof(argv[i+1]);
			} else if(std::strcmp(argv[i], "-m") == 0){
				minCl = atoi(argv[i+1]);  
			}else if(std::strcmp(argv[i], "-i") == 0){
				file.assign(argv[i+1]);                                    
        }
        //std::cout << argv[i] << " ";
	}

	infile.append(file);
	//cout << endl << "Arg, n: " << nth_point << " eps: " << eps << " minCl: " << minCl << endl;

	// MPI initializations



	MPI_Status status;
	MPI_Init (&argc, &argv);
	MPI_Comm_size (MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank (MPI_COMM_WORLD, &my_rank);

	
/*******************************************************************************************
*		Master runs this code
********************************************************************************************/
	if(my_rank == 0){ 

	 int read_file;
	 //Dedicated object vector for box collection.
	 std::vector<object> objects; 
	 //Define float buffer here.

	 pcl::console::TicToc tt;
	 tt.tic();	  
	 //std::vector<std::vector<float> > floats;
	 int m_tag = 0; // MPI message tag

	 std::cout << "Its a me, master." << endl;

	  

	 // Read file and create 8 point clouds
	 // Nth_point will be kept from the data e.g. 3, every third point will be used
	 Filters filt;
	 filt.read_file(infile, nth_point);
	 filt.filter_and_slice();

	 read_file = tt.toc();

	 std::sort(filt.floats.begin(),filt.floats.end(),less_vectors);

	int sectors = 8;
	
	 tt.tic(); // Distributing, processing, and cathering
	for(int i = 0; i < sectors ; i++){ 
	   int bsize = filt.floats.at(i).size();
	   MPI_Send(&bsize, 1, MPI_INT, (i+1), m_tag, MPI_COMM_WORLD);
	   float *f = &filt.floats.at(i)[0];
	   MPI_Send(f, bsize, MPI_FLOAT, (i+1), m_tag, MPI_COMM_WORLD);
	}
	 int sending = tt.toc(); 
	
	int clusterCount = 0;
	for(int i = 0; i < sectors ; i++){ 
		int number_amount;
		MPI_Status status;
		MPI_Probe(MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &status);
		MPI_Get_count(&status, MPI_FLOAT, &number_amount);

		// Allocate a buffer to hold the incoming numbers
		float* number_buf = (float*)malloc(sizeof(float) * number_amount);
		// Now receive the message with the allocated buffer
		MPI_Recv(number_buf, number_amount, MPI_FLOAT, status.MPI_SOURCE, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

		// Loop through all received numbers from this sector and create object representations
		for(int j = 0; j < number_amount; j+6){
			object c;
			c.minPt = pcl::PointXYZ(number_buf[j], number_buf[j+1], number_buf[j+2]);
			c.maxPt = pcl::PointXYZ(number_buf[j+3], number_buf[j+4], number_buf[j+5]);

			// Add the stinking object
			objects.push_back(c);

		}



		clusterCount = clusterCount + (number_amount/3);
		free(number_buf);	
  	}

  	//Initiate merging
  	for (int i = 0; i < objects.size(); ++i)
  	{	
		object a = objects.at(i);
		
		for (int ii = i; ii < objects.size(); ++ii)
		{
			
			object b = objects.at(ii);

			if(DoObjectsIntersect(a,b) && i != ii){

				object merge = MergeObjects(a,b);
				objects.at(ii) = merge;
				objects.at(i).remove = true;
			}
  		}
  	}

  	int mergedobj = 0;
	  	for (int i = 0; i < objects.size(); ++i)
  			if(!objects.at(i).remove) mergedobj++;

	int processing = tt.toc();
	cout << "Read: " << filt.cloud.size() << " from " << infile << " (nth: " << nth_point <<")" << endl; 
	cout << "Number of clusters found:\t"  << clusterCount << endl; 
	cout << "Read file and filter:\t\t" << read_file << " ms" << endl;
	cout << "Sending data-loop:\t\t" << sending << " ms" << endl;
	cout << "Distri, process, gather:\t\t" << processing << " ms" << endl;
	cout << "Merging, found :\t\t" << mergedobj << " objects" << endl;
	cout << "=========== Total: \t" << (read_file + processing) << " ms ==================" <<  endl;

	
/********************************************************************************************************
*		Workers run this code
********************************************************************************************************/
}else if(my_rank > 0){ 

	pcl::console::TicToc tt;


   float buff [80000]; 
   int count;
   MPI_Recv(&count, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
   MPI_Recv(&buff, count, MPI_FLOAT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
  
 	Segmentation seg;
 	seg.build_cloud(buff, count);

 	//cout << "Manage to build cloud! " << seg.cloud.size() << " points." << endl;
 	seg.ransac(0.25, 100); // double Threshhold, int max_number_of_iterations

 	float buffer[200];
 	double eps = 0.6;
 	int minCl = 30;
 	
 	int bsize = seg.euclidian(buffer, eps, minCl); // Returns size of float buffer

	//Send back boxes of found clusters to master
	int root = 0;
	//MPI_Send(&buffer, bsize, MPI_FLOAT, root, 0, MPI_COMM_WORLD); // used with db scan
	MPI_Send(&buffer, bsize  , MPI_FLOAT, root, 0, MPI_COMM_WORLD);// Used with euclidian 
}
//******************************************************************************************************
// End MPI
MPI_Finalize ();
return 0;
}

