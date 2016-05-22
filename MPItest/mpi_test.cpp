#include <pcl/console/time.h>
#include <string>
#include <vector>
#include <sstream>
#include <mpi.h>
#include <algorithm>    // std::sort

#include "filter.h"
#include "segmentation.h"

#define OFFSET 0.08
#define SECTORS 8
#define FILE_READ_PROCESS 0
#define RECEIVER_PROCESS 1


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
	int nth_point = 4; // five is default
	int num_files = 1;
	double eps = 0.5;
	int minCl = 50;
	int db_threads = 2;
	
	std::string infile = "../../Dataframes/";
	std::string bin = ".bin";
	std::string zeros = "000000000"; // 9 zeros

	for (int i = 1; i < argc; i++) { 
        if (i != argc) // Check that we haven't finished parsing already
            if(std::strcmp(argv[i], "-d") == 0) {
                dbscan = true;
                sscanf(argv[i+1], "%i", &db_threads);
            } else if(std::strcmp(argv[i], "-n") == 0){
					sscanf(argv[i+1], "%i", &nth_point);
			}else if(std::strcmp(argv[i], "-i") == 0){
				num_files = atoi(argv[i+1]);
        }
        //std::cout << argv[i] << " ";
	}


	// MPI initializations
	MPI_Status status;
	MPI_Init (&argc, &argv);
	MPI_Comm_size (MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank (MPI_COMM_WORLD, &my_rank);
	pcl::console::TicToc tt;

	
/*******************************************************************************************
*		Master runs this code
********************************************************************************************/
	if(my_rank == FILE_READ_PROCESS){ 

	tt.tic();
	 
	 Filters filt;
	 std::ostringstream os;
	 for(int k = 0; k < num_files; k++){
		 
  		 os << k;
	 	 zeros.append(os.str());
	 	 zeros.append(bin);
	 	 if(k > 9)
	 	 	zeros = zeros.substr(1,zeros.length());

	 	 if(k > 99)
	 	 	zeros = zeros.substr(1,zeros.length());

	 	 infile.append(zeros);

		 //int read_file;
		// tt.tic();	  

		 // Read file and create 8 point clouds
		 // Nth_point will be kept from the data e.g. 3, every third point will be used
		 filt.read_file(infile, nth_point);
		 filt.filter_and_slice();

		 //read_file = tt.toc();

		 // Sort the slices in decending order so the one with highest amount of points
		 // gets processed first
		 std::sort(filt.floats.begin(),filt.floats.end(),less_vectors);

		if(k != 0)// WAIT FOR ACK FROM MERGER TO START AGAIN, WILL BLOCK HERE!
		{
			bool nextfile;	
			MPI_Recv(&nextfile, 1, MPI_INT, RECEIVER_PROCESS, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
		}
		
		
		for(int i = 0; i < SECTORS; i++){ 
		   int bsize = filt.floats.at(i).size();
		   MPI_Send(&bsize, 1, MPI_INT, (i+2), 0, MPI_COMM_WORLD);
		   float *f = &filt.floats.at(i)[0];
		   MPI_Send(f, bsize, MPI_FLOAT, (i+2), 0, MPI_COMM_WORLD);
		}
		//int sending = tt.toc(); 
		os.str("");
		infile = "../../Dataframes/";
		zeros = "000000000";

		//cout << "Read file and filter:\t\t" << read_file << " ms" << endl;
		//cout << "Sending data-loop:\t\t" << sending << " ms" << endl;
	}
	cout << "Total time:" << tt.toc() << " ms"<< endl;


/********************************************************************************************************
*		Collector node runs this code, merges the result and saves to file if specified
********************************************************************************************************/
}else if(my_rank == RECEIVER_PROCESS){
	
	std::vector<object> objects; 

	for(int k = 0; k < num_files; k++){

		tt.tic();
		int clusterCount = 0;
		for(int i = 0; i < SECTORS ; i++){ 
			int number_amount;
			MPI_Status status;
			MPI_Probe(MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &status);
			MPI_Get_count(&status, MPI_FLOAT, &number_amount);

			// Allocate a buffer to hold the incoming numbers
			float* number_buf = (float*)malloc(sizeof(float) * number_amount);
			// Now receive the message with the allocated buffer
			MPI_Recv(number_buf, number_amount, MPI_FLOAT, status.MPI_SOURCE, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

			// Loop through all received numbers from this sector and create object representations
			for(int j = 0; j < number_amount; j++){
				object c;
				if(j % 6 == 0){
					c.minPt = pcl::PointXYZ(number_buf[j], number_buf[j+1], number_buf[j+2]);
					c.maxPt = pcl::PointXYZ(number_buf[j+3], number_buf[j+4], number_buf[j+5]);
					objects.push_back(c);
				}
			}

			clusterCount = clusterCount + (number_amount/6);
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

	  	//Count number of merged objects.
	  	int clusters = 0;
		  	for (int i = 0; i < objects.size(); ++i)
	  			if(!objects.at(i).remove) clusters++;

	  	objects.clear();

	  	// TELL FILE_READER THAT WE'RE DONE
	  	bool nextfile = true;
	  	MPI_Send(&nextfile, 1, MPI_INT, FILE_READ_PROCESS, 0, MPI_COMM_WORLD);
	  	cout << "Found:" << clusters << " in " << tt.toc() << " ms"<< endl;

	}


/********************************************************************************************************
*		Workers run this code
********************************************************************************************************/
}else if(my_rank > 1){ 

	for(int k = 0; k < num_files; k++){

		pcl::console::TicToc tt;


	   float buff [80000]; 
	   int count;
	   MPI_Recv(&count, 1, MPI_INT, FILE_READ_PROCESS, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	   MPI_Recv(&buff, count, MPI_FLOAT, FILE_READ_PROCESS, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

	 	Segmentation seg;
	 	seg.build_cloud(buff, count);
	 	//seg.build_cloud_two(number_buf, number_amount);
	 	//free(number_buf);

	 	//cout << "Manage to build cloud! " << seg.cloud.size() << " points." << endl;
	 	seg.ransac(0.25, 100); // double Threshhold, int max_number_of_iterations

	 	
	 	float buffer[200];
	 	if(dbscan){
	 		minCl = 20;
	 		int bsize = seg.dbscan(buffer,eps,minCl, 2); // Returns size of float buffer
	 		MPI_Send(&buffer, bsize, MPI_FLOAT, RECEIVER_PROCESS, 0, MPI_COMM_WORLD); // used with db scan 
	 	}else{
	 		int bsize = seg.euclidian(buffer, eps, minCl); // Returns size of float buffer
	 		MPI_Send(&buffer, bsize, MPI_FLOAT, RECEIVER_PROCESS, 0, MPI_COMM_WORLD);// Used with euclidian
	 	}
	}
}
//******************************************************************************************************
// End MPI
MPI_Finalize ();
return 0;
}

