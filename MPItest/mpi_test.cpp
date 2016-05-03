#include <pcl/console/time.h>
#include <string>
#include <vector>
#include <sstream>
#include <mpi.h>
#include <pthread.h>

#include "filter.h"
#include "segmentation.h"


using namespace pcl;
using namespace std;

static int numprocs;

/* This struct gets passed on to every
	worker thread created. */
	struct thread_data{
		int thread_id;
	};

/**
*	Thread function, handles the segmentation of one sector
*/
void *segmentation(void *threadarg)
{
	pcl::console::TicToc tt;
	//tt.tic();

	struct thread_data *my_data;
    
    my_data = (struct thread_data *) threadarg;

    cout << "Hello from thread " << my_data->thread_id;
    // Receive sector from master
 	int count;
 	float buff [50000]; 
    MPI_Recv(&count, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
 	MPI_Recv(&buff, count, MPI_FLOAT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

 	Segmentation seg;
 	seg.build_cloud(buff, count);

 	seg.ransac(0.25, 100); // double Threshhold, int max_number_of_iterations

 	float buffer[200];
 	double eps = 0.6;
 	int minCl = 30;
 	int bsize = seg.euclidian(buffer, eps, minCl); // Returns size of float buffer

	//Send back boxes of found clusters to master
	int root = 0;
	//MPI_Send(&buffer, bsize, MPI_FLOAT, root, 0, MPI_COMM_WORLD); // used with db scan
	MPI_Send(&buffer, bsize  , MPI_FLOAT, root, 0, MPI_COMM_WORLD);// Used with euclidian

    
    std::cout << "Thread: " << my_data->thread_id << " done.\n";
    pthread_exit(NULL);
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
        }
        std::cout << argv[i] << " ";
	}

	  cout << endl << "Arg, n: " << nth_point << " eps: " << eps << " minCl: " << minCl << endl;

	// MPI initializations
	MPI_Status status;
	MPI_Init (&argc, &argv);
	MPI_Comm_size (MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank (MPI_COMM_WORLD, &my_rank);

	cout << "Assigning rank works!" <<  endl;

/*******************************************************************************************
*		Master runs this code
********************************************************************************************/
	if(my_rank == 0){ 

	  
	  std::vector<std::vector<float> > floats;
	  int m_tag = 0; // MPI message tag

	  std::string infile = "../../BinAndTxt/0000000001.bin";

	  // Read file and create 8 point clouds
	  // Nth_point will be kept from the data e.g. 3, every third point will be used
	  Filters filt;
	  filt.read_file(infile, nth_point);
	  filt.filter_and_slice(floats);

	  for (int i = 0; i < 7; ++i)
	  {
	  	cout << "Size of sector " << i << " is " << floats.at(i).size() << endl;
	  }
	  

	// Get the number of processes
	int world_size;
	int sectors = 8;
	MPI_Comm_size(MPI_COMM_WORLD, &world_size); 


	// Distribute the data/sectors of point cloud 
	for(int i = 0; i < sectors ; i++){ 
	   int receiver = i%(world_size-1);
	   int bsize = floats.at(i).size();
	   MPI_Send(&bsize, 1, MPI_INT, (receiver+1), m_tag, MPI_COMM_WORLD);
	   float* f = &floats.at(0)[0];
	   MPI_Send(&f, bsize, MPI_FLOAT, (receiver+1), m_tag, MPI_COMM_WORLD);
	}

	cout << "Sending went well!" << endl;

	 // Read new file while waiting...
	 
	 // Receive all sectors from workers...

	// This needs to be done 8 times

	
	for(int i = 0; i < sectors ; i++){ 
		int number_amount;
		MPI_Status status;
		MPI_Probe(MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &status);
		MPI_Get_count(&status, MPI_FLOAT, &number_amount);

		// Allocate a buffer to hold the incoming numbers
		float* number_buf = (float*)malloc(sizeof(float) * number_amount);
		// Now receive the message with the allocated buffer
		MPI_Recv(number_buf, number_amount, MPI_FLOAT, status.MPI_SOURCE, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);  
		free(number_buf);
		cout << "Master received values" << endl; 
	}

	
/********************************************************************************************************
*		Workers run this code
********************************************************************************************************/
}else if(my_rank == 1){ 

	/*pcl::console::TicToc tt;

	//Calculate how many pieces i get..
	//spwan that amount of threads
   int world_size;
   MPI_Comm_size(MPI_COMM_WORLD, &world_size);

   	int pieces_recv = 0;
  	 for(int i = 0; i < 8 ; i++){ // Loop through all the slices
	   if((i%(world_size-1)+1) == my_rank) pieces_recv++;
	}


	pthread_t threads[pieces_recv]; 
	struct thread_data td[pieces_recv];
	pthread_attr_t attr;
	int rc;
    void *status;

    //Initialize and set thread joinable
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    // Create new threads
    cout << "about to create threads" << endl;
	for(int i=0; i < pieces_recv; i++ ){
      td[i].thread_id = i;

      rc = pthread_create(&threads[i], NULL, segmentation, (void *)&td[i]);
      if (rc){
         cout << "Error:unable to create thread," << rc << endl;
         exit(-1);
      }
    }

    // free attribute and wait for the other threads
    pthread_attr_destroy(&attr);
    for(int i=0; i < pieces_recv; i++ ){
       rc = pthread_join(threads[i], &status);
       if (rc){
          cout << "Error:unable to join," << rc << endl;
          exit(-1);
       }
       //cout << "Main: completed thread id :" << i ;
       //cout << "  exiting with status :" << status << endl;
    }*/
}
//******************************************************************************************************
// End MPI
MPI_Finalize ();
return 0;
}

