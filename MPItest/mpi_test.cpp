#include <iostream>
#include <mpi.h>

static int numprocs;

int main(int argc, char **argv) {
int my_rank;
// MPI initializations
MPI_Status status;
MPI_Init (&argc, &argv);
MPI_Comm_size (MPI_COMM_WORLD, &numprocs);
MPI_Comm_rank (MPI_COMM_WORLD, &my_rank);

// Get the name of the processor
char processor_name[MPI_MAX_PROCESSOR_NAME];
int name_len;
MPI_Get_processor_name(processor_name, &name_len);

// Print off a hello world message
printf("Hello world from processor %s ", processor_name);

double time_start = MPI_Wtime();
std::cout << "Hello World, my rank is " << my_rank <<" "<< MPI_Wtime() - time_start << std::endl;
// End MPI
MPI_Finalize ();
return 0;
}