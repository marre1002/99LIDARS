#!/bin/sh

OUTPUTF='output.txt'
rm $OUTFILE

for i in 1 2 3 4 5 6 7 8 9 10
do
	mpirun -np 9 -host worker1 ./my_mpi -n $i >> $OUTPUTF
	echo "worker 1 " + $i
done

for i in 1 2 3 4 5 6 7 8 9 10
do
	mpirun -np 9 -host worker1,worker2 ./my_mpi -n $i >> $OUTPUTF
	echo "worker 1 & 2 " + $i
done
