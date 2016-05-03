#!/bin/sh

OUTPUTF='output.txt'
rm $OUTFILE

for i in 1 2 3 4 5 6 7 8 9 10
do
	mpirun -np 2 -host master,worker1 ./my_mpi -n $i -e 0.6 >> $OUTPUTF
	echo "Euclidian running " + $i
done
