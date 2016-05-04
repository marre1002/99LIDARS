#!/bin/sh

OUTPUTF='output.txt'
rm $OUTFILE

for i in 1 2 4 6 8 10
do
	mpirun -np 9 -host worker1 ./my_mpi -n $i -i '0000000001.bin' >> $OUTPUTF
	echo "worker 1 " + $i
done

for i in 1 2 4 6 8 10
do
	mpirun -np 9 -host worker1,worker2 ./my_mpi -n $i -i '0000000001.bin' >> $OUTPUTF
	echo "worker 1 & 2 " + $i
done

for i in 1 2 4 6 8 10
do
	mpirun -np 9 -host worker1 ./my_mpi -n $i -i '0000000009.bin' >> $OUTPUTF
	echo "worker 1 " + $i
done

for i in 1 2 4 6 8 10
do
	mpirun -np 9 -host worker1,worker2 ./my_mpi -n $i -i '0000000009.bin' >> $OUTPUTF
	echo "worker 1 & 2 " + $i
done

for i in 1 2 4 6 8 10
do
	mpirun -np 9 -host worker1 ./my_mpi -n $i -i '0000000015.bin' >> $OUTPUTF
	echo "worker 1 " + $i
done

for i in 1 2 4 6 8 10
do
	mpirun -np 9 -host worker1,worker2 ./my_mpi -n $i -i '0000000015.bin' >> $OUTPUTF
	echo "worker 1 & 2 " + $i
done
