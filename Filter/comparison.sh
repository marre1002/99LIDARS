#!/bin/sh

STARTTIME=$(date +%s)

# THIS NEEDS TO BE MODIFIED DEPENDING ON UNIT
FILES=/home/mpiu/99LIDARS/Dataframes/*


echo 'Running all files with euclidian'

mkdir output

OUTPUTF1=output/euclidian_n1.txt


for file in $FILES
do
	echo "processing -n 1 " ${file##*/} ...
		./filter_truth -n 1 -i ${file##*/} >> $OUTPUTF1
done

OUTPUTF2=output/euclidian_n2.txt

for file in $FILES
do
	echo "processing -n 2 " ${file##*/} ...
		./filter_truth -n 4 -i ${file##*/} >> $OUTPUTF2
done

OUTPUTF4=output/euclidian_n4.txt

for file in $FILES
do
	echo "processing -n 4 " ${file##*/} ...
		./filter_truth -n 4 -i ${file##*/} >> $OUTPUTF4
done


echo "Running all files with dbscan"

OUTPUTFD1=output/dbscan_n1.txt

for file in $FILES
do
	echo "processing -n 1 " ${file##*/} ...
		./filter_truth -d -n 1 -i ${file##*/} -t 8 >> $OUTPUTFD1
done

OUTPUTFD2=output/dbscan_n2.txt

for file in $FILES
do
	echo "processing -n 2 " ${file##*/} ...
		./filter_truth -d -n 1 -i ${file##*/} -t 8 >> $OUTPUTFD2
done

OUTPUTFD4=output/dbscan_n4.txt

for file in $FILES
do
	echo "processing -n 4 " ${file##*/} ...
		./filter_truth -d -n 4 -i ${file##*/} -t 8 >> $OUTPUTFD4
done


ENDTIME=$(date +%s)
echo "Done in $(($ENDTIME - $STARTTIME)) seconds"
