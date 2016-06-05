#!/bin/sh

STARTTIME=$(date +%s)

# THIS NEEDS TO BE MODIFIED DEPENDING ON UNIT
FILES=/home/martin/99LIDARS/Dataframes/*


echo 'Running all files with euclidian'

mkdir output


OUTPUTF2=output/euclidian_n4.txt

for file in $FILES
do
	echo "processing -n 4 " ${file##*/} ...
	./filter -n 4 -e 0.75 -m 13 -i ${file##*/} >> $OUTPUTF2
	echo "\n" >> $OUTPUTF2
done


echo "Running all files with dbscan"

OUTPUTF4=output/dbscan_n4.txt

for file in $FILES
do
	echo "processing -n 4 " ${file##*/} ...
	./filter -d -n 4 -e 0.55 -m 9 -i ${file##*/} >> $OUTPUTF4
	echo "\n" >> $OUTPUTF4
done

OUTPUTF22=output/euclidian_n4_truth_merge.txt

for file in $FILES
do
	echo "processing -n 4 " ${file##*/} ...
	./filter_truth -n 4 -e 0.75 -m 13 -i ${file##*/} -x >> $OUTPUTF22
	echo "\n" >> $OUTPUTF22
done


echo "Running all files with dbscan"

OUTPUTF44=output/dbscan_n4_truth_merge.txt

for file in $FILES
do
	echo "processing -n 4 " ${file##*/} ...
	./filter_truth -d -n 4 -e 0.55 -m 9 -i ${file##*/} -x >> $OUTPUTF44
	echo "\n" >> $OUTPUTF44
done

ENDTIME=$(date +%s)
echo "Done in $(($ENDTIME - $STARTTIME)) seconds"
