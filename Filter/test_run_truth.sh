#!/bin/sh

STARTTIME=$(date +%s)

# THIS NEEDS TO BE MODIFIED DEPENDING ON UNIT
FILES=/home/martin/99LIDARS/Dataframes/*


echo 'Running all files with euclidian'

mkdir output_truth

OUTPUTF1=output_truth/euclidian_n1.txt


for file in $FILES
do
	echo "processing -n 1 " ${file##*/} ...
		./filter_truth -n 1 -i ${file##*/} >> $OUTPUTF1
done

OUTPUTF2=output_truth/euclidian_n4.txt

for file in $FILES
do
	echo "processing -n 4 " ${file##*/} ...
		./filter_truth -n 4 -i ${file##*/} >> $OUTPUTF2
done


echo "Running all files with dbscan"

OUTPUTF3=output_truth/dbscan_n1.txt

for file in $FILES
do
	echo "processing -n 1 " ${file##*/} ...
		./filter_truth -d -n 1 -i ${file##*/} >> $OUTPUTF3
done

OUTPUTF4=output_truth/dbscan_n4.txt

for file in $FILES
do
	echo "processing -n 4 " ${file##*/} ...
		./filter_truth -d -n 4 -i ${file##*/} >> $OUTPUTF4
done

zip -r output_truth{.zip,}

ENDTIME=$(date +%s)
echo "Done in $(($ENDTIME - $STARTTIME)) seconds"
