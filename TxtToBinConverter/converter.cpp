#include <string>
#include <vector>
#include <pcl/console/parse.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <stdio.h>

using namespace std;

int main (int argc, char** argv)
{
  	//std::string infile = "../../Dataframes_txt/";
  	//string filename = "0000000000.txt";
  	string filename;
	filename.assign(argv[1]);
	
	string outputfile = filename.substr(0,10);
	outputfile.append(".bin");
	
	FILE* file = fopen(filename.c_str(), "r");
	if (NULL == file) {
	   cerr << "Could not read file: " << filename << endl;
	   return 0;
	}
	
	std::vector<float> vec;
	float intensity;

	
	float x;
	float y;
	float z;
	while(fscanf(file,"%f %f %f %f\n", &x, &y, &z, &intensity) == 4) {
		vec.push_back(x);
		vec.push_back(y);
		vec.push_back(z);
	}
	fclose(file);


	float *f = &vec[0];


	// Write to bin file

	 FILE * pFile;
	 pFile = fopen (outputfile.c_str(), "wb");
	 fwrite (f , sizeof(float),vec.size(), pFile);
	 
	 fclose (pFile);

  return 0;

}