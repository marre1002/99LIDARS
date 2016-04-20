#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>
#include <string>
#include <pcl/common/common.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/console/parse.h>
#include <sstream>
#include "include/util.h"

bool start = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

	if(event.getKeySym() == "s" && event.keyDown()){
		cout << "Here we go." << endl;
		start = true;
	}
	
}


int main (int argc, char** argv)
{




 	// Read in the cloud data
   	std::string binfile = "../../BinAndTxt/";
   	std::string txtfile = "../../BinAndTxt/";

   	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	//TODO: Implement some sort of control
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	int z = -1.5;
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(35,0,z), "aline");
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(0,30,z), "bline");
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(-35,0,z), "cline");
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(0,-30,z), "dline");
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(20,20,z), "eline");
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(-20,20,z), "fline");
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(20,-20,z), "gline");
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(0,0,z),pcl::PointXYZ(-20,-20,z), "hline");


   	for(int j= 0; j < 21; j++){

   		std::string curfile = "";

   		if(j >= 10){
   			curfile = "00000000" + to_string(j) + ".bin";
   		}else{
   			curfile = "000000000" + to_string(j) + ".bin";
   		}

   		
   		binfile = binfile + curfile;
   		cout << binfile << endl;

		// load point cloud
		fstream input(binfile.c_str(), ios::in | ios::binary);
		if(!input.good()){
			cerr << "Could not read file: " << binfile << endl;
			exit(EXIT_FAILURE);
		}
		input.seekg(0, ios::beg);

		//pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

		float ignore;
		int i;
		for (i=0; input.good() && !input.eof(); i++) {
			pcl::PointXYZ point;
			input.read((char *) &point.x, 3*sizeof(float));
			input.read((char *) &ignore, sizeof(float));
			cloud->push_back(point);
		}
		input.close();

		std::string curtxt;

		if(j >= 10){
			curtxt = "00000000" + to_string(j) + ".txt";
		}else{
			curtxt = "000000000" + to_string(j) + ".txt";
		}
		
		txtfile = txtfile + curtxt;
		//Get corresponding cluster file.
	  	fstream txtInput(txtfile.c_str(), ios::in);
		if(!txtInput.good()){
			cerr << "Could not read file: " << txtfile << endl;
			exit(EXIT_FAILURE);
		}

	  	std::vector<float> v;

	    float a;
	    while (txtInput >> a)
	    {
	        v.push_back(a);
	    }

	   	
	   	viewer->addPointCloud<pcl::PointXYZ> (cloud, "source");
	   	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.2f, 0.2f, 0.2f, "source");

	   	std::vector<pcl::PointXYZ> vpoints;


	   	std::stringstream ss;
	   	for(int h = 0 ; h < v.size(); h++){
	   			//Ensure that we read xmin, xmax, ymin, ymax, zmin, zmax as the file is structured.
	   			if(h%6 == 0){
	  				ss << "id" << h << "test";
	    			std::string str = ss.str();
	  				viewer->addCube(v.at(h), v.at(h+3), v.at(h+1), v.at(h+4), v.at(h+2), v.at(h+5), 1.0,0.0,0.0, str ,0);
	  			}
	    }	

		  //------------------------------------------------------------------------------------------------------------



		while(!viewer->wasStopped ()){
			while(start == false){
				viewer->spinOnce(1000);
			}
		   	viewer->spinOnce (10);
		   	viewer->removeAllShapes();
		   	viewer->removeAllPointClouds();
		   	break;
		   //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}



		binfile = "../../BinAndTxt/";
		txtfile = "../../BinAndTxt/";
  	}
  return (0);
}

