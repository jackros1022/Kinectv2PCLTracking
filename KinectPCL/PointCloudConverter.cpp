#include "PointCloudConverter.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>



#include <fstream>
#include <iostream>

using namespace std;

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

PointCloudConverter::PointCloudConverter(void)
{

}


PointCloudConverter::~PointCloudConverter(void)
{
}

void PointCloudConverter::initializeCloud() {
	

	// pcl::PointCloud<pcl::PointXYZ> cloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	float maxZ = 0;
	float cubeSize = 500;
	string line;
	ifstream clouddata;
	clouddata.open ("clouddata.txt", ios::in);
	std::vector<float> depthData;

	if (clouddata.is_open()) {

		int rowCount = 0;
		int colCount = 0;

		while ( getline (clouddata,line) ) {
			float z = std::stof(line);
			depthData.push_back(z);
			if(maxZ <= z) maxZ = z;
		}
		std::cout << maxZ << std::endl;
		for each (float z in depthData) {	
			float x = colCount * cubeSize / cDepthWidth;
			float y = rowCount * cubeSize / cDepthHeight;
			float newZ = z * cubeSize / maxZ;
		
			colCount++;

			if (colCount == cDepthWidth) { 
				colCount = 0; 
				rowCount++;
			}

			if(z != 0) cloud->push_back(pcl::PointXYZ(x,y,z));
		}
		clouddata.close();
	}


	// create cloud viewer
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

	viewer.showCloud(cloud);
      
	while (!viewer.wasStopped ());
   

	//for each (pcl::PointXYZ p in cloud->points) 
    //{  
	//	cout << p << endl;
    //}

	

}