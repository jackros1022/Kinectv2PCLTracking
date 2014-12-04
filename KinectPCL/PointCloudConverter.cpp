#include "PointCloudConverter.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/algorithm/string.hpp>
#include <sstream>

#include <fstream>
#include <iostream>

using namespace std;

PointCloudConverter::PointCloudConverter(void) : cloud(new pcl::PointCloud<pcl::PointXYZ>)
{

}


PointCloudConverter::~PointCloudConverter(void)
{
}

void PointCloudConverter::initializeCloudFromTXT(string fileName) {
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	float maxZ = 0;
	float cubeSize = 500;
	string line;
	ifstream clouddata;
	clouddata.open (fileName, ios::in);
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
	//pcl::visualization::CloudViewer viewer("Cloud Viewer");
	//viewer.showCloud(cloud);
	//while (!viewer.wasStopped ());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudConverter::initializeCloudFromDFR(std::string fileName) {
	string line;
	std::vector<std::string> splitLine;
	stringstream sstr;
	float x;
	float y;
	float z;
	ifstream clouddata;
	clouddata.open (fileName, ios::in);
	int lineCounter = 0;

	if(clouddata.is_open()) {
		while(getline(clouddata,line)) {
			if(line.find("INF") == string::npos) {
				boost::split(splitLine, line, boost::is_any_of(";"));
				x = stof(splitLine[0]);
				y = stof(splitLine[1]);
				z = stof(splitLine[2]);
				cloud->push_back(pcl::PointXYZ(x * -200,y * -200,z * 200));
				//cout << ++lineCounter << ": " << x << ";" << y << ";" << z << endl;
				splitLine.clear();
			}
		}
		clouddata.close();

		return cloud;
	}

}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudConverter::getPointCloud() {
	return cloud;
}