// #include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <fstream>
#include <iostream>
#include <string>
#include "PointCloudConverter.h"

using namespace std;

int main() {

	PointCloudConverter pcconverter;
	pcconverter.initializeCloudFromDFR("frames/testFrame.dfr");
	//pcconverter.initializeCloudFromTXT("clouddata.txt");

	return 0;
}