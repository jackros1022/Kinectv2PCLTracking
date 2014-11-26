// #include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <fstream>
#include <iostream>
#include <string>

#include <pcl/filters/passthrough.h>

#include "PointCloudConverter.h"
#include "SegmentationTest.h"
#include "FilterTest.h"

using namespace std;

int main() {

	// init point cloud
	PointCloudConverter pcconverter;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcconverter.initializeCloudFromDFR("frames/testFrame.dfr");
	//pcconverter.initializeCloudFromTXT("clouddata.txt");

	cerr << "PointCloud has: " << cloud->points.size() << " data points." << endl;

	//SegmentationTest segTest;
	//segTest.runSegmentation(cloud);

	//FilterTest filterTest;
	//filterTest.testPassThrough();

	// create cloud viewer
	pcl::visualization::CloudViewer viewer("cloud viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped());
	

	return 0;
}