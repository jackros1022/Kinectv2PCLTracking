// #include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <fstream>
#include <iostream>
#include <string>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "PointCloudConverter.h"
#include "SegmentationTest.h"
#include "FilterTest.h"

using namespace std;

int main() {

	// init point cloud
	PointCloudConverter pcconverter;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcconverter.initializeCloudFromDFR("frames/testFrame.dfr");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcconverter.initializeCloudFromDFR("frames/tisch_mit_alles.dfr");

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcconverter.initializeCloudFromDFR("tisch_mit_alles_von_hinten.dfr");
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcconverter.initializeCloudFromDFR("frames/cylinder_in_the_air.dfr");

	//pcconverter.initializeCloudFromTXT("clouddata.txt");

	/*cerr << "PointCloud has: " << cloud->points.size() << " data points." << endl;*/

	/*SegmentationTest segTest;
	segTest.runCylinderSegmentation(cloud);*/

	/*FilterTest filterTest;
	filterTest.conditionalRemoval(cloud);*/

	/**/

	////create & use Cloud-Viewer
	//pcl::visualization::CloudViewer viewer("cloud viewer");
	//viewer.showCloud(cloud);
	//while (!viewer.wasStopped());

	////create & use PassThrough
	//FilterTest filterTest;
	//filterTest.passThroughCloud(cloud);

	////create & use Radius-Outlier-Filter
	//FilterTest filterTest;
	//filterTest.radiusOutlierRemoval(cloud);

	////create & use Static-Outlier-Filter
	//FilterTest filterTest;
	//filterTest.staticOutlierFilter(cloud);

	//create & use Voxel_Grid_Filter
	FilterTest filterTest;
	filterTest.voxelGrid(cloud);

	return 0;

}
