#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <vector>

using namespace std;
//#include <pcl/visualization/cloud_viewer.h>


class PointCloudConverter
	
{
		
static const int cDepthWidth  = 512;
static const int cDepthHeight = 424;
//pcl::visualization::PCLVisualizer viewer;



public:
	PointCloudConverter(void);
	~PointCloudConverter(void);
	void initializeCloudFromTXT(std::string fileName);
	void initializeCloudFromDFR(std::string fileName);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

