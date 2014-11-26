#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <vector>

using namespace std;

class PointCloudConverter
	
{
static const int cDepthWidth  = 512;
static const int cDepthHeight = 424;

public:
	PointCloudConverter(void);
	~PointCloudConverter(void);
	void initializeCloudFromTXT(std::string fileName);
	pcl::PointCloud<pcl::PointXYZ>::Ptr initializeCloudFromDFR(std::string fileName);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud();

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

