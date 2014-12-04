#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <vector>

class SegmentationTest
{
public:
	SegmentationTest();
	~SegmentationTest(void);
	//void runSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void runCylinderSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void setupCamera(boost::shared_ptr<pcl::visualization::PCLVisualizer>);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

