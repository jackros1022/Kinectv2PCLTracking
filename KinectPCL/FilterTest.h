#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <vector>

class FilterTest
{
public:
	FilterTest(void);
	~FilterTest(void);

	void testPassThrough(void);
	void passThroughCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
};

