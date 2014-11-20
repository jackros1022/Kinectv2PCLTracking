#pragma once

#include <memory>
#include <array>

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <atlbase.h>

#include <Kinect.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Kinect {
public:
	Kinect();
	~Kinect();

	int getNumKinectAvailable(); 
	bool init(int index);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr grabFrame();

private:	
	enum Streams {
		COLOR = 0,
		DEPTH = 1
	};

	CComPtr<INuiSensor> nuiSensor;

	std::array<HANDLE, 2> readyHandles;
	std::array<HANDLE, 2> streamHandles;
	std::array<NUI_IMAGE_FRAME, 2> frames;
};
