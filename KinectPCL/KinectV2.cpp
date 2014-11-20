#include "KinectV2.h"

struct KinectRGBPixel {
	unsigned char b;
	unsigned char g;
	unsigned char r;
	unsigned char x;
};

Kinect::Kinect() {
	readyHandles[COLOR] = CreateEvent(nullptr, TRUE, FALSE, nullptr);
	readyHandles[DEPTH] = CreateEvent(nullptr, TRUE, FALSE, nullptr);
}

Kinect::~Kinect() {
	CloseHandle(readyHandles[COLOR]);
	CloseHandle(readyHandles[DEPTH]);
}

int Kinect::getNumKinectAvailable() {
	int sensorCount = 0;
	NuiGetSensorCount(&sensorCount);
	return sensorCount;
}

bool Kinect::init(int index) {
	HRESULT hr = NuiCreateSensorByIndex(index, &nuiSensor);
	if(FAILED(hr)) {
		return false;
	}

	hr = nuiSensor->NuiStatus();
	if(FAILED(hr)) {
		nuiSensor.Release();
		return false;
	}

	hr = nuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
	if(FAILED(hr)) {
		return false;
	}

	hr = nuiSensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, readyHandles[COLOR], &streamHandles[COLOR]);
	if(FAILED(hr)) {
		return false;
	}

	hr = nuiSensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480, 0, 2, readyHandles[DEPTH], &streamHandles[DEPTH]);
	if(FAILED(hr)) {
		return false;
	}

	hr = nuiSensor->NuiImageStreamSetImageFrameFlags(streamHandles[DEPTH], NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);
	if(FAILED(hr)) {
		return false;
	}

	return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect::grabFrame() {
	DWORD waitResult = WaitForMultipleObjects(2, readyHandles.data(), TRUE, 5000);
	if(waitResult != WAIT_OBJECT_0 ) {
		return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(nullptr);
	}

	HRESULT hr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
	
	hr = nuiSensor->NuiImageStreamGetNextFrame(streamHandles[COLOR], 0, &frames[COLOR]);
    if (FAILED(hr)) {
		return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(nullptr);
    }
	hr = nuiSensor->NuiImageStreamGetNextFrame(streamHandles[DEPTH], 0, &frames[DEPTH]);
    if (FAILED(hr)) {
		return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(nullptr);
    }

	NUI_LOCKED_RECT colorLockRect, depthLockRect;
	hr = frames[COLOR].pFrameTexture->LockRect(0, &colorLockRect, nullptr, 0); 
	if(FAILED(hr) || colorLockRect.Pitch == 0) {
		return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(nullptr);
	}
	hr = frames[DEPTH].pFrameTexture->LockRect(0, &depthLockRect, nullptr, 0); 
	if(FAILED(hr) || depthLockRect.Pitch == 0) {
		return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(nullptr);
	}

	for(int row = 0; row < 480; ++row) {
		unsigned short * rowStart = (unsigned short *)(depthLockRect.pBits + depthLockRect.Pitch * row);

		for(int col = 0; col < 640; ++col) {

			auto pos = NuiTransformDepthImageToSkeleton(col, row, rowStart[col]);

			if(pos.z == 0.0f) {
				continue;
			}

			LONG colorX, colorY;
			NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
				NUI_IMAGE_RESOLUTION_640x480,
				NUI_IMAGE_RESOLUTION_640x480,
				&frames[COLOR].ViewArea,
				col,
				row,
				rowStart[col],
				&colorX,
				&colorY);

			pcl::PointXYZRGB point;
			point.x = pos.x;
			point.y = pos.y;
			point.z = pos.z;

			if(colorX >= 0 && colorX < 640 && colorY >= 0 && colorY < 480) {
				auto row = (KinectRGBPixel *)(colorLockRect.pBits + colorY * colorLockRect.Pitch);

				point.r = row[colorX].r;
				point.g = row[colorX].g;
				point.b = row[colorX].b;
			} else {
				point.r = 240;
				point.g = 32;
				point.b = 240;
			}

			cloud->push_back(point);
		}
	}
	
	frames[COLOR].pFrameTexture->UnlockRect(0);
	frames[DEPTH].pFrameTexture->UnlockRect(0);

	nuiSensor->NuiImageStreamReleaseFrame(streamHandles[COLOR], &frames[COLOR]);
	nuiSensor->NuiImageStreamReleaseFrame(streamHandles[DEPTH], &frames[DEPTH]);

	return cloud;
}
