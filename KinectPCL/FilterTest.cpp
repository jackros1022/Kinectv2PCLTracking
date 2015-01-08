#include "FilterTest.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

FilterTest::FilterTest(void)
{
}


FilterTest::~FilterTest(void)
{
}



//----- PassThrough-Filter -----
void FilterTest::passThroughCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	cerr << "PathThrough is run!" << endl;
	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "Cloud before filtering: " << cloud->points.size() << std::endl;

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0 * 300);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);


	std::cerr << "Cloud after filtering: " << cloud_filtered->points.size () << std::endl;

	pcl::visualization::CloudViewer viewer("cloud viewer");
	//viewer.showCloud(cloud);
	viewer.showCloud(cloud_filtered);
	while (!viewer.wasStopped());

	//cin.get();

}



//----- Radius-Outlier-Filter -----
void FilterTest::radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	cerr << "Radius-Outlier-Filter is run!" << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "Cloud before filtering: " << cloud->points.size() << std::endl;

	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	// build the filter
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(0.7);
	outrem.setMinNeighborsInRadius (2);
	// apply filter
	outrem.filter (*cloud_filtered);

	std::cerr << "Cloud after filtering: " << cloud_filtered->points.size () << std::endl;

	pcl::visualization::CloudViewer viewer("cloud viewer");
	//viewer.showCloud(cloud);
	viewer.showCloud(cloud_filtered);
	while (!viewer.wasStopped());

	cin.get();

}



//----- Static-Outlier-Filter -----
void FilterTest::staticOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	cerr << "Static-Outlier-Filter is run!" << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "Cloud before filtering: " << cloud->points.size() << std::endl;

	//----------------------------------------filter
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (1);
	sor.setStddevMulThresh (0.1);
	sor.filter (*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

	sor.setNegative (true);
	sor.filter (*cloud_filtered);

	//----------------------------------output
	std::cerr << "Cloud after filtering: " << cloud_filtered->points.size () << std::endl;

	pcl::visualization::CloudViewer viewer("cloud viewer");
	//viewer.showCloud(cloud);
	viewer.showCloud(cloud_filtered);
	while (!viewer.wasStopped());

	cin.get();

}



//----- Conditional-Removal-Filter -----
void FilterTest::conditionalRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	cerr << "Conditional-Removal-Filter" << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "Cloud before filtering: " << cloud->points.size() << std::endl;

	// build the condition
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
		pcl::ConditionAnd<pcl::PointXYZ> ());
	range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
		pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
	range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
		pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));

	// build the filter
	pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
	condrem.setInputCloud (cloud);
	condrem.setKeepOrganized(true);
	// apply filter
	condrem.filter (*cloud_filtered);

	std::cerr << "Cloud after filtering: " << cloud_filtered->points.size () << std::endl;

	pcl::visualization::CloudViewer viewer("cloud viewer");
	//viewer.showCloud(cloud);
	viewer.showCloud(cloud_filtered);
	while (!viewer.wasStopped());



	cin.get();

}



//----- VoxelGrid filter -----
void FilterTest::voxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	cerr << "Voxel_Grid_Filter is run!" << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "Cloud before filtering: " << cloud->points.size() << std::endl;

	//----------------------------------------filter
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (3.0f, 3.0f, 3.0f);
	sor.filter (*cloud_filtered);

	//----------------------------------output
	std::cerr << "Cloud after filtering: " << cloud_filtered->points.size () << std::endl;

	pcl::visualization::CloudViewer viewer("cloud viewer");
	//viewer.showCloud(cloud);
	viewer.showCloud(cloud_filtered);
	while (!viewer.wasStopped());

	cin.get();

}