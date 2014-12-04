#include "SegmentationTest.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

SegmentationTest::SegmentationTest(void)
{
}

SegmentationTest::~SegmentationTest(void)
{
}

void SegmentationTest::runCylinderSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

  float POINTDATA_SCALAR = 200.0;

 // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

  // Datasets
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  std::cerr << "LOADED PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  //// Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 300);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.43); 
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.8); 
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setMethodType (pcl::SAC_LMEDS);
  //seg.setMethodType (pcl::SAC_MSAC);
  
  seg.setNormalDistanceWeight (0.05);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.05 * POINTDATA_SCALAR); // distance threshold from each inlier point to the model no greater than 5cm
  seg.setRadiusLimits (0, 0.7 * POINTDATA_SCALAR); // radius of cyl. model smaller than 15cm (0.15)
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
  }

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	
	//setupCamera(viewer);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_green_color(cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_red_color(cloud, 255, 0, 0);
	
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "raw cloud");	
	viewer->addPointCloud<pcl::PointXYZ> (cloud_plane, single_green_color, "desk cloud");	
	viewer->addPointCloud<pcl::PointXYZ> (cloud_cylinder, single_red_color, "cylinder cloud");	

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "raw cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "desk cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cylinder cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	
	viewer->resetCamera ();

	while (!viewer->wasStopped ()) {
		viewer->spinOnce (100);
	}

}


void SegmentationTest::setupCamera(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
// show camera properties on console by hitting 'C' inside the viewer

	viewer->setBackgroundColor (0, 0, 255);
	viewer->camera_.clip[0] = 3.58123;   // A
	viewer->camera_.clip[1] = 3581.23;  // B
	viewer->camera_.pos[0] = -7.09619; // C
	viewer->camera_.pos[1] = 55.4104; // D
	viewer->camera_.pos[2] = 225.4168; // E
	viewer->camera_.focal[0] = -15.6291; // F
	viewer->camera_.focal[1] = 135.056; // G
	viewer->camera_.focal[2] = -268.34; // H
	viewer->camera_.view[0] = -0.0162736; // I
	viewer->camera_.view[1] = -0.965146; // J
	viewer->camera_.view[2] = -0.261205; // K
	viewer->camera_.fovy = 0.523599; // L
	viewer->updateCamera();  


	//Example:
	//
	//0.00522511,5.22511/0.0427789,-0.185814,0.0496169/0.0497412,-0.196849,-0.0978747/-0.0956887,-0.992963,0.0697719/0.523599/631,491/1650,152
	//
	//	==>
	//
	//m_pclVisualizer = new pcl::visualization::PCLVisualizer("PCL_Visualizer");
	//m_pclVisualizer->setBackgroundColor(0, 0, 0);
	//m_pclVisualizer->camera_.clip[0] = 0.00522511;
	//m_pclVisualizer->camera_.clip[1] = 5.22511;
	//m_pclVisualizer->camera_.focal[0] = 0.0497412;
	//m_pclVisualizer->camera_.focal[1] = -0.196849;
	//m_pclVisualizer->camera_.focal[2] = -0.0978747;
	//m_pclVisualizer->camera_.pos[0] = 0.0427789;
	//m_pclVisualizer->camera_.pos[1] = -0.185814;
	//m_pclVisualizer->camera_.pos[2] = 0.0496169;
	//m_pclVisualizer->camera_.view[0] = -0.0956887;
	//m_pclVisualizer->camera_.view[1] = -0.992963;
	//m_pclVisualizer->camera_.view[2] = 0.0697719;
	//m_pclVisualizer->camera_.fovy = 0.523599;
	//m_pclVisualizer->camera_.window_size[0] = 631;
	//m_pclVisualizer->camera_.window_size[1] = 491;
	//m_pclVisualizer->camera_.window_pos[0] = 1650;
	//m_pclVisualizer->camera_.window_pos[1] = 152;
	//m_pclVisualizer->updateCamera(); 

}