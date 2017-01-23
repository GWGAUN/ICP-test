/*
3D alignment of ball based on ICP with Normals. Visualisation 3D ball model before/after applying ICP
*/
#include <iostream>  
#include <string>  
#include <stdio.h>  
//soving Error: no override found for 'vtkPolyDataMapper' it must put before headerfile that using vtk
#define vtkRenderingCore_AUTOINIT 4(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingFreeTypeOpenGL,vtkRenderingOpenGL)
#define vtkRenderingVolume_AUTOINIT 1(vtkRenderingVolumeOpenGL)
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointXYZRGB PointT_color;
typedef pcl::PointCloud<PointT_color> PointCloud_color;

//get normals 
void addNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZ>);
	searchTree->setInputCloud(cloud);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(searchTree);
	normalEstimator.setKSearch(15);
	normalEstimator.compute(*normals);

	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
}


int main(int argc, char** argv)
{
	//load a 3D ball 
	PointCloud::Ptr cloudSource(new PointCloud); // point to 3D model
	PointCloud::Ptr cloudTarget(new PointCloud); // point to detected ball 

	cout << "Estimate ball center : " << endl;
	if (pcl::io::loadPLYFile("C:\\Users\\jsteil\\Desktop\\realSense trials\\3Dballalignment\\pcd file\\3D_half_model_z.ply", *cloudSource) < 0)
	{

		return (-1);
	}
	cout << "the point cloud contained in reference 3D ball is : " << cloudSource->points.size() << endl;

	if (pcl::io::loadPCDFile("C:\\Users\\jsteil\\Desktop\\realSense trials\\3Dballalignment\\pcd file\\pointcloud62.pcd", *cloudTarget) < 0)
	{

		return (-1);
	}
	cout << "the point cloud contained in detected ball is : " << cloudTarget->points.size() << endl;

	//remove outliers
	PointCloud::Ptr cloudTarget_filtered(new PointCloud);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_target;
	sor_target.setInputCloud(cloudTarget);
	sor_target.setMeanK(200);
	sor_target.setStddevMulThresh(0.01);
	sor_target.filter(*cloudTarget_filtered);

	//save point cloud of the detected ball
	pcl::io::savePLYFile("C:\\Users\\jsteil\\Desktop\\realSense trials\\3Dballalignment\\result\\pointcloudball62.ply", *cloudTarget_filtered);
	cout << "Point cloud of the detected ball saved." << endl;

	//initialisation of ICP
	// set detected ball center, got from closest point
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	//translation 
	transformation_matrix(0, 3) = -0.266047; // unit: m not mm
	transformation_matrix(1, 3) = 0.0195214;
	transformation_matrix(2, 3) = 0.883;

	PointCloud::Ptr cloudSourcetransformed(new PointCloud);
	pcl::transformPointCloud(*cloudSource, *cloudSourcetransformed, transformation_matrix);
	//save point cloud of the original position
	pcl::io::savePLYFile("C:\\Users\\jsteil\\Desktop\\realSense trials\\3Dballalignment\\result\\pointcloudOriginalPosition62.ply", *cloudSourcetransformed);
	cout << "Point cloud of the original position saved." << endl;

	// prepare could with normals
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source_normals(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target_normals(new pcl::PointCloud<pcl::PointNormal>());
	
	addNormal(cloudSourcetransformed, cloud_source_normals);
	addNormal(cloudTarget_filtered, cloud_target_normals);
	
	//ICP
	cout << "start ICP" << endl;
	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>::Ptr icp(new pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>());
	icp->setInputSource(cloud_source_normals);
	icp->setInputTarget(cloud_target_normals);
	/*
	//setting
	icp.setMaxCorrespondenceDistance(100);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.001);
	*/
	//icp.setMaximumIterations(1);
	pcl::PointCloud<pcl::PointNormal> Final;
	icp->align(Final);
	std::cout << "has converged:" << icp->hasConverged() << " score: " << icp->getFitnessScore() << std::endl;
	std::cout << "the rotation matrix and translation matrix is: " << endl;
	std::cout << icp->getFinalTransformation() << std::endl;
	Eigen::Matrix4f transformation = icp->getFinalTransformation();

	PointCloud::Ptr cloudSourcetransformedFinal(new PointCloud);
	pcl::transformPointCloud(*cloudSourcetransformed, *cloudSourcetransformedFinal, transformation);
	//save point cloud of the final position
	pcl::io::savePLYFile("C:\\Users\\jsteil\\Desktop\\realSense trials\\3Dballalignment\\result\\pointcloudbfinalposition62.ply", *cloudSourcetransformedFinal);
	cout << "Point cloud of the final position saved." << endl;

	//calculate ball center
	Eigen::Matrix3f rotationMatrix = transformation.block(0, 0, 3, 3);
	Eigen::Vector3f translator;
	translator << transformation(0, 3), transformation(1, 3), transformation(2, 3);
	Eigen::Vector3f center_before;
	center_before << transformation_matrix(0, 3), transformation_matrix(1, 3), transformation_matrix(2, 3);
	Eigen::Vector3f center_after;
	center_after = rotationMatrix * center_before + translator;
	cout << "center coordinate is : " << center_after << endl;

	//visulisation
	PointCloud_color::Ptr cloud_scene(new PointCloud_color); // point to scene
	if (pcl::io::loadPCDFile("C:\\Users\\jsteil\\Desktop\\realSense trials\\3Dballalignment\\pcd file\\pointcloudScene62.pcd", *cloud_scene) < 0)
	{

	return (-1);
	}
	//save point cloud of the scene
	pcl::io::savePLYFile("C:\\Users\\jsteil\\Desktop\\realSense trials\\3Dballalignment\\result\\pointcloudscene62.ply", *cloud_scene);
	cout << "Point cloud of the scene saved." << endl;

	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);

	//visualisation of scene
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_scene);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud_scene, rgb, "sample cloud");

	// visualisation of detected ball
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb0(cloudTarget_filtered, 0, 0, 255); //This will display the point cloud in green (R,G,B)
	viewer.addPointCloud<pcl::PointXYZ>(cloudTarget_filtered, rgb0, "cloud0");

	//First pointcloud
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb1(cloudSourcetransformed, 255, 0, 0); //This will display the point cloud in red (R,G,B)
	viewer.addPointCloud<pcl::PointXYZ>(cloudSourcetransformed, rgb1, "cloud1");

	// Second pointcloud
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb2(cloudSourcetransformedFinal, 0, 255, 0); //This will display the point cloud in green (R,G,B)
	viewer.addPointCloud<pcl::PointXYZ>(cloudSourcetransformedFinal, rgb2, "cloud2");

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud0");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}


	system("pause");
	return 0;
}
