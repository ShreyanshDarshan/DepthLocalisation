#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <bits/stdc++.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>


#define resolution 2.0f

using namespace std;

pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

float dist(pcl::PointXYZ a, pcl::PointXYZ b)
{
	return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
}

int i = 0;
pcl::PointCloud<pcl::PointXYZ>::Ptr globalmap(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr localmap(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCLPointCloud2 in_cloud;

// void pc2_to_pcl(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
// {
// 	pcl_conversions::toPCL(*input, in_cloud);
// 	pcl::fromPCLPointCloud2(in_cloud, *globalmap);
// }

void extract_local_map(float X, float Y, float Z, float radius)
{
	pcl::PointXYZ searchPoint;

	searchPoint.x = X;
	searchPoint.y = Y;
	searchPoint.z = Z;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// std::cout << "Neighbors within radius search at (" << searchPoint.x
	// 		  << " " << searchPoint.y
	// 		  << " " << searchPoint.z
	// 		  << ") with radius=" << radius << std::endl;

	if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		// Generate pointcloud data
		localmap->width = pointIdxRadiusSearch.size();
		localmap->height = 1;
		localmap->points.resize (localmap->width * localmap->height);
		
		for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
		{
			localmap->points[i] =  globalmap->points[pointIdxRadiusSearch[i]];
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "edge_pub");
	// ros::package::getPath('PKG_NAME')
	pcl::io::loadPCDFile<pcl::PointXYZ> ("/media/shreyanshdarshan/New Volume/vision/PCL/XYZ2PCD/build/pepsi_down.pcd", *globalmap);

	ros::NodeHandle n;
	// ros::Subscriber sub = n.subscribe("/velodyne_points", 1000, pc2_to_pcl);
	ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("/local_map", 1000);
	sensor_msgs::PointCloud2 localmap_msg;

	octree.setInputCloud(globalmap);
	octree.addPointsFromInputCloud();

	float x=0, y=0, z=0;
	while (ros::ok())
	{
		x+=0.01;
		extract_local_map (x, y, z, 20);
		pcl::toROSMsg(*localmap.get(), localmap_msg);
		localmap_msg.header.frame_id = "odom";
		map_pub.publish(localmap_msg);
		//cout<<object_msg;
		ros::spinOnce();
		// break;
	}
	return 0;
}