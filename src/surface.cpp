#include "surface.h"
#include <pcl/point_types.h>

void generateSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, double search_radius)
{
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (point_cloud_ptr);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (search_radius);
    ne.compute (*cloud_normals);
}