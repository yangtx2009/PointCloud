#ifndef SURFACE_
#define SURFACE_

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

void generateSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, double search_radius);

#endif