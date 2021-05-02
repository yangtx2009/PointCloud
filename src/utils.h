#ifndef UTILS_
#define UTILS_

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

void createColorPointByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr, 
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_ptr,
                                float max_distance = 2.5);

#endif