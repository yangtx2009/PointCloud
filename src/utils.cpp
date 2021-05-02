#include "utils.h"

void createColorPointByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr, 
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_ptr,
                            float max_distance)
{
    pcl::PointXYZ origin(0, 0, 0);

    for(pcl::PointXYZ basic_point: basic_cloud_ptr->points)
    {
        pcl::PointXYZRGB point;
        if (std::isnan(basic_point.x))
            continue;
        point.x = basic_point.x;
        point.y = basic_point.y;
        point.z = basic_point.z;

        float distance = std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z);

        float value = static_cast<std::uint8_t>(std::fmax(0.0, (max_distance - distance)/max_distance) * 255.0);

        std::uint32_t rgb = (static_cast<std::uint32_t>(value) << 16 |
                    static_cast<std::uint32_t>(value) << 8 | static_cast<std::uint32_t>(value));
        point.rgb = *reinterpret_cast<float*>(&rgb);
        color_cloud_ptr->points.push_back (point);
    }

}