#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

class DepthToPCLConverter
{
public:
    DepthToPCLConverter(ros::NodeHandle& nh)
    {
        depth_sub_ = nh.subscribe("/depth/image_raw", 1, &DepthToPCLConverter::depthCallback, this);
        camera_info_sub_ = nh.subscribe("/depth/camera_info", 1, &DepthToPCLConverter::cameraInfoCallback, this);
        pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud", 1);
        
        nh.param("min_dist", min_dist_, 0.1); 
        nh.param("max_dist", max_dist_, 7.0);

        has_camera_info_ = false;
    }

private:
    ros::Subscriber depth_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher pcl_pub_;
    bool has_camera_info_;
    float fx_, fy_, cx_, cy_;
    double min_dist_, max_dist_;

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
    {
        fx_ = camera_info->K[0];
        fy_ = camera_info->K[4];
        cx_ = camera_info->K[2];
        cy_ = camera_info->K[5];
        has_camera_info_ = true;
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg)
    {
        if (!has_camera_info_)
        {
            ROS_WARN("Waiting for camera info...");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        int width = cv_ptr->image.cols;
        int height = cv_ptr->image.rows;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->width = width;
        cloud->height = height;
        cloud->is_dense = false; 
        cloud->points.resize(width * height);

        for (int v = 0; v < height; ++v)
        {
            for (int u = 0; u < width; ++u)
            {
                float depth_value = cv_ptr->image.at<float>(v, u);

                if (std::isnan(depth_value) || depth_value <= 0.0 || depth_value < min_dist_ || depth_value > max_dist_)
                {
                    cloud->points[v * width + u].x = std::numeric_limits<float>::quiet_NaN();
                    cloud->points[v * width + u].y = std::numeric_limits<float>::quiet_NaN();
                    cloud->points[v * width + u].z = std::numeric_limits<float>::quiet_NaN();
                    continue;
                }

                float z = depth_value;
                float x = (u - cx_) * z / fx_;
                float y = (v - cy_) * z / fy_;

                cloud->points[v * width + u].x = x;
                cloud->points[v * width + u].y = y;
                cloud->points[v * width + u].z = z;
            }
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header = depth_msg->header;
        pcl_pub_.publish(output);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth2pcl");
    ros::NodeHandle nh;
    DepthToPCLConverter converter(nh);
    ros::spin();
    return 0;
}
