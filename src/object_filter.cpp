#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class ObjectFilterPCL{
    private:
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_rgb_;
    public:
        ObjectFilterPCL(){

        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr operator()(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_in){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_rgb_;

            //Voxel (pcl_rgb_raw -> pcl_rgb)
            pcl::VoxelGrid<pcl::PointXYZRGB> vox;
            vox.setInputCloud(pcl_rgb_raw_);
            vox.setLeafSize(0.05,0.05,0.05);
            vox.filter(*pcl_rgb_);

            // filter by Z/H
            pcl::PointIndices::Ptr idx (new pcl::PointIndices ());
            const float H_MIN = 60;
            const float H_MAX = 150;
            const float S_MIN = 50 / 255.0;
            const float S_MAX = 180 / 255.0;
            const float V_MIN = 0 / 255.0;
            const float V_MAX = 255 / 255.0;
            const float Z_MIN = -0.1;
            const float Z_MAX = 0.1;

            for(size_t i=0; i<pcl_hsv_->points.size(); ++i){
                if(
                        pcl_hsv_->points[i].h >= H_MIN &&
                        pcl_hsv_->points[i].h < H_MAX &&
                        pcl_hsv_->points[i].s >= S_MIN &&
                        pcl_hsv_->points[i].s < S_MAX &&
                        pcl_hsv_->points[i].v >= V_MIN &&
                        pcl_hsv_->points[i].v < V_MAX &&
                        pcl_rgb_->points[i].z > Z_MIN &&
                        pcl_rgb_->points[i].z < Z_MAX
                  ){
                    idx->indices.push_back(i);
                }
            }

            // extract indices
            pcl::ExtractIndices<pcl::PointXYZRGB> ex;
            ex.setInputCloud(pcl_rgb_);
            ex.setNegative(false);
            ex.setIndices(idx);
            ex.filter(*_pcl_grs); // pcl-grass
        }
};

class ObjectFilterROS{
    private:
        // ros handle
        ros::NodeHandle nh_;
        ros::Subscriber pcl_sub_;
        ros::Publisher pcl_pub_;
        tf::TransformListener tfl_;
        tf::TransformBroadcaster tfb_;

        // ros data
        sensor_msgs::PointCloud2::Ptr pcl_in_;
        sensor_msgs::PointCloud2::Ptr pcl_out_;

        // data handle
        ObjectFilterPCL filter_;
        bool new_cloud_;

    public:
        ObjectFilterROS(ros::NodeHandle nh): nh_(nh){
            // Define Publishers and Subscribers here
            pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_out", 3);
            pcl_sub_ = nh_.subscribe("cloud_in", 3, &ObjectFilterROS::pcl_cb, this);
            new_cloud_ = false;
        }

        void publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
            // publish output
            pcl::toROSMsg (*cloud, *pcl_out_);
            pcl_out_->header.seq += 1;
            pcl_out_->header.stamp = pcl_in_->header.stamp;
            pcl_out_->header.frame_id = pcl_in_->header.frame_id;
            pcl_pub_.publish(pcl_out_);
        }

        void pcl_cb(const sensor_msgs::PointCloud2ConstPtr& msg_in){
            pcl_in_.reset(new sensor_msgs::PointCloud2);
        }

        void step(){
            ros::Rate rate(50);
            if(new_cloud_){
                this->publish( filter_(pcl_in_));
                new_cloud_ = false;
            }
            rate.sleep();
            ros::spinOnce();
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_filter");
    ros::NodeHandle nh;
    ObjectFilterROS filter(nh);

    // Spin until ROS is shutdown
    while (ros::ok())
        ros::spin();

    return 0;
}
