/*
   Authored by Yoonyoung Cho @ 09.28.2018
   Heavily inspired by [tabletop_object_detector](https://github.com/ros-interactive-manipulation/pr2_object_manipulation/blob/groovy-devel/perception/tabletop_object_detector/src/tabletop_segmentation.cpp).





*/

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>

#include <pcl/point_types_conversion.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using point_t=pcl::PointXYZRGB;
using cloud_t=pcl::PointCloud<pcl::PointXYZRGB>;

class ObjectFilterPCL{
    private:
        cloud_t::Ptr pcl_rgb_;
    public:
        ObjectFilterPCL(){

        }

        void denoise(){

        }

        cloud_t::Ptr passthrough(cloud_t::Ptr cloud_in,
                float z_min=0.22,
                float z_max=1.0){ //for D415
            // setup
            pcl::PassThrough<point_t> pass_;
            pass_.setFilterFieldName ("z");
            pass_.setFilterLimits (z_min, z_max);

            // compute
            cloud_t::Ptr cloud_out(new cloud_t);
            pass_.setInputCloud (cloud_in);
            pass_.filter (*cloud_out);
            return cloud_out;
        }

        cloud_t::Ptr downsample(cloud_t::Ptr cloud_in, float leaf_size=0.015){
            // configure filter
            pcl::VoxelGrid<point_t> vox_; 
            vox_.setLeafSize(leaf_size,leaf_size,leaf_size);
            vox_.setDownsampleAllData (false); //TODO : ???

            // filter to output
            cloud_t::Ptr cloud_out(new cloud_t);
            vox_.setInputCloud(cloud_in);
            vox_.filter(*cloud_out);
            return cloud_out;
        }

        cloud_t::Ptr fit_plane(cloud_t::Ptr cloud_in,
                float dist_thresh=0.05,
                int max_iter=10000,
                float w_ndist = 0.1,
                bool opt_coeffs=true
                ){

            /* normals */
            // setup
            pcl::NormalEstimation<point_t, pcl::Normal> norm_;
            pcl::search::KdTree<point_t>::Ptr normals_tree_ = boost::make_shared<pcl::search::KdTree<point_t> > ();
            pcl::PointCloud<pcl::Normal>::Ptr cloud_norm(new pcl::PointCloud<pcl::Normal>); 
            norm_.setKSearch(10);  
            norm_.setSearchMethod(normals_tree_);

            // compute
            norm_.setInputCloud(cloud_in);
            norm_.compute(*cloud_norm);

            /* plane */
            // setup
            pcl::SACSegmentationFromNormals<point_t, pcl::Normal> seg_;
            seg_.setDistanceThreshold(dist_thresh); 
            seg_.setMaxIterations(max_iter);
            seg_.setNormalDistanceWeight(w_ndist);
            seg_.setOptimizeCoefficients(opt_coeffs);
            seg_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
            seg_.setMethodType(pcl::SAC_RANSAC);
            seg_.setProbability(0.99);

            // output
            pcl::PointIndices::Ptr table_idx(new pcl::PointIndices); 
            pcl::ModelCoefficients::Ptr table_coeffs(new pcl::ModelCoefficients); 

            // compute
            seg_.setInputCloud(cloud_in);
            seg_.setInputNormals(cloud_norm);
            seg_.segment (*table_idx, *table_coeffs);

            /* project */
            pcl::ProjectInliers<point_t> proj_;
            proj_.setModelType (pcl::SACMODEL_PLANE);

            cloud_t::Ptr cloud_table(new cloud_t); 
            proj_.setInputCloud (cloud_in);
            proj_.setIndices (table_idx);
            proj_.setModelCoefficients (table_coeffs);
            proj_.filter (*cloud_table);

            return cloud_table; //for debugging for now
        }

        void cluster(){
            // object segmentation step here
        }

        cloud_t::Ptr operator()(cloud_t::Ptr cloud_in){
            cloud_t::Ptr cloud_pass  = this->passthrough(cloud_in);
            ROS_INFO("Pass Point Size : %d", cloud_pass->points.size());
            cloud_t::Ptr cloud_down  = this->downsample(cloud_pass);
            ROS_INFO("Down Point Size : %d", cloud_down->points.size());
            cloud_t::Ptr cloud_table = this->fit_plane(cloud_down);
            ROS_INFO("Table Point Size : %d", cloud_table->points.size());

            return cloud_table;
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
        cloud_t::Ptr pcl_in_;
        ros::Time stamp;
        std::string frame_id;
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

        void publish(cloud_t::Ptr cloud){
            // publish output
            pcl_out_.reset(new sensor_msgs::PointCloud2);
            pcl::toROSMsg (*cloud, *pcl_out_);
            pcl_out_->header.seq += 1;
            pcl_out_->header.stamp = stamp;
            pcl_out_->header.frame_id = frame_id;
            pcl_pub_.publish(pcl_out_);
        }

        void pcl_cb(const sensor_msgs::PointCloud2ConstPtr& msg_in){
            pcl_in_.reset(new cloud_t);
            pcl::fromROSMsg(*msg_in, *pcl_in_);

            // save stamp+frame
            stamp = msg_in->header.stamp;
            frame_id = msg_in->header.frame_id;

            new_cloud_ = true;
            //ROS_INFO_THROTTLE(1.0, "NEW CLOUD!");
        }

        void step(){
            ros::Rate rate(50);
            if(new_cloud_){
                new_cloud_ = false;
                //ROS_INFO("%d\n", pcl_in_->points.size());
                this->publish( filter_(pcl_in_));
            }
            rate.sleep();
            ros::spinOnce();
        }

        void run(){
            while (ros::ok()){
                step();
            }
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_filter");
    ros::NodeHandle nh;
    ObjectFilterROS filter(nh);
    filter.run();

    return 0;
}
