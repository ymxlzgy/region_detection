#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>

using namespace std;

double g_scaleValue = 3.0;
double g_ndtResolution = 0.25;
int g_pointCloudSaveCount=0;
double lamda_p = 0.7;

ros::Publisher vis_plane_pub, vis_other_pub, vis_region_pub;

//octomap::ColorOcTree* colortree = new octomap::ColorOcTree(0.03);

pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());

/**
 * @brief checkType_2 论文的方式检测
 * @param eigenVector
 * @param evecs
 * @return 5是非平面 0 1 2分别是三个主轴
 */
int checkType_2(Eigen::Vector3d eigenVector, Eigen::Matrix3d evecs)
{
    double value0 = g_scaleValue;

    double value0_1 = eigenVector[0]/eigenVector[1];
    double value0_2 = eigenVector[0]/eigenVector[2];
    double value1_0 = eigenVector[1]/eigenVector[0];
    double value1_2 = eigenVector[1]/eigenVector[2];
    double value2_0 = eigenVector[2]/eigenVector[0];
    double value2_1 = eigenVector[2]/eigenVector[1];

    std::vector<double> lamdas;//从小到大排列
    for( int i=0; i<3; i++)
        lamdas.push_back(eigenVector[i]);
    std::sort(lamdas.begin(), lamdas.end());

    double p = 2*(lamdas[1] - lamdas[0])/(lamdas[0] + lamdas[1] + lamdas[2]);

    if(p < lamda_p)
    {
        return 5;
    }

    int min_idx = 0;
    for( int i=0; i<3; i++)
        if(lamdas[0] == eigenVector[i]){
            min_idx = i;
        }

    Eigen::Vector3d eigen_vector = evecs.block<3,1>(0,min_idx);
    if(fabs(eigen_vector(0)) > fabs(eigen_vector(1)) && fabs(eigen_vector(0)) > fabs(eigen_vector(2)))
        return 0;
    if(fabs(eigen_vector(1)) > fabs(eigen_vector(0)) && fabs(eigen_vector(1)) > fabs(eigen_vector(2)))
        return 1;
    return 2;
}

/**
 * @brief checkType
 * 检测当前cell的种类
 * @param eigenVector
 * @return
 * 1 1大2小 针状
 * 2 2大1小 饼状
 * 3 一般
 */
int checkType(Eigen::Vector3d eigenVector)
{
    double value0 = g_scaleValue;

    double value0_1 = eigenVector[0]/eigenVector[1];
    double value0_2 = eigenVector[0]/eigenVector[2];
    double value1_0 = eigenVector[1]/eigenVector[0];
    double value1_2 = eigenVector[1]/eigenVector[2];
    double value2_0 = eigenVector[2]/eigenVector[0];
    double value2_1 = eigenVector[2]/eigenVector[1];

    ////1大2小
    if(value0_1 > value0 && value0_2 > value0)
        return 1;
    if(value1_0 > value0 && value1_2 > value0)
        return 1;
    if(value2_0 > value0 && value2_1 > value0)
        return 1;

    //2大1小
    if(value1_0 > value0 && value2_0 > value0)
        return 2;
    if(value0_1 > value0 && value2_1 > value0)
        return 2;
    if(value0_2 > value0 && value1_2 > value0)
        return 2;

    return 3;
}

void PlaneHandler(const sensor_msgs::PointCloud2ConstPtr &localmap)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_origin(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr region(new pcl::PointCloud<pcl::PointXYZRGB>());
    sensor_msgs::PointCloud2 region_ros;
    pcl::fromROSMsg(*localmap, *target_cloud);
    pcl::fromROSMsg(*localmap, *target_cloud_origin);

//    for(auto p:target_cloud_origin->points)
//    {
//        colortree->updateNode(octomap::point3d(p.x, p.y, p.z), true);
//    }
//
//    for(auto p:target_cloud_origin->points)
//    {
//        colortree->integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
//    }

//    colortree->updateInnerOccupancy();

//    octomap_msgs::Octomap map_msg;
//    map_msg.header.frame_id = "camera_link";
//    map_msg.header.stamp = ros::Time::now();
//    if (octomap_msgs::fullMapToMsg(*colortree, map_msg))
//        pub_octomap.publish(map_msg);

    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt_omp->setResolution(g_ndtResolution);

    ndt_omp->setNumThreads(8);
    ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);

    if(target_cloud->points.size() > 5000)
    {
        ndt_omp->setInputTarget(target_cloud);

        Eigen::Vector3d cell_mean;
        std::vector<visualization_msgs::Marker > planeMarkers;
        std::vector<visualization_msgs::Marker > otherMarkers;

        int idd=0;


        for (auto it = ndt_omp->getTargetCells().getLeaves().begin (); it != ndt_omp->getTargetCells().getLeaves().end (); ++it)
        {
            //pclomp::VoxelGridCovariance::Leaf& leaf = it->second;
            auto leaf = it->second;

            if (leaf.nr_points >= 10)
            {
                cell_mean = leaf.mean_;

                Eigen::Matrix3d evecs_;
                Eigen::Vector3d evals_;
                evecs_ = leaf.getEvecs();
                evals_ = leaf.getEvals();

                Eigen::Matrix3d evecs_reset;
                Eigen::Vector3d evals_reset = evals_;

                Eigen::Vector3d tempp = evecs_.block<3,1>(0,0).cross(evecs_.block<3,1>(0,1));
                evecs_reset.block<3,2>(0,0) = evecs_.block<3,2>(0,0);
                evecs_reset.block<3,1>(0,2) = tempp;

                Eigen::Quaterniond q(evecs_reset);

                //cout << "evecs_\n" <<evecs_ <<"\n evals_ \n" <<evals_ <<endl ;

                //int temp = checkType(evals_reset);
                int temp = checkType_2(evals_reset, evecs_reset);

                visualization_msgs::Marker planemarker;
                planemarker.header.frame_id = "camera_link";
                planemarker.header.stamp = ros::Time();
                planemarker.ns = "my_namespace";
                planemarker.id = idd++;
                planemarker.type = visualization_msgs::Marker::SPHERE;
                planemarker.action = visualization_msgs::Marker::ADD;
                planemarker.pose.position.x = cell_mean(0);
                planemarker.pose.position.y = cell_mean(1);
                planemarker.pose.position.z = cell_mean(2);

                planemarker.pose.orientation.x = q.x();
                planemarker.pose.orientation.y = q.y();
                planemarker.pose.orientation.z = q.z();
                planemarker.pose.orientation.w = q.w();

                //归一化，方便绘制
                planemarker.scale.x = evals_reset(0)/evals_reset.sum()*g_ndtResolution*2;
                planemarker.scale.y = evals_reset(1)/evals_reset.sum()*g_ndtResolution*2;
                planemarker.scale.z = evals_reset(2)/evals_reset.sum()*g_ndtResolution*2;

                planemarker.color.a = 0.6;         // Don't forget to set the alpha!

                visualization_msgs::Marker othermarker;
                othermarker.header.frame_id = "camera_link";
                othermarker.header.stamp = ros::Time();
                othermarker.ns = "my_namespace";
                othermarker.id = idd++;
                othermarker.type = visualization_msgs::Marker::CUBE;
                othermarker.action = visualization_msgs::Marker::ADD;
                othermarker.pose.position.x = cell_mean(0);
                othermarker.pose.position.y = cell_mean(1);
                othermarker.pose.position.z = cell_mean(2);

                othermarker.pose.orientation.x = q.x();
                othermarker.pose.orientation.y = q.y();
                othermarker.pose.orientation.z = q.z();
                othermarker.pose.orientation.w = q.w();

                //归一化，方便绘制
                othermarker.scale.x = evals_reset(0)/evals_reset.sum()*g_ndtResolution*2;
                othermarker.scale.y = evals_reset(1)/evals_reset.sum()*g_ndtResolution*2;
                othermarker.scale.z = evals_reset(2)/evals_reset.sum()*g_ndtResolution*2;

                othermarker.color.a = 0.6;         // Don't forget to set the alpha!
                if(temp == 0)
                {
                    planemarker.color.r = 0.5;
                    planemarker.color.g = 0.0;
                    planemarker.color.b = 0.0;
                    for(auto p:leaf.pointList_.points)
                    {
                        pcl::PointXYZRGB p_color;
                        p_color.x = p.x;
                        p_color.y = p.y;
                        p_color.z = p.z;
                        p_color.b = 0.0;
                        p_color.g = 0.0;
                        p_color.r = 255;
                        p_color.a = 0.5;
                        region->points.push_back(p_color);
                    }
                }
                else if (temp == 1)
                {
                    planemarker.color.r = 0.0;
                    planemarker.color.g = 1.0;
                    planemarker.color.b = 0.0;

                    for(auto p:leaf.pointList_.points)
                    {
                        pcl::PointXYZRGB p_color;
                        p_color.x = p.x;
                        p_color.y = p.y;
                        p_color.z = p.z;
                        p_color.b = 0.0;
                        p_color.g = 255;
                        p_color.r = 0.0;
                        p_color.a = 0.5;
                        region->points.push_back(p_color);
                    }

                }
                else if (temp == 2)
                {
                    planemarker.color.r = 0.0;
                    planemarker.color.g = 0.0;
                    planemarker.color.b = 1.0;
                    for(auto p:leaf.pointList_.points)
                    {
                        pcl::PointXYZRGB p_color;
                        p_color.x = p.x;
                        p_color.y = p.y;
                        p_color.z = p.z;
                        p_color.b = 0.0;
                        p_color.g = 0.0;
                        p_color.r = 255;
                        p_color.a = 0.5;
                        region->points.push_back(p_color);
                    }
                }
                else
                {
                    othermarker.color.r = 1.0;
                    othermarker.color.g = 0.0;
                    othermarker.color.b = 0.0;
                    for(auto p:leaf.pointList_.points)
                    {
                        pcl::PointXYZRGB p_color;
                        p_color.x = p.x;
                        p_color.y = p.y;
                        p_color.z = p.z;
                        p_color.b = 0.0;
                        p_color.g = 0.0;
                        p_color.r = 255;
                        p_color.a = 0.5;
                        region->points.push_back(p_color);
                    }
                }
                if(temp != 5)
                {
                    planeMarkers.push_back(planemarker);
                }
                else
                {
                    otherMarkers.push_back(othermarker);
                }

            }
        }
        pcl::toROSMsg(*region, region_ros);
        region_ros.header.frame_id = "camera_link";
        cout<<"plane extract done"<<endl;
        vis_plane_pub.publish(planeMarkers);
        vis_other_pub.publish(otherMarkers);
        vis_region_pub.publish(region_ros);
    }


}


int main(int argc, char** argv) {


    ros::init(argc, argv, "region_detection");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");

    nh_param.param<double>("ndtResolution", g_ndtResolution, 0.5);
    nh_param.param<double>("scaleValue", g_scaleValue, 3.0);
    nh_param.param<double>("lamda_p", lamda_p, 0.7);

    ros::Subscriber subLocalMap =nh.subscribe<sensor_msgs::PointCloud2>("/localmap", 100, PlaneHandler);
    vis_plane_pub = nh.advertise<visualization_msgs::MarkerArray>( "/plane_marker", 0 );
    vis_other_pub = nh.advertise<visualization_msgs::MarkerArray>( "/other_marker", 0 );
    vis_region_pub = nh.advertise<sensor_msgs::PointCloud2>( "/region", 0 );

//    pub_octomap = nh.advertise<octomap_msgs::Octomap>("/local_octomap", 100, true);

    ros::Rate rate(1);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

