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

using namespace std;

double g_scaleValue = 3.0;
double g_ndtResolution = 0.25;
int g_pointCloudSaveCount=0;
double lamda_p = 0.7;
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


int main(int argc, char** argv) {


  ros::init(argc, argv, "autoSelectPlaneWithNdt");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");

  string filename, outputDir;

  nh_param.param<string>("fileName", filename, "/home/xjh/catkin_buildMap/src/ndt_omp/data/251370668.pcd");
  nh_param.param<string>("outputDir", outputDir, "/home/xjh/catkin_buildMap/src/ndt_omp/data/");
  nh_param.param<double>("ndtResolution", g_ndtResolution, 0.5);
  nh_param.param<double>("scaleValue", g_scaleValue, 3.0);
  nh_param.param<double>("lamda_p", lamda_p, 0.7);


  mkdir(outputDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);



  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_origin(new pcl::PointCloud<pcl::PointXYZRGB>());

  if(pcl::io::loadPCDFile(filename, *target_cloud_origin) || pcl::io::loadPCDFile(filename, *target_cloud)) {
    std::cerr << "failed to load " << filename << std::endl;
    return 0;
  }

//  for(int i = 0; i < target_cloud_origin->size(); i++ )
//  {
//      cout<<"origin"<<target_cloud_origin->points[i]<<endl;
//
//  }

  ros::Time::init();

  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());

  //设置NDT网格结构的分辨率（VoxelGridCovariance）
  ndt_omp->setResolution(g_ndtResolution);

  ndt_omp->setNumThreads(8);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt_omp->setInputTarget(target_cloud);

  Eigen::Vector3d cell_mean;
  std::vector<visualization_msgs::Marker > Markers;
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

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "my_namespace";
      marker.id = idd++;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = cell_mean(0);
      marker.pose.position.y = cell_mean(1);
      marker.pose.position.z = cell_mean(2);

      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();

      //归一化，方便绘制
      marker.scale.x = evals_reset(0)/evals_reset.sum()*g_ndtResolution*2;
      marker.scale.y = evals_reset(1)/evals_reset.sum()*g_ndtResolution*2;
      marker.scale.z = evals_reset(2)/evals_reset.sum()*g_ndtResolution*2;

      marker.color.a = 0.6;         // Don't forget to set the alpha!
      if(temp == 0)
      {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      }
      else if (temp == 1)
      {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      }
      else
      {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
      }

      if(temp != 5)
      {

        std::stringstream ss_pcd;
        ss_pcd << g_pointCloudSaveCount;
        std::string s_pcd = outputDir + "surface" + ss_pcd.str() + ".pcd";
        pcl::io::savePCDFileBinary(s_pcd, leaf.pointList_);
        g_pointCloudSaveCount++;
        Markers.push_back(marker);
      }



    }
  }

  //cout << (ndt_omp->getTargetCells()).getLeafSize().norm () <<endl;

  ros::Publisher pubMapCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map", 1);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );

  ros::Rate rate(1);

  while (ros::ok())
  {
    sensor_msgs::PointCloud2 PonitMap2;
    pcl::toROSMsg(*target_cloud_origin, PonitMap2);
    //PonitMap2.header.stamp = laserCloudMsg->header.stamp;
    PonitMap2.header.frame_id = "/map";
    pubMapCloud.publish(PonitMap2);

    vis_pub.publish(Markers);
    rate.sleep();
  }

  return 0;
}

