#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pclomp/ndt_omp.h>


bool reflashFlag = false;
pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_downsample(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_downsample(new pcl::PointCloud<pcl::PointXYZ>());
Eigen::Matrix4f deltaT = Eigen::Isometry3f::Identity().matrix();
Eigen::Matrix4f ndtResult = Eigen::Isometry3f::Identity().matrix();
int process(int argc, char** argv) {
  if(argc != 3) {
    std::cout << "usage: switchPointCloud target.pcd source.pcd" << std::endl;
    return 0;
  }

  ros::init(argc, argv, "switchPointCloud");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string target_pcd = argv[1];
  std::string source_pcd = argv[2];



  if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
    std::cerr << "failed to load " << target_pcd << std::endl;
    return 0;
  }
  if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
    std::cerr << "failed to load " << source_pcd << std::endl;
    return 0;
  }

  // downsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

  voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*downsampled);
  *target_cloud_downsample = *downsampled;

  voxelgrid.setInputCloud(source_cloud);
  voxelgrid.filter(*downsampled);
  *source_cloud_downsample = *downsampled;

  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt_omp->setResolution(1.0);
  ndt_omp->setNumThreads(omp_get_max_threads());
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);

  ndt_omp->setInputTarget(target_cloud_downsample);
  ndt_omp->setInputSource(source_cloud_downsample);

  ndt_omp->align(*aligned);
  reflashFlag = true;

  {
  pcl::PointCloud<pcl::PointXYZ> trans_temp;
  deltaT = ndt_omp->getFinalTransformation();
  cout << "score:" << ndt_omp->getFitnessScore() << " iterNum:" << ndt_omp->getFinalNumIteration() <<endl;
  pcl::transformPointCloud (*source_cloud, trans_temp, deltaT);
  pcl::PointCloud<pcl::PointXYZ> result  = *target_cloud + trans_temp;
  pcl::io::savePCDFile("switched.pcd", result);
  }

  while(ros::ok())
  {
      cout <<" 红色target 绿色source 要保证蓝色的点和红色的点匹配在一起。如果结果无误，在图形界面按q即可退出并保存。 输入相对于target坐标的变换。 \r\n (x(m), y(m) ,z(m), thx, thy thz(degree)):" <<endl;
      float dx=0, dy=0, dz=0, dthx=0, dthy=0, dthz=0;
      while(!(cin >> dx >> dy >> dz >> dthx >> dthy >> dthz))
         {
           cin.clear();
           cin.ignore();
           cin.sync();
           cout << "输入错误"<<endl;
         }
      //2d 转 3d
      Eigen::AngleAxisf rx(dthx/180*3.14, Eigen::Vector3f(1,0,0));
      Eigen::AngleAxisf ry(dthy/180*3.14, Eigen::Vector3f(0,1,0));
      Eigen::AngleAxisf rz(dthz/180*3.14, Eigen::Vector3f(0,0,1));


      deltaT.block<3,3>(0,0) = (rx*ry*rz).matrix();

      deltaT(0,3) = dx;
      deltaT(1,3) = dy;
      deltaT(2,3) = dz;

      ndt_omp->align(*aligned, deltaT);
      deltaT = ndt_omp->getFinalTransformation();
      cout << "score:" << ndt_omp->getFitnessScore() << " iterNum:" << ndt_omp->getFinalNumIteration() <<endl;
      ndtResult  = ndt_omp->getFinalTransformation();
      cout << ndtResult << endl;

      {
      pcl::PointCloud<pcl::PointXYZ> trans_temp;
      pcl::transformPointCloud (*source_cloud, trans_temp, deltaT);
      pcl::PointCloud<pcl::PointXYZ> result  = *target_cloud + trans_temp;
      pcl::io::savePCDFile("switched.pcd", result);
      }

      reflashFlag = true;

  }

  return 0;
}

void gui()
{
    pcl::visualization::PCLVisualizer vis("vis");
    vis.setBackgroundColor (0, 0, 0);
    //vis.addCoordinateSystem (4.0, "target");
    Eigen::Transform<float, 3, Eigen::Affine> globalPose (ndtResult);

    vis.addCoordinateSystem (2.0, globalPose, "source" );

    vis.initCameraParameters ();

    while(true)
    {
        if(reflashFlag)
        {
            reflashFlag = false;
            // visulization

            vis.removeAllPointClouds();
            pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_trans(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud (*source_cloud, *source_cloud_trans, deltaT);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(target_cloud, 255.0, 0.0, 0.0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(source_cloud_trans, 0.0, 255.0, 0.0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_handler(aligned, 0.0, 0.0, 255.0);
            vis.addPointCloud(target_cloud, target_handler, "target");
            //vis.addPointCloud(source_cloud_trans, source_handler, "source");
            vis.addPointCloud(aligned, aligned_handler, "aligned");


        }
        vis.spinOnce(100);
    }

}

int main(int argc, char** argv)
{
  boost::thread thrd1(&process, argc, argv);
  boost::thread thrd2(&gui);
  thrd1.join();
  thrd2.join();
  return 0;
}

