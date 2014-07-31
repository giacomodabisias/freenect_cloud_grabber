

#include "freenect_grabber.hpp"


const int distance = 7000;



int main( int argc, char** argv )
{


  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;

  freenectGrabber<pcl::PointXYZRGB> c;

  cloud = c.get_point_cloud(distance, true);


  cloud->sensor_orientation_.w () = 0.0;
  cloud->sensor_orientation_.x () = 1.0;
  cloud->sensor_orientation_.y () = 0.0;
  cloud->sensor_orientation_.z () = 0.0;


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");


  while (!viewer->wasStopped ()) {
    viewer->spinOnce ();
    cloud = c.get_point_cloud(distance, true);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud"); 

  }
  cloud->height = 1;
  cloud->width = cloud->points.size();

  return 0;
}
