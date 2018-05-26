
#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_labeled_clusters.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>

#define NUM_PARTS   25
#define AREA_THRES  350

using namespace std;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZL>::ConstPtr cloud)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZL> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr add_cloud(new pcl::PointCloud<pcl::PointXYZL>);
  // Read in the cloud data with label
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZL>);
  reader.read ("888_out.pcd", *cloud_in);
  std::cout << "PointCloud  has: " << cloud_in->points.size () << " data points." << std::endl; //*

 // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZL>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZL>);
  //pcl::search::OrganizedNeighbor<pcl::PointXYZL>::Ptr tree (new pcl::search::OrganizedNeighbor<pcl::PointXYZL>);

  std::vector<std::vector<pcl::PointIndices> > cluster_indices;

  cluster_indices.resize(10);  //NUM_PARTS==25
  pcl::LabeledEuclideanClusterExtraction<pcl::PointXYZL> ec;
  // ec.setClusterTolerance (0.5); // 30cm
  //ec.setClusterTolerance (3); // 3pixels

  //delete these magic numbers to something learned
  ec.setMinClusterSize (1);  //AREA_THRES =350
  ec.setMaxClusterSize (8888);
  ec.setSearchMethod (tree);
  ec.setInputCloud( cloud_in);
  ec.setMaxLabels(10);
  ec.extract (cluster_indices);

  std::cout<<cluster_indices.size()<<std::endl;

  int num=0;
  for(unsigned int k = 0; k < cluster_indices.size(); k++)
  {
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZL>);
    std::cout << "For label " << k << " Found " << cluster_indices[k].size() << std::endl;

   std::vector<pcl::PointIndices> indices;
   indices = cluster_indices[k];

  pcl::PCDWriter writer;

  for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it)
  { 
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
       {
        cloud_cluster->points.push_back (cloud_in->points[*pit]); 
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
       }     
  }
       std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
       if(cloud_cluster->points.size () !=0)
       {
       std::stringstream ss;
       ss << "cloud_cluster_" << k << ".pcd";
       writer.write<pcl::PointXYZL> (ss.str (), *cloud_cluster, false);
   
  *add_cloud+=*cloud_cluster;
  pcl::io::savePCDFileASCII("add_cloud.pcd",*add_cloud);
   // num+=cluster_indices[k].size();
   
    }
}
 // std::cout<<"the label 0-k number is :"<<num<<std::endl;
    viewer = simpleVis(add_cloud);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;
}








