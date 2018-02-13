#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/integral_image_normal.h>


using namespace pcl;
using namespace std;

int
  main (int argc, char** argv)
{
   PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
   if(io::loadPLYFile<PointXYZ> ("All_Points.ply", *cloud) == -1){
      cout << "fail" << endl;

   } else {

      cout << "loaded" << endl;

      cout << "begin passthrough filter" << endl;
      PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
      PassThrough<PointXYZ> filter;
      filter.setInputCloud(cloud);
      filter.filter(*filtered);
      cout << "passthrough filter complete" << endl;

      // cout << "begin moving least squares" << endl;
      // MovingLeastSquares<PointXYZ, PointXYZ> mls;
      // mls.setInputCloud(filtered);
      // mls.setSearchRadius(0.01);
      // mls.setPolynomialFit(true);
      // mls.setPolynomialOrder(2);
      // mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
      // mls.setUpsamplingRadius(0.005);
      // mls.setUpsamplingStepSize(0.003);

      // PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ>());
      // mls.process(*cloud_smoothed);
      // cout << "MLS complete" << endl;

	  cout << "begin normal estimation" << endl;
	  pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());
	  NormalEstimationOMP<PointXYZ, Normal> ne;
	  ne.setSearchMethod(tree);
	  ne.setNumberOfThreads(8);
	  ne.setInputCloud(filtered);
	  ne.setKSearch(100);
	  // compute the centroid of pointcloud
	  Eigen::Vector4f centroid;
	  compute3DCentroid(*filtered, centroid);
	  //ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
	  // compute normals
	  PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
	  ne.compute(*cloud_normals);
	  cout << "normal estimation complete" << endl;
	  // reverse normals
	  cout << "reverse normals' direction" << endl;
	  for (size_t i = 0; i < cloud_normals->size(); ++i) {
		  cloud_normals->points[i].normal_x *= -1;
		  cloud_normals->points[i].normal_y *= -1;
		  cloud_normals->points[i].normal_z *= -1;
	  }



      cout << "combine points and normals" << endl;
      PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
      concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);
	  io::savePLYFile("pointNormals.ply", *cloud_smoothed_normals);

      cout << "begin poisson reconstruction" << endl;
      Poisson<PointNormal> poisson;
      poisson.setDepth(9);
      poisson.setInputCloud(cloud_smoothed_normals);
	  poisson.setPointWeight(0);
	  poisson.setSamplesPerNode(1);

      PolygonMesh mesh;
      poisson.reconstruct(mesh);

	  cout << "writing.....";

      io::savePLYFile("mesh.ply", mesh);

	  cout << "done babe!";
	  system("pause");

   }
  return (0);
}