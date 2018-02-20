#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int
main (int argc, char** argv)
{
  // Fill in the Cloud_Root & Cloud Extra data
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PLYReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointNormal>("test.ply", *cloud);
  std::cout << "Point cloud loaded" << std::endl;

  // Create a KD-Tree
  std::cout << "Generating KD Tree" << std::endl;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  std::cout << "Start MLS Smoothing" << std::endl;
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
 
  mls.setComputeNormals (false);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.05);

  // Reconstruct
  mls.process (mls_points);

  // Save output
  std::cout << "Writing outputs" << std::endl;
  pcl::io::savePLYFile ("test-mls.ply", mls_points);

  system("pause");
}
