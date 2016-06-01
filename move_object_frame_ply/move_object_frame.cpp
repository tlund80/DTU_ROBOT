#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/pca.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.ply output.ply  <options>\n", argv[0]);
 // print_info ("  where options are:\n");
}

bool moveObjectFrame(PointCloudNT::Ptr &src_cloud, PointCloudNT::Ptr &tar_cloud){
  
	PointCloudNT::Ptr source (new PointCloudNT ());
	//fromPCLPointCloud2 (*src_cloud, *source);
	*source += *src_cloud;
 	
	Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*source,centroid);
	
	//  pcl::transformPointCloud(*src_cloud,*temp,mat);
		 
	pcl::PCA<PointNT> _pca; 
	PointNT projected; 
	PointNT reconstructed;
	PointCloudNT cloudi = *source;
	PointCloudNT finalCloud;
		 
	try{
	     //Do PCA for each point to preserve color information
	     //Add point cloud to force PCL to init_compute else a exception is thrown!!!HACK
	     _pca.setInputCloud(source);
	     int i;
	 //    #pragma omp parallel for
	     for(i = 0; i < (int)source->size(); i++)     {
	       _pca.project(cloudi[i],projected);
	       Eigen::Matrix3f eigen = _pca.getEigenVectors();
	      
	       cloudi[i].getNormalVector3fMap() = _pca.getEigenVectors().transpose() *  cloudi[i].getNormalVector3fMap();
	       // flip axis to satisfy right-handed system
              if (eigen.col(0).cross(eigen.col(1)).dot(eigen.col(2)) < 0) {
		        projected.z = -projected.z; //Avoid flipping the model
		        
		      
			projected.normal_x = cloudi[i].normal_x;
			projected.normal_y = cloudi[i].normal_y;
			projected.normal_z = -cloudi[i].normal_z;
			
			//projected.normal_x = cloudi[i].normal_x;
			//projected.normal_y = cloudi[i].normal_y;
			//projected.normal_z = -cloudi[i].normal_z;
			projected.curvature = cloudi[i].curvature;
              }else{
			projected.normal_x = cloudi[i].normal_x;
			projected.normal_y = cloudi[i].normal_y;
			projected.normal_z = cloudi[i].normal_z;
			
		
	      }
      
       
	//       if(pcl::getFieldIndex(*src_cloud,"rgba") >= 0){
		 //assign colors
		 projected.r = cloudi[i].r;
		 projected.g = cloudi[i].g;
		 projected.b = cloudi[i].b;
		 
		
			//projected.curvature = cloudi[i].curvature;
	  //     }
		 
	       //add point to cloud
	       finalCloud.push_back(projected);
	       
	    }
	    
	}catch(pcl::InitFailedException &e){
	  PCL_ERROR(e.what());
	  
	}

	//*tar
	// toPCLPointCloud2 (finalCloud, *tar_cloud);
	pcl::copyPointCloud(finalCloud,*tar_cloud);
}

bool
loadCloud_ply (const std::string &filename, PointCloudNT &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (pcl::io::loadPLYFile(filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" seconds : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" seconds : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  pcl::io::savePCDFile (filename, output);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" seconds : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

void
saveCloud_ply (const std::string &filename, const PointCloudNT &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  pcl::io::savePLYFile (filename, output);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" seconds : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

int
main (int argc, char** argv)
{

  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
 // p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  p_file_indices = parse_file_extension_argument (argc, argv, ".ply");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and one output PCD file to continue.\n");
    return (-1);
  }

  // Command line parsing

  // Load the first file
 // pcl::PCLPointCloud2::Ptr cloud_source (new pcl::PCLPointCloud2 ());
   PointCloudNT::Ptr cloud_source (new PointCloudNT());
  if (!loadCloud_ply (argv[p_file_indices[0]], *cloud_source))
    return (-1);
  
  PointCloudNT::Ptr cloud_target (new PointCloudNT ()); 
// pcl::PCLPointCloud2::Ptr cloud_target (new pcl::PCLPointCloud2 ());
  moveObjectFrame(cloud_source, cloud_target);
 // saveCloud(argv[p_file_indices[1]], *cloud_target);

  saveCloud_ply(argv[p_file_indices[1]], *cloud_target);
  
return 0;
}

