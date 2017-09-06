#include <iostream>
#include <map>

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Geometry>

#include <vtkVersion.h>
#include <vtkTriangleStrip.h>
#include <vtkRenderWindow.h>
#include <vtkColor.h>

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//convenient structure to handle our pointclouds
struct PCD {
	PointCloud::Ptr cloud;
	std::string f_name;

	PCD() : cloud(new PointCloud) {};
};

pcl::visualization::PCLVisualizer *viewer;

template<typename PointType>
void getAABB(pcl::PointCloud<PointType>& cloud, PointType& minP, PointType& maxP) {
	minP = maxP = cloud[0];
	for (int i = 1; i < cloud.size(); i++) {
		minP.x = std::min(cloud[i].x, minP.x);
		minP.y = std::min(cloud[i].y, minP.y);
		minP.z = std::min(cloud[i].z, minP.z);
		maxP.x = std::max(cloud[i].x, maxP.x);
		maxP.y = std::max(cloud[i].y, maxP.y);
		maxP.z = std::max(cloud[i].z, maxP.z);
	}
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
* \param argc the number of arguments (pass from main ())
* \param argv the actual command line arguments (pass from main ())
* \param models the resultant vector of point cloud datasets
*/
bool loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
	std::string extension(".pcd");
	// Suppose the first argument is the actual test model
	for (int i = 1; i < argc; i++)
	{
		std::string fname = std::string(argv[i]);
		// Needs to be at least 5: .plot
		if (fname.size() <= extension.size())
			continue;

		// turn all letters to lower case
		std::transform(fname.begin(), fname.end(), fname.begin(), (int(*)(int))tolower);

		//check that the argument is a pcd file
		if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
		{
			// Load the cloud and saves it into the global list of models
			PCD m;
			m.f_name = argv[i]; 
			if (pcl::io::loadPCDFile(argv[i], *m.cloud) < 0) return false;
			//remove NAN points from the cloud
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

			models.push_back(m);
		}
	}
	return true;
}

inline int zToColor(float z) {
	float fColor = (z - 15) / 4.f; // [15,18]->[0,1]
	int color = fColor * 180 + 40; // [0,1]->[40,220]
	return color;
}

int main(int argc, char** argv) {
	pcl::console::TicToc time;

	// load cloud
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
	time.tic();
	PCL_INFO("Start load...");
	if (!loadData(argc, argv, data)) {
		PCL_ERROR("Cannot find this file!");
		return -1;
	}
	// Check user input
	if (data.empty()) {
		//PCL_ERROR("[*] - multiple files can be added. ");
		// no input, try default name
		PCD m;
		m.f_name = "cloud";
		if (pcl::io::loadPCDFile("out.pcd", *m.cloud) < 0) {
			PCL_ERROR("Cannot find this file!");
			system("pause");
			return -1;
		}
		data.push_back(m);
	}
	PCL_INFO("Loaded %d datasets in %lf seconds.\n", (int)data.size(),time.toc()/1000.0);

	// Create point cloud
	PointCloud::Ptr cloud;
	for (size_t i = 0; i < data.size(); ++i) {
		cloud = data[i].cloud;
		// change color
		for (size_t j = 0; j < cloud->size(); j++) {
			int color = zToColor(cloud->points[j].z);
			uint32_t rgb = color << 16 | color << 8 | color;
			cloud->points[j].rgb = *reinterpret_cast<float*>(&rgb);
		}
	}

	PointT minP, maxP;
	time.tic();
	getAABB(*cloud, minP, maxP);
	PCL_INFO("%f,%f,%f,%f,%f,%f time:%f seconds.\n",
		minP.x, maxP.x, minP.y, maxP.y, minP.z, maxP.z, time.toc() / 1000.0);

	// move data to origin
	Eigen::Affine3f a;
	a = Eigen::Translation3f(-(minP.x + maxP.x) / 2, -(minP.y + maxP.y) / 2, -minP.z);
	pcl::transformPointCloud(*cloud, *cloud, a);
	minP = pcl::transformPoint(minP, a);
	maxP = pcl::transformPoint(maxP, a);
	PCL_INFO("%f,%f,%f,%f,%f,%f ", minP.x, maxP.x, minP.y, maxP.y, minP.z, maxP.z);

	// load model
	time.tic();
	pcl::PolygonMesh polygonMesh;
	pcl::io::loadPolygonFileSTL("cp2017.STL", polygonMesh);
	std::cout << time.toc() / 1000.0 << 's' << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZ>);
	fromPCLPointCloud2(polygonMesh.cloud, *meshCloud);

	pcl::PointXYZ minP2, maxP2;
	time.tic();
	getAABB(*meshCloud, minP2, maxP2);
	
	a = Eigen::AngleAxisf(180 * static_cast<float>(M_PI) / 180, Eigen::Vector3f::UnitY())*
		Eigen::Translation3f(-(minP2.x + maxP2.x) / 2, -(minP2.y + maxP2.y) / 2, -maxP2.z);
	pcl::transformPointCloud(*meshCloud, *meshCloud, a);
	toPCLPointCloud2(*meshCloud, polygonMesh.cloud);
	getAABB(*meshCloud, minP2, maxP2);
	PCL_INFO("%f,%f,%f,%f,%f,%f time:%f seconds.\n",
		minP2.x, maxP2.x, minP2.y, maxP2.y, minP2.z, maxP2.z, time.toc() / 1000.0);

	// load sampled model cloud
	time.tic();
	PointCloudWithNormals::Ptr meshSampCloud(new PointCloudWithNormals);
	if (pcl::io::loadPCDFile("model_cloud.pcl", *meshSampCloud) < 0) {
		PCL_ERROR("Cannot find this file!");
		system("pause");
		return -1;
	}
	PCL_INFO("Loaded model cloud in %f seconds.\n", time.toc() / 1000.0);

	//////////////////////////////////////////////////////////////////////////
	// try to calculate distance
	//create an index map
/*
	time.tic();
	std::map<uint32_t, size_t> posOfIndices;
	for (size_t i = 0; i < polygonMesh.polygons.size(); i++) {
		std::vector<uint32_t> vertices = polygonMesh.polygons[i].vertices;
		for (size_t j = 0; j < vertices.size(); j++) {
			posOfIndices.insert(std::pair<uint32_t, size_t>(vertices[j], i));
		}
	}
	PCL_INFO("Map in: %f seconds.\n", time.toc() / 1000.0);*/

/*
	time.tic();
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	kdTree.setInputCloud(meshCloud);
	pcl::PointXYZ searchPoint;
	int K = 3;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	std::vector<uint32_t> triFound;
	Eigen::Vector3f p0(0, 0, 0), p1(0, 0, 0), p2(0, 0, 0), p3(0, 0, 0);

	for (size_t i = 0; i < cloud->size(); i++) {
		searchPoint.x = cloud->points[i].x;
		searchPoint.y = cloud->points[i].y;
		searchPoint.z = cloud->points[i].z;
		if (kdTree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
			p0 << searchPoint.x, searchPoint.y, searchPoint.z;

			triFound = polygonMesh.polygons[posOfIndices[pointIdxNKNSearch[0]]].vertices;
			
			p1 << meshCloud->points[triFound[0]].x,
				meshCloud->points[triFound[0]].y,
				meshCloud->points[triFound[0]].z;
			p2 << meshCloud->points[triFound[1]].x,
				meshCloud->points[triFound[1]].y,
				meshCloud->points[triFound[1]].z;
			p3 << meshCloud->points[triFound[2]].x,
				meshCloud->points[triFound[2]].y,
				meshCloud->points[triFound[2]].z;
			//PCL_INFO("(%f,%f,%f),(%f,%f,%f),(%f,%f,%f) ", 
			//	p1(0), p1(1), p1(2), p2(0), p2(1), p2(2), p3(0), p3(1), p3(2));
			//PCL_INFO("(%f,%f,%f),(%f,%f,%f),(%f,%f,%f) ",
			//	meshCloud->points[triFound[0]].x, meshCloud->points[triFound[0]].y,	meshCloud->points[triFound[0]].z, 
			//	meshCloud->points[triFound[1]].x, meshCloud->points[triFound[1]].y, meshCloud->points[triFound[1]].z,
			//	meshCloud->points[triFound[2]].x, meshCloud->points[triFound[2]].y, meshCloud->points[triFound[2]].z);

/ *
			// Cannot find any index, it should not happen
			PCL_ERROR("cannot find this index: %d\n", pointIdxNKNSearch[0]);
			p1 << meshCloud->points[pointIdxNKNSearch[0]].x,
				meshCloud->points[pointIdxNKNSearch[0]].y,
				meshCloud->points[pointIdxNKNSearch[0]].z;
			p2 << meshCloud->points[pointIdxNKNSearch[1]].x,
				meshCloud->points[pointIdxNKNSearch[1]].y,
				meshCloud->points[pointIdxNKNSearch[1]].z;
			p3 << meshCloud->points[pointIdxNKNSearch[2]].x,
				meshCloud->points[pointIdxNKNSearch[2]].y,
				meshCloud->points[pointIdxNKNSearch[2]].z;* /

			// do calculation
			Eigen::Vector3f n = (p2 - p1).cross(p3 - p1);
			float dis = (p0 - p1).dot(n.normalized());
			uint32_t rgb = 255 << 16 | 255 << 8 | 255;
			if (fabs(dis) > 0.12) {
				rgb = 255 << 16 | 0 << 8 | 0;
			}
			else if (dis >= 0 && dis <= 0.12) {
				int color = 255 * (dis / 0.12);
				rgb = 0 << 16 | color << 8 | 0;
			}
			else if (dis < 0 && dis >= -0.12) {
				int color = 255 * (dis / -0.12);
				rgb = 0 << 16 | 0 << 8 | color;
			}
			else { //this should not happen
				PCL_ERROR("Overestimated value: %f\n", dis);
			}
			//PCL_INFO("dis:%f\n", dis);
			cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
		}
	}*/
	
	time.tic();
	pcl::KdTreeFLANN<PointNormalT> kdTree;
	kdTree.setInputCloud(meshSampCloud);
	PointNormalT searchPoint;
	int K = 3;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	Eigen::Vector3f p0(0, 0, 0), p1(0, 0, 0), p2(0, 0, 0), p3(0, 0, 0), 
		n1(0, 0, 0), n2(0, 0, 0), n3(0, 0, 0);

	for (size_t i = 0; i < cloud->size(); i++) {
		searchPoint.x = cloud->points[i].x;
		searchPoint.y = cloud->points[i].y;
		searchPoint.z = cloud->points[i].z;
		if (kdTree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
			p0 << searchPoint.x, searchPoint.y, searchPoint.z;

			// Cannot find any index, it should not happen
			p1 << meshSampCloud->points[pointIdxNKNSearch[0]].x,
				meshSampCloud->points[pointIdxNKNSearch[0]].y,
				meshSampCloud->points[pointIdxNKNSearch[0]].z;
			p2 << meshSampCloud->points[pointIdxNKNSearch[1]].x,
				meshSampCloud->points[pointIdxNKNSearch[1]].y,
				meshSampCloud->points[pointIdxNKNSearch[1]].z;
			p3 << meshSampCloud->points[pointIdxNKNSearch[2]].x,
				meshSampCloud->points[pointIdxNKNSearch[2]].y,
				meshSampCloud->points[pointIdxNKNSearch[2]].z;
			n1 << meshSampCloud->points[pointIdxNKNSearch[0]].normal_x,
				meshSampCloud->points[pointIdxNKNSearch[0]].normal_y,
				meshSampCloud->points[pointIdxNKNSearch[0]].normal_z;
			n2 << meshSampCloud->points[pointIdxNKNSearch[1]].normal_x,
				meshSampCloud->points[pointIdxNKNSearch[1]].normal_y,
				meshSampCloud->points[pointIdxNKNSearch[1]].normal_z;
			n3 << meshSampCloud->points[pointIdxNKNSearch[2]].normal_x,
				meshSampCloud->points[pointIdxNKNSearch[2]].normal_y,
				meshSampCloud->points[pointIdxNKNSearch[2]].normal_z;

			// do calculation
			Eigen::Vector3f n = (n1+n2+n3)/3;

			float dis = (p0 - p1).dot(n.normalized());
			uint32_t rgb = 255 << 16 | 255 << 8 | 255;
			if (fabs(dis) > 0.12) {
				rgb = 255 << 16 | 0 << 8 | 0;
			}
			else if (dis >= 0 && dis <= 0.12) {
				int color = 255 * (dis / 0.12);
				rgb = 0 << 16 | color << 8 | 0;
			}
			else if (dis < 0 && dis >= -0.12) {
				int color = 255 * (dis / -0.12);
				rgb = 0 << 16 | 0 << 8 | color;
			}
			else { //this should not happen
				PCL_ERROR("Unexpected value: %f\n", dis);
			}
			//PCL_INFO("dis:%f\n", dis);
			cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
		}
	}
	PCL_INFO("calculate distance in: %f seconds.\n", time.toc() / 1000.0);

	// Create a PCLVisualizer object
	viewer = new pcl::visualization::PCLVisualizer(argc, argv, "Viewer test");
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> handler(cloud);
	std::string cloudName = "input cloud";
	viewer->addPointCloud<PointT>(cloud, handler, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudName);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
	//viewer->addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	// Show bounding box of data
	viewer->addCube(minP.x, maxP.x, minP.y, maxP.y, minP.z, maxP.z, 255, 0, 0, "AABB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, "AABB");

	// Show model
	//viewer->addPolygonMesh(polygonMesh, "mesh");

	viewer->addCube(minP2.x, maxP2.x, minP2.y, maxP2.y, minP2.z, maxP2.z, 0, 255, 0, "meshAABB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "meshAABB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, "meshAABB");

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0.013336, -1.73476, 144.206, -0.999603, 0.0281814, 0.000434471);
	viewer->setSize(1366, 768);

	// -----Main loop-----
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}