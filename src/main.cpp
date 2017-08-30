#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>   // TicToc

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

	//PointCloud cloud1;
	//uint8_t r(255), g(15), b(15);
	//for (float z(-1.0); z <= 1.0; z += 0.05) {
	//	for (float angle(0.0); angle <= 360.0; angle += 5.0) {
	//		PointT point;
	//		point.x = 0.5 * cosf(pcl::deg2rad(angle));
	//		point.y = sinf(pcl::deg2rad(angle));
	//		point.z = z;
	//		r = g = b = static_cast<int>(angle) % 256;
	//		uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
	//			static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	//		point.rgb = *reinterpret_cast<float*>(&rgb);
	//		cloud1.push_back(point);
	//	}
	//}
	//cloud1.width = cloud1.size();
	//cloud1.height = 1;
	//cloud1.is_dense = true;

	// Create a PCLVisualizer object
	viewer = new pcl::visualization::PCLVisualizer(argc, argv, "Viewer test");
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> handler(cloud);
	std::string cloudName = "input cloud";
	viewer->addPointCloud<PointT>(cloud, handler, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudName);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
	//viewer->addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	double pu0[3] = { -26., 64.9, 15.432 };
	double pu1[3] = { 26., 65.0, 15.453 };
	double pu2[3] = { 25.5, -65.9, 15.458 };
	double pu3[3] = { -25.8, -65.9, 15.5 };
	double pb0[3] = { -25.9, 64.9, 15.152 };
	double pb1[3] = { 26., 65.0, 15.163 };
	double pb2[3] = { 25.5, -65.9, 15.149 };
	double pb3[3] = { -25.8, -65.9, 15.209 };
	vtkSmartPointer<vtkPoints> pointsU = vtkSmartPointer<vtkPoints>::New();
	pointsU->InsertNextPoint(pu0);
	pointsU->InsertNextPoint(pu1);
	pointsU->InsertNextPoint(pu3);
	pointsU->InsertNextPoint(pu2);
	vtkSmartPointer<vtkPoints> pointsB = vtkSmartPointer<vtkPoints>::New();
	pointsB->InsertNextPoint(pb0);
	pointsB->InsertNextPoint(pb1);
	pointsB->InsertNextPoint(pb3);
	pointsB->InsertNextPoint(pb2);

	// Setup the colors array
	uint8_t z0 = static_cast<uint8_t>(zToColor(15.432));
	uint8_t z1 = static_cast<uint8_t>(zToColor(15.453));
	uint8_t z2 = static_cast<uint8_t>(zToColor(15.458));
	uint8_t z3 = static_cast<uint8_t>(zToColor(15.5));
	uint8_t c0[3] = { z0, z0, z0 };
	uint8_t c1[3] = { z1, z1, z1 };
	uint8_t c2[3] = { z2 ,z2 ,z2 };
	uint8_t c3[3] = { z3 ,z3, z3 };
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");
	colors->InsertNextTupleValue(c0);
	colors->InsertNextTupleValue(c1);
	colors->InsertNextTupleValue(c3);
	colors->InsertNextTupleValue(c2);
	z0 = static_cast<uint8_t>(zToColor(15.152));
	z1 = static_cast<uint8_t>(zToColor(15.163));
	z2 = static_cast<uint8_t>(zToColor(15.149));
	z3 = static_cast<uint8_t>(zToColor(15.209));
	uint8_t cb0[3] = { z0, z0, z0 };
	uint8_t cb1[3] = { z1, z1, z1 };
	uint8_t cb2[3] = { z2 ,z2 ,z2 };
	uint8_t cb3[3] = { z3 ,z3, z3 };
	vtkSmartPointer<vtkUnsignedCharArray> colorsB = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colorsB->SetNumberOfComponents(3);
	colorsB->SetName("ColorsB");
	colorsB->InsertNextTupleValue(cb0);
	colorsB->InsertNextTupleValue(cb1);
	colorsB->InsertNextTupleValue(cb3);
	colorsB->InsertNextTupleValue(cb2);

	vtkSmartPointer<vtkTriangleStrip> triangleStripU = vtkSmartPointer<vtkTriangleStrip>::New();
	triangleStripU->GetPointIds()->SetNumberOfIds(4);
	triangleStripU->GetPointIds()->SetId(0, 0);
	triangleStripU->GetPointIds()->SetId(1, 1);
	triangleStripU->GetPointIds()->SetId(2, 2);
	triangleStripU->GetPointIds()->SetId(3, 3);
	vtkSmartPointer<vtkTriangleStrip> triangleStripB = vtkSmartPointer<vtkTriangleStrip>::New();
	triangleStripB->GetPointIds()->SetNumberOfIds(4);
	triangleStripB->GetPointIds()->SetId(0, 0);
	triangleStripB->GetPointIds()->SetId(1, 1);
	triangleStripB->GetPointIds()->SetId(2, 2);
	triangleStripB->GetPointIds()->SetId(3, 3);

	vtkSmartPointer<vtkCellArray> cellsU = vtkSmartPointer<vtkCellArray>::New();
	cellsU->InsertNextCell(triangleStripU);
	vtkSmartPointer<vtkCellArray> cellsB = vtkSmartPointer<vtkCellArray>::New();
	cellsB->InsertNextCell(triangleStripB);

	vtkSmartPointer<vtkPolyData> polyDataU = vtkSmartPointer<vtkPolyData>::New();
	polyDataU->SetPoints(pointsU);
	polyDataU->SetStrips(cellsU);
	polyDataU->GetPointData()->SetScalars(colors);
	vtkSmartPointer<vtkPolyData> polyDataB = vtkSmartPointer<vtkPolyData>::New();
	polyDataB->SetPoints(pointsB);
	polyDataB->SetStrips(cellsB);
	polyDataB->GetPointData()->SetScalars(colorsB);

	vtkSmartPointer<vtkPolyDataMapper> mapperU = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperU->SetInputData(polyDataU);
	vtkSmartPointer<vtkPolyDataMapper> mapperB = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperB->SetInputData(polyDataB);
	vtkSmartPointer<vtkActor> actorU = vtkSmartPointer<vtkActor>::New();
	actorU->SetMapper(mapperU);
	actorU->GetProperty()->SetAmbient(1);
	actorU->GetProperty()->SetDiffuse(0);// turn off diffuse light
	vtkSmartPointer<vtkActor> actorB = vtkSmartPointer<vtkActor>::New();
	actorB->SetMapper(mapperB);
	actorB->GetProperty()->SetAmbient(1);
	actorB->GetProperty()->SetDiffuse(0);// turn off diffuse light

	vtkSmartPointer<vtkRenderWindow> vtkRV = viewer->getRenderWindow();
	vtkSmartPointer<vtkRenderer> renderer = vtkRV->GetRenderers()->GetFirstRenderer();

	renderer->AddActor(actorU);
	renderer->AddActor(actorB);
	
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