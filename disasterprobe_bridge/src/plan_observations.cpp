#include <iostream>
#include <limits>
#include <exception>
#include <fstream>

#include "ros/ros.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/common/transforms.h>  
#include <pcl/common/centroid.h> 

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <opencv2/core.hpp>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <boost/algorithm/string.hpp>

// #include <OpenREALM/realm_core/enums.h>
// #include <OpenREALM/realm_core/frame.h>
// #include <OpenREALM/realm_core/utm32.h>
// #include <OpenREALM/realm_core/structs.h>
// #include <OpenREALM/realm_core/camera.h>
// #include <OpenREALM/realm_core/analysis.h>

// #include <OpenREALM/realm_core/cv_grid_map.h>
// #include <realm_ros/conversions.h>
// #include <realm_msgs/CvGridMap.h>

// #include <geometry_msgs/Pose.h>
// #include <disasterprobe_bridge/PlanObservations.h>


using namespace std;
using namespace octomap;

typedef octomap::ColorOcTree OcTreeT;


point3d eigen2point(Eigen::Vector3d vec) {
  return point3d(vec(0), vec(1), vec(2));
}

Eigen::Vector3d point2eigen(point3d point) {
  return Eigen::Vector3d(point(0), point(1), point(2));
}

void convert_to_octomap(pcl::PointCloud<pcl::PointXYZRGB> pointcloud, OcTreeT *tree, Eigen::Affine3d *utm_transform) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr translated_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::copyPointCloud(pointcloud, *cloud_ptr);

 	Eigen::Vector4d center; 
  pcl::compute3DCentroid(*cloud_ptr, center);
  std::cout << center.matrix() << std::endl;

  // Transform to recenter the pcl
  *utm_transform = Eigen::Affine3d::Identity();
  utm_transform->translation() << -center(0), -center(1), 0.0;
  // std::cout << utm_transform.matrix() << std::endl;

  pcl::transformPointCloud(*cloud_ptr, *translated_cloud_ptr, *utm_transform);

  pcl::VoxelGrid<pcl::PointXYZRGB> vgd;
  vgd.setInputCloud(translated_cloud_ptr);
  vgd.setLeafSize (0.1f, 0.1f, 0.1f); // milimeters
  vgd.filter (*filtered_cloud_ptr);

	octomap::Pointcloud octocloud;
	
	for (int i = 0; i < filtered_cloud_ptr->points.size(); i++) {
		point3d endpoint(filtered_cloud_ptr->points[i].x,filtered_cloud_ptr->points[i].y, filtered_cloud_ptr->points[i].z);
		octocloud.push_back(endpoint);
	}

	point3d sensorOrigin(0,0,0);
	tree->insertPointCloud(octocloud, sensorOrigin);
}

bool find_observables(OcTreeT *tree, point3d vehicle_position_utm, double vehicle_rotation, Eigen::Affine3d utm_transform, vector<double>* result) {
  int steps = 10;
  float observe_angle = 30;
  float observe_altitude = 10;


  point3d vehicle_position = eigen2point(utm_transform*point2eigen(vehicle_position_utm));

  // find key for vehicle position
  octomap::OcTreeKey vehicle_key = tree->coordToKey(vehicle_position);
  // TODO: check undefined?
  // TODO: check if observable
  // if vehicle height is equal to ground, hard to observe
  // maybe draw sphere around vehicle, or hit should be distance from vehicle?

  // color vehicle node
  tree->setNodeColor(vehicle_key, 0, 0, 255);
  vehicle_position = tree->keyToCoord(vehicle_key);

  Eigen::Affine3d trace_transform = Eigen::Affine3d::Identity();

  float observe_distance = observe_altitude / tan(observe_angle * M_PI/180);
  Eigen::Vector3d start = Eigen::Vector3d(observe_distance, 0.0, observe_altitude);
  cout << "Observe altitude: " << observe_altitude << " with angle: " << observe_angle << " distance " << observe_distance << endl;


  octomath::Vector3 up_direction = octomath::Vector3(0, 0, 1);
  octomath::Vector3 down_direction = octomath::Vector3(0, 0, -1);

  bool observed = false;

  for (int j = 0; j < 2; j++) { // front and back
    for (int i = 0; i < steps; i++) {

      double angle = -40.0 + 80.0 * i/steps + vehicle_rotation + 180*j;

      trace_transform =  Eigen::Translation<double,3>(0.0, 0.0, 0.0)*
            Eigen::AngleAxis<double>(angle * M_PI/180, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxis<double>(0.0 * M_PI/180, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxis<double>(0.0 * M_PI/180, Eigen::Vector3d::UnitX());

      point3d view_location = eigen2point(trace_transform * start) + vehicle_position;
      octomath::Vector3 direction = octomath::Vector3(vehicle_position.x() - view_location.x(), 
                                                      vehicle_position.y() - view_location.y(),
                                                      vehicle_position.z() - view_location.z());

      octomap::OcTreeKey view_key = tree->coordToKey(view_location);

      // TODO: check if area around view is free

      cout << "casting ray from " << view_location << " in the " << direction << " direction"<< endl;
      point3d ray_end, up_end, down_end;
      bool success = tree->castRay(view_location, direction, ray_end, true);

      if(success){ // has hit obstacle
        cout << "ray hit cell with center " << ray_end << endl;
        octomap::OcTreeKey end_key = tree->coordToKey(ray_end);

        if (end_key == vehicle_key) {
          cout << "Vehicle was observed" << endl;
          
          // check if obstacles above
          bool up_success = tree->castRay(view_location, up_direction, up_end, true, 30);
          bool down_success = tree->castRay(view_location, down_direction, down_end, true, 30);

          tree->updateNode(view_key, true);
          if (up_success) { // obstacle above
            cout << "Obstacle above area" << endl;
            tree->setNodeColor(view_key, 255, 0, 255);
          } else {
            cout << "Flight location accessible from top" << endl;
            if (down_success){
              cout << "Area below is known" << endl;
              tree->setNodeColor(view_key, 0, 255, 0);
              if (!observed) {
                cout << "---------------- found ------------" << endl;
                // view_location 
                Eigen::Vector3d view_location_utm = utm_transform.inverse()*point2eigen(view_location);
                (*result)[0] = view_location_utm[0];
                (*result)[1] = view_location_utm[1];
                (*result)[2] = view_location_utm[2];

                // direction 
                observed = true;
              }
            } else {
              cout << "Area below unknown" << endl;
              tree->setNodeColor(view_key, 0, 255, 255);

            }
          }
        } else {
          cout << "Obstacle was hit" << endl;
          tree->updateNode(view_key, true);
          tree->setNodeColor(view_key, 255, 0, 0);
          tree->setNodeColor(end_key, 255, 255, 0);
        }
      } else { 
        cout << "No intersection, false" << view_location << endl;

        if (tree->search(ray_end) == NULL) {
          cout << "End was NULL" << endl;
        } else {
          cout << "Ray end: " << ray_end << endl;
        }
      }
    }
  }
  return observed;
}

vector<vector<double>> calculate(vector<vector<double>> v_vehicles, string ply_path) {
  Eigen::Affine3d utm_transform; // transform from UTM to local
	pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
	pcl::io::loadPLYFile<pcl::PointXYZRGB>(ply_path, pointcloud);
	OcTreeT tree(1); // resolution

  convert_to_octomap(pointcloud, &tree, &utm_transform);

  // Measured vehicle position
  vector<vector<double>> observation_positions;

  vector<vector<double>>::iterator i;
  for(i=v_vehicles.begin(); i != v_vehicles.end(); i++) {
    point3d vehicle_position = {(*i)[0], (*i)[1], (*i)[5]};
    double vehicle_angle = (*i)[4];

    vector<double> result = {0, 0, 0, 0, 0, 0};
    bool success = find_observables(&tree, vehicle_position, vehicle_angle, utm_transform, &result);
    if (success) {

    }
    observation_positions.push_back(result);
  }

  tree.write("elevation.ot");
  return observation_positions;
}

int main(int argc, char** argv) 
{ 
    cout << "Input args: " << endl;
    for (int i = 0; i < argc; ++i) 
        cout << argv[i] << "\n"; 
    
    std::string ply_path = argv[1];
    std::string vehicles_path = argv[2];
    std::string output_path = argv[3];

    std::ifstream infile(vehicles_path);
    std::vector<std::vector<double>> v_vehicle_coordinates;

    std::string line;
    while(std::getline(infile, line)) {
      // cx cy h w angle alt
      std::vector<std::string> result;
      boost::trim_right(line);
      boost::split(result, line, boost::is_any_of("\n "));

      vector<double> vehicle_coordinates;

      vector<string>::iterator i ;       
      for(i = result.begin() ; i != result.end(); i++) {
        vehicle_coordinates.push_back(std::stod(*i));
      }
      v_vehicle_coordinates.push_back(vehicle_coordinates);
    }

    vector<vector<double>> observations = calculate(v_vehicle_coordinates, ply_path);

    std::ofstream outfile(output_path);
    vector<vector<double>>::iterator j ;       
    for(j = observations.begin() ; j != observations.end(); j++) {
      for (const auto &e : *j) outfile << e << " ";
      outfile << "\n";
    }
    return 0; 
} 