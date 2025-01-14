// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// GNU-GPL licence that follows one of libQGLViewer.

#include "./visualizer.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <limits>
#include <string>
#include <vector>

#include <cstdlib>  // For rand() and srand()

namespace depth_clustering {

using std::array;
using std::string;
using std::to_string;
using std::vector;

using std::lock_guard;
using std::map;
using std::mutex;
using std::string;
using std::thread;
using std::unordered_map;
using std::vector;

// static vector<array<int, 3>> COLORS;

// Visualizer::Visualizer(QWidget* parent)
//     : QGLViewer(parent), AbstractClient<Cloud>(), _updated{false} {
//   _cloud_obj_storer.SetUpdateListener(this);
// }

Visualizer::Visualizer(QWidget* parent, ros::NodeHandle* nh)
    : QGLViewer(parent), AbstractClient<Cloud>(), _nh(nh) {
  _cloud_obj_storer.SetUpdateListener(this);

  if (_nh) {
  object_segments_pub = _nh->advertise<visualization_msgs::MarkerArray>("/depth_clustering/object_segments", 1);
  } else {
    std::cout << "Error: NodeHandle is null in Visualizer constructor." << std::endl;
  }

  srand(static_cast<unsigned int>(time(nullptr))); // Seed the random number generator
}

void Visualizer::draw() {
  lock_guard<mutex> guard(_cloud_mutex);
  DrawCloud(_cloud);
  
  int id = 0; // Initialize marker ID

  for (const auto& kv : _cloud_obj_storer.object_clouds()) {
    const auto& cluster = kv.second;
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f extent = Eigen::Vector3f::Zero();
    Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest());
    Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max());
    for (const auto& point : cluster.points()) {
      center = center + point.AsEigenVector();
      min_point << std::min(min_point.x(), point.x()),
          std::min(min_point.y(), point.y()),
          std::min(min_point.z(), point.z());
      max_point << std::max(max_point.x(), point.x()),
          std::max(max_point.y(), point.y()),
          std::max(max_point.z(), point.z());
    }
    center /= cluster.size();
    if (min_point.x() < max_point.x()) {
      extent = max_point - min_point;
    }

    float volume = extent.x() * extent.y() * extent.z();
    
    if (volume > 0.002f && extent.x() < 0.3 && extent.y() < 0.3 && extent.z() < 4 && extent.z() > 0.3) {    
      DrawCube(center, extent);
      PublishObjectSegments(kv, id);
    }
  }
  // ros::Duration(0.1).sleep();  // Small delay to separate the publishing of each cluster
  
}

void Visualizer::init() {
  // setSceneRadius(100.0);
  setSceneRadius(5.0);   // Default: 10
  camera()->showEntireScene();
  glDisable(GL_LIGHTING);
}

void Visualizer::DrawCloud(const Cloud& cloud) {
  glPushMatrix();
  glBegin(GL_POINTS);
  glColor3f(1.0f, 1.0f, 1.0f);
  for (const auto& point : cloud.points()) {
    glVertex3f(point.x(), point.y(), point.z());
  }
  glEnd();
  glPopMatrix();
}

void Visualizer::DrawCube(const Eigen::Vector3f& center,
                          const Eigen::Vector3f& scale) {
  glPushMatrix();
  glTranslatef(center.x(), center.y(), center.z());
  glScalef(scale.x(), scale.y(), scale.z());
  // float volume = scale.x() * scale.y() * scale.z();
  
  glColor3f(0.0f, 0.0f, 1.0f);
    glLineWidth(2.0f);

    glBegin(GL_LINE_STRIP);

    // Bottom of Box
    glVertex3f(-0.5, -0.5, -0.5);
    glVertex3f(-0.5, -0.5, 0.5);
    glVertex3f(0.5, -0.5, 0.5);
    glVertex3f(0.5, -0.5, -0.5);
    glVertex3f(-0.5, -0.5, -0.5);

    // Top of Box
    glVertex3f(-0.5, 0.5, -0.5);
    glVertex3f(-0.5, 0.5, 0.5);
    glVertex3f(0.5, 0.5, 0.5);
    glVertex3f(0.5, 0.5, -0.5);
    glVertex3f(-0.5, 0.5, -0.5);

    glEnd();

    glBegin(GL_LINES);
    // For the Sides of the Box

    glVertex3f(-0.5, 0.5, -0.5);
    glVertex3f(-0.5, -0.5, -0.5);

    glVertex3f(-0.5, -0.5, 0.5);
    glVertex3f(-0.5, 0.5, 0.5);

    glVertex3f(0.5, -0.5, 0.5);
    glVertex3f(0.5, 0.5, 0.5);

    glVertex3f(0.5, -0.5, -0.5);
    glVertex3f(0.5, 0.5, -0.5);

    glEnd();

    glColor3f(1.0f, 1.0f, 0.0f);
    glLineWidth(8.0f);

    glBegin(GL_LINES);
    // For the Sides of the Box

    glVertex3f(0.0, 0.0, 0.5);
    glVertex3f(0.0, 0.0, -0.5);

    glEnd();

  // if (volume > 0.002f && scale.x() < 0.3 && scale.y() < 0.3 && scale.z() < 4 && scale.z() > 0.3) {
  //   glColor3f(0.0f, 0.0f, 1.0f);
  //   glLineWidth(2.0f);

  //   glBegin(GL_LINE_STRIP);

  //   // Bottom of Box
  //   glVertex3f(-0.5, -0.5, -0.5);
  //   glVertex3f(-0.5, -0.5, 0.5);
  //   glVertex3f(0.5, -0.5, 0.5);
  //   glVertex3f(0.5, -0.5, -0.5);
  //   glVertex3f(-0.5, -0.5, -0.5);

  //   // Top of Box
  //   glVertex3f(-0.5, 0.5, -0.5);
  //   glVertex3f(-0.5, 0.5, 0.5);
  //   glVertex3f(0.5, 0.5, 0.5);
  //   glVertex3f(0.5, 0.5, -0.5);
  //   glVertex3f(-0.5, 0.5, -0.5);

  //   glEnd();

  //   glBegin(GL_LINES);
  //   // For the Sides of the Box

  //   glVertex3f(-0.5, 0.5, -0.5);
  //   glVertex3f(-0.5, -0.5, -0.5);

  //   glVertex3f(-0.5, -0.5, 0.5);
  //   glVertex3f(-0.5, 0.5, 0.5);

  //   glVertex3f(0.5, -0.5, 0.5);
  //   glVertex3f(0.5, 0.5, 0.5);

  //   glVertex3f(0.5, -0.5, -0.5);
  //   glVertex3f(0.5, 0.5, -0.5);

  //   glEnd();

  //   glColor3f(1.0f, 1.0f, 0.0f);
  //   glLineWidth(8.0f);

  //   glBegin(GL_LINES);
  //   // For the Sides of the Box

  //   glVertex3f(0.0, 0.0, 0.5);
  //   glVertex3f(0.0, 0.0, -0.5);

  //   glEnd();

    
  // } else {
  //   glColor3f(0.3f, 0.3f, 0.3f);
  //   glLineWidth(0.02f);
  // }

  // glBegin(GL_LINES);
  // // For the Sides of the Box

  // glVertex3f(0.0, 0.0, 0.5);
  // glVertex3f(0.0, 0.0, -0.5);

  // glEnd();

  // glBegin(GL_LINE_STRIP);

  // // Bottom of Box
  // glVertex3f(-0.5, -0.5, -0.5);
  // glVertex3f(-0.5, -0.5, 0.5);
  // glVertex3f(0.5, -0.5, 0.5);
  // glVertex3f(0.5, -0.5, -0.5);
  // glVertex3f(-0.5, -0.5, -0.5);

  // // Top of Box
  // glVertex3f(-0.5, 0.5, -0.5);
  // glVertex3f(-0.5, 0.5, 0.5);
  // glVertex3f(0.5, 0.5, 0.5);
  // glVertex3f(0.5, 0.5, -0.5);
  // glVertex3f(-0.5, 0.5, -0.5);

  // glEnd();

  // glBegin(GL_LINES);
  // // For the Sides of the Box

  // glVertex3f(-0.5, 0.5, -0.5);
  // glVertex3f(-0.5, -0.5, -0.5);

  // glVertex3f(-0.5, -0.5, 0.5);
  // glVertex3f(-0.5, 0.5, 0.5);

  // glVertex3f(0.5, -0.5, 0.5);
  // glVertex3f(0.5, 0.5, 0.5);

  // glVertex3f(0.5, -0.5, -0.5);
  // glVertex3f(0.5, 0.5, -0.5);

  // glEnd();
  glPopMatrix();
}

Visualizer::~Visualizer() {}

void Visualizer::OnNewObjectReceived(const Cloud& cloud, const int) {
  lock_guard<mutex> guard(_cloud_mutex);
  _cloud = cloud;
}

void Visualizer::onUpdate() { this->update(); }

unordered_map<uint16_t, Cloud> ObjectPtrStorer::object_clouds() const {
  lock_guard<mutex> guard(_cluster_mutex);
  return _obj_clouds;
}

void ObjectPtrStorer::OnNewObjectReceived(
    const unordered_map<uint16_t, Cloud>& clouds, const int) {
  lock_guard<mutex> guard(_cluster_mutex);
  _obj_clouds = clouds;

  if (_update_listener) {
    _update_listener->onUpdate();
  }
}

// void Visualizer::PublishObjectSegments(std::pair<const uint16_t, Cloud> kv, int& id) 
// {
//   visualization_msgs::MarkerArray marker_array;

//   const auto& cluster = kv.second;

//   visualization_msgs::Marker marker;
//   marker.header.frame_id = "velodyne1_";  // Adjust as needed
//   marker.header.stamp = ros::Time::now();
//   marker.ns = "object_segments_markers";
//   marker.id = id++;
//   marker.type = visualization_msgs::Marker::POINTS;
//   marker.action = visualization_msgs::Marker::ADD;
//   marker.scale.x = 0.01;  // Size of points
//   marker.scale.y = 0.01;
//   marker.scale.z = 0.01;
//   marker.color.a = 1.0;  // Alpha

//   // Assign a random color
//   marker.color.r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
//   marker.color.g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
//   marker.color.b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

//   marker.lifetime = ros::Duration(1.0);  // Lifetime of the marker (3 seconds)

//   for (const auto& point : cluster.points()) {
//       geometry_msgs::Point ros_point;
//       ros_point.x = point.x();
//       ros_point.y = point.y();
//       ros_point.z = point.z();
//       marker.points.push_back(ros_point);
//   }

//   marker_array.markers.push_back(marker);

//   if (_nh) {
//       object_segments_pub.publish(marker_array);
//   }
// }

void Visualizer::PublishObjectSegments(std::pair<const uint16_t, Cloud> kv, int& id) {
    visualization_msgs::MarkerArray marker_array;

    // Clear previous markers
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    object_segments_pub.publish(marker_array);

    marker_array.markers.clear();  // Clear the array for new markers

    const auto& cluster = kv.second;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne1_";  // Adjust as needed
    marker.header.stamp = ros::Time::now();
    marker.ns = "object_segments_markers";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;  // Size of points
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;  // Alpha

    // Assign a random color
    marker.color.r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    marker.color.g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    marker.color.b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

    marker.lifetime = ros::Duration(1.5);  // Lifetime of the marker

    for (const auto& point : cluster.points()) {
        geometry_msgs::Point ros_point;
        ros_point.x = point.x();
        ros_point.y = point.y();
        ros_point.z = point.z();
        marker.points.push_back(ros_point);
    }

    marker_array.markers.push_back(marker);

    if (_nh) {
        object_segments_pub.publish(marker_array);
    }
}

// void Visualizer::PublishObjectSegments(std::pair<const uint16_t, Cloud> kv, int& id) {
//   visualization_msgs::MarkerArray marker_array;
//   // int id = 0;

//   // for (const auto& kv : _cloud_obj_storer.object_clouds()) {
//   const auto& cluster = kv.second;

//   visualization_msgs::Marker marker;
//   marker.header.frame_id = "velodyne1_";  // Adjust as needed
//   marker.header.stamp = ros::Time(0);
//   // marker.header.stamp = ros::Time::now();
//   marker.ns = "object_segments_markers";
//   marker.id = id++;
//   marker.type = visualization_msgs::Marker::POINTS;
//   marker.action = visualization_msgs::Marker::ADD;
//   marker.scale.x = 0.01;  // Size of points, adjust as needed
//   marker.scale.y = 0.01;  // Size of points, adjust as needed
//   marker.scale.z = 0.01;  // Size of points, adjust as needed
//   marker.color.a = 1.0;  // Alpha

//   // Assign a random color
//   marker.color.r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
//   marker.color.g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
//   marker.color.b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

//   for (const auto& point : cluster.points()) {
//     geometry_msgs::Point ros_point;
//     ros_point.x = point.x();
//     ros_point.y = point.y();
//     ros_point.z = point.z();
//     marker.points.push_back(ros_point);
//   }

//   marker_array.markers.push_back(marker);
//   // }

//   if (_nh) {
//     object_segments_pub.publish(marker_array);
//   }
// }


}  // namespace depth_clustering
