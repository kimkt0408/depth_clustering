// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include "projections/cloud_projection.h"
#include <string>
#include <vector>

#include "utils/mem_utils.h"

#include <iostream>

namespace depth_clustering {

using mem_utils::make_unique;

CloudProjection::PointContainer::PointContainer() {}

CloudProjection::CloudProjection(const ProjectionParams& params)
    : _params(params) {
  if (!_params.valid()) {
    throw std::runtime_error("_params not valid for projection.");
  }

  _data = PointMatrix(_params.cols(), PointColumn(_params.rows()));
  _depth_image =
      cv::Mat::zeros(_params.rows(), _params.cols(), cv::DataType<float>::type);
}

RichPoint CloudProjection::UnprojectPoint(const cv::Mat& image, const int row,
                                          const int col) const {
  float depth = image.at<float>(row, col);
  Radians angle_z = this->_params.AngleFromRow(row);
  Radians angle_xy = this->_params.AngleFromCol(col);
  RichPoint point{depth * cosf(angle_z.val()) * cosf(angle_xy.val()),
                  depth * cosf(angle_z.val()) * sinf(angle_xy.val()),
                  depth * sinf(angle_z.val())};
  return point;
}

void CloudProjection::CheckCloudAndStorage(
    const RichPoint::AlignedVector& points) {
  if (this->_data.size() < 1) {
    throw std::length_error("_data size is < 1");
  }
  if (points.empty()) {
    throw std::runtime_error("cannot fill from cloud: no points");
  }
}

void CloudProjection::CheckImageAndStorage(const cv::Mat& image) {

  std::cout << "-----/src/projections/cloud_projection.cpp/CheckImageAndStorage: start-----" << std::endl;
  if (image.type() != CV_32F) {
    throw std::runtime_error("wrong image format");
  }

  std::cout << "-----/src/projections/cloud_projection.cpp/CheckImageAndStorage: 1-----" << std::endl;
  if (this->_data.size() < 1) {
    throw std::length_error("_data size is < 1");
  }

  std::cout << "-----/src/projections/cloud_projection.cpp/CheckImageAndStorage: 2-----" << std::endl;
  if (this->rows() != static_cast<size_t>(image.rows) ||
      this->cols() != static_cast<size_t>(image.cols)) {
    throw std::length_error("_data dimentions do not correspond to image ones");
  }
  std::cout << "-----/src/projections/cloud_projection.cpp/CheckImageAndStorage: end-----" << std::endl;
}

void CloudProjection::FixDepthSystematicErrorIfNeeded() {
  if (_depth_image.rows < 1) {
    fprintf(stderr, "[INFO]: image of wrong size, not correcting depth\n");
    return;
  }
  if (_corrections.size() != static_cast<size_t>(_depth_image.rows)) {
    fprintf(stderr, "[INFO]: Not correcting depth data.\n");
    return;
  }
  for (int r = 0; r < _depth_image.rows; ++r) {
    auto correction = _corrections[r];
    for (int c = 0; c < _depth_image.cols; ++c) {
      if (_depth_image.at<float>(r, c) < 0.001f) {
        continue;
      }
      _depth_image.at<float>(r, c) -= correction;
    }
  }
}

const cv::Mat& CloudProjection::depth_image() const {
  return this->_depth_image;
}

cv::Mat& CloudProjection::depth_image() { return this->_depth_image; }

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudProjection::ConvertToPointCloud(
    const cv::Mat& no_ground_image, const CloudProjection& projector) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (int r = 0; r < no_ground_image.rows; ++r) {
    for (int c = 0; c < no_ground_image.cols; ++c) {
      if (no_ground_image.at<float>(r, c) < 0.001f) {
        // Skip invalid or negligible depth values
        continue;
      }
      RichPoint point = projector.UnprojectPoint(no_ground_image, r, c);
      cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
  }

  return cloud;
}

sensor_msgs::PointCloud2 CloudProjection::ConvertToROSPointCloud2(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = "base_link";  // Set the frame ID
    ros_cloud.header.stamp = ros::Time::now();
    // ros_cloud.header.stamp = ros::Time(0); // Represents time zero, often used as a default

    return ros_cloud;
}


}  // namespace depth_clustering
