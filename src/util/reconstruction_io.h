#ifndef SRC_UTIL_RECONSTRUCTION_IO_H_
#define SRC_UTIL_RECONSTRUCTION_IO_H_

#include <glog/logging.h>

#include <fstream>

#include "base/camera.h"
#include "base/image.h"
#include "base/point3d.h"
#include "util/endian.h"

using namespace colmap;

namespace DAGSfM {

inline size_t ComputeNumObservations(const EIGEN_STL_UMAP(image_t,
                                                          class Image) &
                                     images) {
  size_t num_obs = 0;
  // for (const image_t image_id : reg_image_ids_) {
  //     num_obs += Image(image_id).NumPoints3D();
  // }
  for (auto image : images) {
    num_obs += image.second.NumPoints3D();
  }
  return num_obs;
}

inline double ComputeMeanObservationsPerRegImage(
    const EIGEN_STL_UMAP(image_t, class Image) & images) {
  if (images.empty()) {
    return 0.0;
  } else {
    return ComputeNumObservations(images) / static_cast<double>(images.size());
  }
}

inline double ComputeMeanTrackLength(
    const EIGEN_STL_UMAP(image_t, class Image) & images,
    const EIGEN_STL_UMAP(point3D_t, class Point3D) & points3D) {
  if (points3D.empty()) {
    return 0.0;
  } else {
    return ComputeNumObservations(images) /
           static_cast<double>(points3D.size());
  }
}

inline void WriteCamerasText(const EIGEN_STL_UMAP(camera_t, class Camera) &
                                 cameras,
                             const std::string& path) {
  std::ofstream file(path, std::ios::trunc);
  CHECK(file.is_open()) << path;

  file << "# Camera list with one line of data per camera:" << std::endl;
  file << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]" << std::endl;
  file << "# Number of cameras: " << cameras.size() << std::endl;

  for (const auto& camera : cameras) {
    std::ostringstream line;

    line << camera.first << " ";
    line << camera.second.ModelName() << " ";
    line << camera.second.Width() << " ";
    line << camera.second.Height() << " ";

    for (const double param : camera.second.Params()) {
      line << param << " ";
    }

    std::string line_string = line.str();
    line_string = line_string.substr(0, line_string.size() - 1);

    file << line_string << std::endl;
  }
}

inline void WriteImagesText(const EIGEN_STL_UMAP(image_t, class Image) & images,
                            const std::string& path) {
  std::ofstream file(path, std::ios::trunc);
  CHECK(file.is_open()) << path;

  file << "# Image list with two lines of data per image:" << std::endl;
  file << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, "
          "NAME"
       << std::endl;
  file << "#   POINTS2D[] as (X, Y, POINT3D_ID)" << std::endl;
  file << "# Number of images: " << images.size()
       << ", mean observations per image: "
       << ComputeMeanObservationsPerRegImage(images) << std::endl;

  for (const auto& image : images) {
    if (!image.second.IsRegistered()) {
      continue;
    }

    std::ostringstream line;
    std::string line_string;

    line << image.first << " ";

    line << image.second.ClusterId() << " ";

    // QVEC (qw, qx, qy, qz)
    const Eigen::Vector4d normalized_qvec =
        NormalizeQuaternion(image.second.Qvec());
    line << normalized_qvec(0) << " ";
    line << normalized_qvec(1) << " ";
    line << normalized_qvec(2) << " ";
    line << normalized_qvec(3) << " ";

    // TVEC
    line << image.second.Tvec(0) << " ";
    line << image.second.Tvec(1) << " ";
    line << image.second.Tvec(2) << " ";

    line << image.second.CameraId() << " ";

    line << image.second.Name();

    file << line.str() << std::endl;

    line.str("");
    line.clear();

    for (const Point2D& point2D : image.second.Points2D()) {
      line << point2D.X() << " ";
      line << point2D.Y() << " ";
      if (point2D.HasPoint3D()) {
        line << point2D.Point3DId() << " ";
      } else {
        line << -1 << " ";
      }
    }
    line_string = line.str();
    line_string = line_string.substr(0, line_string.size() - 1);
    file << line_string << std::endl;
  }
}

inline void WritePoints3DText(const EIGEN_STL_UMAP(image_t, class Image) &
                                  images,
                              const EIGEN_STL_UMAP(point3D_t, class Point3D) &
                                  points3D,
                              const std::string& path) {
  std::ofstream file(path, std::ios::trunc);
  CHECK(file.is_open()) << path;

  file << "# 3D point list with one line of data per point:" << std::endl;
  file << "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, "
          "TRACK[] as (IMAGE_ID, POINT2D_IDX)"
       << std::endl;
  file << "# Number of points: " << points3D.size()
       << ", mean track length: " << ComputeMeanTrackLength(images, points3D)
       << std::endl;

  for (const auto& point3D : points3D) {
    file << point3D.first << " ";
    file << point3D.second.XYZ()(0) << " ";
    file << point3D.second.XYZ()(1) << " ";
    file << point3D.second.XYZ()(2) << " ";
    file << static_cast<int>(point3D.second.Color(0)) << " ";
    file << static_cast<int>(point3D.second.Color(1)) << " ";
    file << static_cast<int>(point3D.second.Color(2)) << " ";
    file << point3D.second.Error() << " ";

    std::ostringstream line;

    for (const auto& track_el : point3D.second.Track().Elements()) {
      line << track_el.image_id << " ";
      line << track_el.point2D_idx << " ";
    }

    std::string line_string = line.str();
    line_string = line_string.substr(0, line_string.size() - 1);

    file << line_string << std::endl;
  }
}

inline void WriteCamerasBinary(const EIGEN_STL_UMAP(camera_t, class Camera) &
                                   cameras,
                               const std::string& path) {
  std::ofstream file(path, std::ios::trunc | std::ios::binary);
  CHECK(file.is_open()) << path;

  WriteBinaryLittleEndian<uint64_t>(&file, cameras.size());

  for (const auto& camera : cameras) {
    WriteBinaryLittleEndian<camera_t>(&file, camera.first);
    WriteBinaryLittleEndian<int>(&file, camera.second.ModelId());
    WriteBinaryLittleEndian<uint64_t>(&file, camera.second.Width());
    WriteBinaryLittleEndian<uint64_t>(&file, camera.second.Height());
    for (const double param : camera.second.Params()) {
      WriteBinaryLittleEndian<double>(&file, param);
    }
  }
}

inline void WriteImagesBinary(const EIGEN_STL_UMAP(image_t, class Image) &
                                  images,
                              const std::string& path) {
  std::ofstream file(path, std::ios::trunc | std::ios::binary);
  CHECK(file.is_open()) << path;

  WriteBinaryLittleEndian<uint64_t>(&file, images.size());

  for (const auto& image : images) {
    if (!image.second.IsRegistered()) {
      continue;
    }

    WriteBinaryLittleEndian<image_t>(&file, image.first);

    WriteBinaryLittleEndian<size_t>(&file, image.second.ClusterId());

    const Eigen::Vector4d normalized_qvec =
        NormalizeQuaternion(image.second.Qvec());
    WriteBinaryLittleEndian<double>(&file, normalized_qvec(0));
    WriteBinaryLittleEndian<double>(&file, normalized_qvec(1));
    WriteBinaryLittleEndian<double>(&file, normalized_qvec(2));
    WriteBinaryLittleEndian<double>(&file, normalized_qvec(3));

    WriteBinaryLittleEndian<double>(&file, image.second.Tvec(0));
    WriteBinaryLittleEndian<double>(&file, image.second.Tvec(1));
    WriteBinaryLittleEndian<double>(&file, image.second.Tvec(2));

    WriteBinaryLittleEndian<camera_t>(&file, image.second.CameraId());

    const std::string name = image.second.Name() + '\0';
    file.write(name.c_str(), name.size());

    WriteBinaryLittleEndian<uint64_t>(&file, image.second.NumPoints2D());
    for (const Point2D& point2D : image.second.Points2D()) {
      WriteBinaryLittleEndian<double>(&file, point2D.X());
      WriteBinaryLittleEndian<double>(&file, point2D.Y());
      WriteBinaryLittleEndian<point3D_t>(&file, point2D.Point3DId());
    }
  }
}

inline void WritePoints3DBinary(const EIGEN_STL_UMAP(point3D_t, class Point3D) &
                                    points3D,
                                const std::string& path) {
  std::ofstream file(path, std::ios::trunc | std::ios::binary);
  CHECK(file.is_open()) << path;

  WriteBinaryLittleEndian<uint64_t>(&file, points3D.size());

  for (const auto& point3D : points3D) {
    WriteBinaryLittleEndian<point3D_t>(&file, point3D.first);
    WriteBinaryLittleEndian<double>(&file, point3D.second.XYZ()(0));
    WriteBinaryLittleEndian<double>(&file, point3D.second.XYZ()(1));
    WriteBinaryLittleEndian<double>(&file, point3D.second.XYZ()(2));
    WriteBinaryLittleEndian<uint8_t>(&file, point3D.second.Color(0));
    WriteBinaryLittleEndian<uint8_t>(&file, point3D.second.Color(1));
    WriteBinaryLittleEndian<uint8_t>(&file, point3D.second.Color(2));
    WriteBinaryLittleEndian<double>(&file, point3D.second.Error());

    WriteBinaryLittleEndian<uint64_t>(&file, point3D.second.Track().Length());
    for (const auto& track_el : point3D.second.Track().Elements()) {
      WriteBinaryLittleEndian<image_t>(&file, track_el.image_id);
      WriteBinaryLittleEndian<point2D_t>(&file, track_el.point2D_idx);
    }
  }
}

}  // namespace DAGSfM

#endif