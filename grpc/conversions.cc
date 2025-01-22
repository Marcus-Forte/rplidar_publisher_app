
#include "conversions.hh"

msensor::Scan3D fromGRPC(const sensors::PointCloud3 &msg) {
  msensor::Scan3D pointcloud;

  pointcloud.points.reserve(msg.points_size());
  for (const auto &pt : msg.points()) {
    pointcloud.points.emplace_back(pt.x(), pt.y(), pt.z());
  }
  pointcloud.timestamp = msg.timestamp();

  return pointcloud;
}

msensor::IMUData fromGRPC(const sensors::IMUData &msg) {

  return {
      msg.ax(), msg.ay(), msg.az(),        msg.gx(),
      msg.gy(), msg.gz(), msg.timestamp(),
  };
}
