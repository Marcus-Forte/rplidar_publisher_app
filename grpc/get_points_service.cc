#include "get_points_service.hh"
#include <chrono>
#include <grpcpp/support/status.h>
#include <thread>

struct Point2 {
  float x;
  float y;
};

ScanService::ScanService() = default;

grpc::Status
ScanService::getScan(::grpc::ServerContext *context,
                     const ::google::protobuf::Empty * /*request*/,
                     ::grpc::ServerWriter<lidar::PointCloud3> *writer) {
  static bool s_client_connected = false;
  if (s_client_connected)
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client supported");

  std::cout << "Start stream." << std::endl;
  s_client_connected = true;
  while (!context->IsCancelled()) {

    while (!scan_queue_.empty()) {
      auto scan = scan_queue_.front();
      lidar::PointCloud3 point_cloud;

      for (const auto &point : scan) {
        auto pt = point_cloud.add_points();
        pt->set_x(point.x);
        pt->set_y(point.y);
        // 2D Lidar
        pt->set_z(0.0);
        pt->set_r(0.0);
        pt->set_g(1.0);
        pt->set_b(0.0);
      }
      writer->Write(point_cloud);
      scan_queue_.pop_front();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::cout << "Canceled stream." << std::endl;
  s_client_connected = false;
  return ::grpc::Status::OK;
}

void ScanService::putScan(const std::vector<Point2> &scan) {
  scan_queue_.push_front(scan);
  // Limit to 100 samples
  if (scan_queue_.size() > 100) {
    scan_queue_.pop_back();
  }
}