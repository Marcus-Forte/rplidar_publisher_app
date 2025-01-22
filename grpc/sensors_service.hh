#pragma once

#include "imu/IImu.hh"
#include "lidar/ILidar.hh"
#include "sensors.grpc.pb.h"

/**
 * @brief Call the methods of this service to publish data via gRPC.
 *
 */
class ScanService : public sensors::SensorService::Service {
public:
  ScanService();
  ::grpc::Status
  getScan(::grpc::ServerContext *context,
          const ::google::protobuf::Empty *request,
          ::grpc::ServerWriter<sensors::PointCloud3> *writer) override;

  ::grpc::Status
  getImu(::grpc::ServerContext *context,
         const ::google::protobuf::Empty *request,
         ::grpc::ServerWriter<sensors::IMUData> *writer) override;

  void putScan(const msensor::Scan3D &scan);
  void putImuData(const msensor::IMUData &imu_data);

private:
  std::deque<msensor::Scan3D> scan_queue_;
  std::deque<msensor::IMUData> imu_queue_;
};