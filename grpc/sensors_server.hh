#pragma once

#include "sensors_service.hh"
#include <grpcpp/grpcpp.h>
#include <grpcpp/server_builder.h>

/**
 * @brief This class manages the gRPC server and provides methods to publish
 * data.
 *
 */
class SensorsServer {
public:
  SensorsServer();

  void start();
  void stop();

  void publishScan(const msensor::Scan3D &scan);
  void publishImu(const msensor::IMUData &data);

private:
  ScanService scan_service_;
  std::unique_ptr<grpc::Server> server_;
};