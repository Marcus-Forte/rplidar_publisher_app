#pragma once

#include "imu/IImu.hh"
#include "lidar/ILidar.hh"

#include "sensors.grpc.pb.h"
#include <deque>
#include <grpcpp/channel.h>
#include <memory>
#include <thread>

/**
 * @brief This class connects to a SensorService and provides methods to get
 * sensor data remotely.
 *
 */
class SensorsClient : public msensor::ILidar, public msensor::IImu {
public:
  SensorsClient(const std::string &remote_ip);
  virtual ~SensorsClient();
  void init() override;
  void start();
  void stop();
  void startSampling() override;
  void stopSampling() override;

  msensor::Scan3D getScan() override;
  msensor::IMUData getImuData() override;

private:
  std::string remote_ip_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<sensors::SensorService::Stub> service_stub_;

  std::jthread read_thread_;
  std::jthread imu_reader_thread_;
  std::unique_ptr<grpc::ClientContext> context_;

  std::deque<msensor::Scan3D> scans_;
  std::deque<msensor::IMUData> imu_measurements_;
};