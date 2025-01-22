#include "sensors_client.hh"
#include "conversions.hh"
#include <google/protobuf/empty.pb.h>
#include <grpcpp/client_context.h>
#include <grpcpp/grpcpp.h>
#include <mutex>
#include <thread>

constexpr int g_idleTimeMs = 5;
constexpr int g_LidarQueueSize = 10;
constexpr int g_imuQueueSize = 100;

std::mutex g_mutex;
std::mutex g_imu_mutex;

SensorsClient::SensorsClient(const std::string &remote_ip)
    : remote_ip_(remote_ip) {

  channel_ = grpc::CreateChannel(remote_ip, grpc::InsecureChannelCredentials());
  service_stub_ = sensors::SensorService::NewStub(channel_);
}

void SensorsClient::init() {}
void SensorsClient::startSampling() {}
void SensorsClient::stopSampling() {}

SensorsClient::~SensorsClient() { stop(); }

msensor::Scan3D SensorsClient::getScan() {

  if (!scans_.empty()) {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto pointcloud = scans_.front();
    scans_.pop_front();
    return pointcloud;
  }

  return {};
}

msensor::IMUData SensorsClient::getImuData() {

  if (!imu_measurements_.empty()) {
    std::lock_guard<std::mutex> lock(g_imu_mutex);
    const auto imu_data = imu_measurements_.front();
    imu_measurements_.pop_front();
    return imu_data;
  }

  return {};
}

void SensorsClient::start() {

  read_thread_ = std::jthread([&](std::stop_token stop_token) {
    auto service_context_ = std::make_unique<grpc::ClientContext>();
    google::protobuf::Empty empty_response;
    auto reader =
        service_stub_->getScan(service_context_.get(), empty_response);

    sensors::PointCloud3 msg;
    reader->Finish();
    while (!stop_token.stop_requested()) {
      if (!reader->Read(&msg)) {
        std::cout << "Unable to read remote lidar" << std::endl;
      } else {
        std::lock_guard<std::mutex> lock(g_mutex);
        scans_.push_back(fromGRPC(msg));

        if (scans_.size() > g_LidarQueueSize) {
          scans_.pop_front();
        }
      }
    }
  });

  imu_reader_thread_ = std::jthread([&](std::stop_token stop_token) {
    auto service_context_ = std::make_unique<grpc::ClientContext>();
    google::protobuf::Empty empty_response;
    auto imu_reader =
        service_stub_->getImu(service_context_.get(), empty_response);
    sensors::IMUData msg;
    imu_reader->Finish();

    while (!stop_token.stop_requested()) {

      if (!imu_reader->Read(&msg)) {
        std::cout << "Unable to read remote imu" << std::endl;
      } else {
        std::lock_guard<std::mutex> lock(g_imu_mutex);
        imu_measurements_.push_back(fromGRPC(msg));

        if (imu_measurements_.size() > g_imuQueueSize) {
          imu_measurements_.pop_front();
        }
      }
    }
  });
}

void SensorsClient::stop() {
  read_thread_.request_stop();
  imu_reader_thread_.request_stop();

  // read_thread_.join();
  // imu_reader_thread_.join();
}
