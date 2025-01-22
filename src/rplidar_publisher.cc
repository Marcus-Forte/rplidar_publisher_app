#include "file/File.hh"
#include "imu/icm-20948.h"
#include "imu/icm-20948_defs.h"
#include "lidar/RPLidar.hh"
#include "recorder/ScanRecorder.hh"
#include "sensors_server.hh"
#include "timing/timing.hh"
#include <filesystem>
#include <future>
#include <getopt.h>
#include <iostream>
#include <mutex>
#include <thread>
std::mutex g_mutex;

// 100 Hz
void ImuLoop(SensorsServer &server) {
  std::cout << "Start imu loop" << std::endl;
  ICM20948 icm20948(1, ICM20948_ADDR0);
  icm20948.init();

  icm20948.calibrate();
  while (true) {

    auto acc_data = icm20948.get_acc_data();
    auto gyr_data = icm20948.get_gyro_data();
    auto dbl_acc_data = icm20948.convert_raw_data(acc_data, FACTOR_ACC_2G);
    auto dbl_gyr_data =
        icm20948.convert_raw_data(gyr_data, FACTOR_GYRO_500DPS_RADS);
    auto timestamp = timing::getNowUs();
    msensor::IMUData data;
    data.timestamp = timestamp;
    data.ax = static_cast<float>(dbl_acc_data.x);
    data.ay = static_cast<float>(dbl_acc_data.y);
    data.az = static_cast<float>(dbl_acc_data.z);
    data.gx = static_cast<float>(dbl_gyr_data.x);
    data.gy = static_cast<float>(dbl_gyr_data.y);
    data.gz = static_cast<float>(dbl_gyr_data.z);
    {
      std::lock_guard<std::mutex> lock(g_mutex);
      server.publishImu(data);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
void print_usage() {
  std::cout << "Usage: rplidar_publisher [serial device path]  [-r record] "
            << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 2) {
    print_usage();
    exit(0);
  }

  std::unique_ptr<msensor::ILidar> lidar;
  if (!std::filesystem::exists(argv[1])) {
    std::cerr << "Device: " << argv[1] << " does not exist. Exiting..."
              << std::endl;
    exit(-1);
  }
  lidar = std::make_unique<msensor::RPLidar>(argv[1]);
  dynamic_cast<msensor::RPLidar *>(lidar.get())->setMotorRPM(360);

  auto file = std::make_shared<File>();
  msensor::ScanRecorder recorder(file);

  bool record_scans = false;
  int opt;
  while ((opt = getopt(argc, argv, "r")) != -1) {
    switch (opt) {
    case 'r':
      record_scans = true;
      std::cout << "Recording scan enabled" << std::endl;
      recorder.start();
      break;
    }
  }

  lidar->init();

  SensorsServer server;
  server.start();

  auto imu_loop =
      std::async(std::launch::async, [&server]() { ImuLoop(server); });

  while (true) {
    const auto scan = lidar->getScan();
    std::cout << "Scans pts: " << scan.points.size() << std::endl;
    {
      std::lock_guard<std::mutex> lock(g_mutex);
      server.publishScan(scan);
    }
    recorder.record(scan);
  }

  server.stop();
}