#include "file/File.hh"
#include "lidar/simLidar.hh"
#include "recorder/ScanRecorder.hh"
#include "sensors_server.hh"
#include <getopt.h>
#include <iostream>

void print_usage() {
  std::cout << "Usage: sim_publisher [-r record]" << std::endl;
}

int main(int argc, char **argv) {

  auto simLidar = std::make_unique<msensor::SimLidar>();

  auto file = std::make_shared<File>();
  msensor::ScanRecorder recorder(file);

  bool record_scans = false;
  int opt;
  while ((opt = getopt(argc, argv, "rh")) != -1) {
    switch (opt) {
    case 'h':
      print_usage();
      exit(0);
    case 'r':
      record_scans = true;
      break;
    }
  }

  if (record_scans) {
    std::cout << "Recording scan enabled" << std::endl;
    recorder.start();
  }

  SensorsServer server;
  server.start();

  while (true) {
    const auto scan = simLidar->getScan();
    std::cout << "Scans pts: " << scan.points.size() << std::endl;
    server.publishScan(scan);
    recorder.record(scan);
  }

  server.stop();
}