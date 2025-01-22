add_library(Timing timing.cc)
target_include_directories(Timing PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../include)


add_library(msensor::Timing ALIAS Timing)