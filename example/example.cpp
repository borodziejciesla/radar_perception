#include <cstdlib>
#include <string>

//#include "radar_processor.hpp"

#include "detections_reader.hpp"
#include "sensors_data_reader.hpp"

int main() {
    std::string sensors_file = "/home/maciek/Desktop/sensors.csv";
    std::string detections_file = "/home/maciek/Desktop/detections.csv";

    SensorsDataReader reader;
    reader.Open(sensors_file);
    auto sensors_data = reader.GetSensorsData();
    reader.Close();

    DetectionsReader detection_reader;
    detection_reader.Open(detections_file);
    auto detections_data = detection_reader.GetDetectionsByIndex(1u);
    detection_reader.Close();

    return EXIT_SUCCESS;
}