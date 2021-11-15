#include <cstdlib>
#include <cmath>
#include <string>

#include "radar_processor.hpp"

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
    

    measurements::radar::ProcessorCalibration calibration;

    calibration.dealiaser_calibration.dealiaser_threshold = 1.0f;

    calibration.segmentator_calibration.neighbourhood_threshold = 3.0f;
    calibration.segmentator_calibration.minimum_detection_in_segment = 2u;

    calibration.velocity_estimator_calibration.maximum_iterations_number = 5u;
    calibration.velocity_estimator_calibration.inlier_threshold = 0.5f;

    measurements::radar::RadarProcessor radar_processor(calibration);


    for (auto index = 1u; index <= detection_reader.GetNumberOfIndex(); index++) {
        auto detections_data = detection_reader.GetDetectionsByIndex(1u);

        for (const auto & scan : detections_data) {
            measurements::radar::RadarScan radar_scan;

            if (scan.first != 1u)
                continue;

            radar_scan.sensor_origin.x = sensors_data.at(0u).sensor_location_x;
            radar_scan.sensor_origin.y = sensors_data.at(0u).sensor_location_y;
            radar_scan.sensor_origin.z = 0.0f;
            radar_scan.sensor_origin.roll = sensors_data.at(0u).roll;
            radar_scan.sensor_origin.pitch = sensors_data.at(0u).pitch;
            radar_scan.sensor_origin.yaw = sensors_data.at(0u).yaw;

            auto idx = 1u;
            for (const auto & detection : scan.second.detections) {
                measurements::radar::RadarDetection radar_detection;

                radar_detection.id = idx++;

                radar_detection.range = detection.range;
                radar_detection.range_std = 0.1f;
                radar_detection.range_rate = detection.range_rate;
                radar_detection.range_rate_std = 0.01f;
                radar_detection.azimuth = detection.azimuth;
                radar_detection.azimuth_std = 0.05f;

                radar_detection.x = detection.range * std::cos(detection.azimuth);
                radar_detection.x_std = std::pow(std::cos(detection.azimuth) * radar_detection.range_std, 2.0f)
                    + std::pow(std::sin(detection.azimuth) * radar_detection.range_std * radar_detection.range, 2.0f);
                radar_detection.y = detection.range * std::sin(detection.azimuth);
                radar_detection.y_std = std::pow(std::sin(detection.azimuth) * radar_detection.range_std, 2.0f)
                    + std::pow(std::cos(detection.azimuth) * radar_detection.range_std * radar_detection.range, 2.0f);
                radar_detection.y = detection.range * std::sin(detection.azimuth);
                radar_detection.z = 0.0f;

                radar_detection.dealiasing_status = measurements::radar::DealiasingStatus::MovingObjectDealiased;

                radar_scan.detections.push_back(radar_detection);
            }

            auto output = radar_processor.ProcessScan(radar_scan);

            auto j = 77;
        }
    }

    detection_reader.Close();

    return EXIT_SUCCESS;
}