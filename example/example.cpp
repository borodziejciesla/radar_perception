#include <cstdlib>
#include <cmath>
#include <string>

#include "matplotlibcpp.h"

#include "radar_processor.hpp"

#include "detections_reader.hpp"
#include "sensors_data_reader.hpp"

namespace plt = matplotlibcpp;

/* Helper functions Declaration */
static void PlotScan(const size_t idx, const size_t radar_index, const measurements::radar::RadarScan & radar_scan, const measurements::radar::RadarProcessor::ProcessingOutput & output);
static measurements::radar::RadarDetection ConvertDetection(const Detection & detection_raw, const size_t id);
static measurements::radar::SensorOrigin ConvertSensorOrigin(const SensorData & sensor_data);

/* Main function */
int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cout << "Invalid Number of arguments!";
        return EXIT_FAILURE;
    }

    const std::string sensors_file(argv[1]);
    const std::string detections_file(argv[2]);

    /* Open logs */
    SensorsDataReader reader;
    reader.Open(sensors_file);
    auto sensors_data = reader.GetSensorsData();
    reader.Close();

    DetectionsReader detection_reader;
    detection_reader.Open(detections_file);
    
    /* Prepare processor */
    measurements::radar::ProcessorCalibration calibration;

    calibration.dealiaser_calibration.dealiaser_threshold = 1.0f;

    calibration.segmentator_calibration.neighbourhood_threshold = 3.0f;
    calibration.segmentator_calibration.minimum_detection_in_segment = 2u;

    calibration.velocity_estimator_calibration.maximum_iterations_number = 10u;
    calibration.velocity_estimator_calibration.inlier_threshold = 0.75f;

    measurements::radar::RadarProcessor radar_processor(calibration);

    /* Run Processing */
    for (auto index = 1u; index <= detection_reader.GetNumberOfIndex(); index++) {
        auto detections_data = detection_reader.GetDetectionsByIndex(index);

        for (const auto & scan : detections_data) {
            measurements::radar::RadarScan radar_scan;

            radar_scan.sensor_origin = ConvertSensorOrigin(sensors_data.at(scan.first - 1u));

            auto idx = 1u;
            for (const auto & detection : scan.second.detections)
                radar_scan.detections.push_back(ConvertDetection(detection, idx++));

            auto output = radar_processor.ProcessScan(radar_scan);

            PlotScan(index, scan.first, radar_scan, output);
        }
    }

    detection_reader.Close();

    return EXIT_SUCCESS;
}

/* Helper functions definitions */
static void PlotScan(const size_t idx, const size_t radar_index, const measurements::radar::RadarScan & radar_scan, const measurements::radar::RadarProcessor::ProcessingOutput & output) {
    plt::figure();
    
    /* Detections */
    std::vector<double> x;
    std::vector<double> y;

    for (const auto & detection : radar_scan.detections) {
        x.push_back(static_cast<double>(detection.x));
        y.push_back(static_cast<double>(detection.y));
    }

    plt::plot(x, y, "b.", {{"label", "Detections"}});

    /* Output */
    if (output.has_value()) {
        /* Moving */
        auto objects = std::get<0>(output.value());

        for (const auto & object : objects) {
            std::vector<double> x_obj;
            std::vector<double> y_obj;

            auto c = std::cos(object.object_center.orientation);
            auto s = std::sin(object.object_center.orientation);

            // FL
            x_obj.push_back(static_cast<double>(0.5 * object.object_size.length * c - 0.5 * object.object_size.width * s) + object.object_center.x);
            y_obj.push_back(static_cast<double>(0.5 * object.object_size.length * s + 0.5 * object.object_size.width * c) + object.object_center.y);
            // FR
            x_obj.push_back(static_cast<double>(0.5 * object.object_size.length * c + 0.5 * object.object_size.width * s) + object.object_center.x);
            y_obj.push_back(static_cast<double>(0.5 * object.object_size.length * s - 0.5 * object.object_size.width * c) + object.object_center.y);
            // RR
            x_obj.push_back(static_cast<double>(0.5 * -object.object_size.length * c + 0.5 * object.object_size.width * s) + object.object_center.x);
            y_obj.push_back(static_cast<double>(0.5 * -object.object_size.length * s - 0.5 * object.object_size.width * c) + object.object_center.y);
            // RL
            x_obj.push_back(static_cast<double>(0.5 * -object.object_size.length * c - 0.5 * object.object_size.width * s) + object.object_center.x);
            y_obj.push_back(static_cast<double>(0.5 * -object.object_size.length * s + 0.5 * object.object_size.width * c) + object.object_center.y);
            // FL
            x_obj.push_back(static_cast<double>(0.5 * object.object_size.length * c - 0.5 * object.object_size.width * s) + object.object_center.x);
            y_obj.push_back(static_cast<double>(0.5 * object.object_size.length * s + 0.5 * object.object_size.width * c) + object.object_center.y);

            plt::plot(x_obj, y_obj, {{"label", "Object"}});
        }

        /* Static */
        auto static_objects = std::get<1>(output.value());

        for (const auto & object : static_objects) {
            std::vector<double> x_obj;
            std::vector<double> y_obj;

            const auto range_delta = object.range.end - object.range.start;
            for (auto index = 0u; index < 100u; index++) {
                const auto r = object.range.start + static_cast<float>(index) * range_delta;
                const auto y = std::pow(r, 3.0f) * object.polynomial.a3
                    + std::pow(r, 2.0f) * object.polynomial.a2
                    + r * object.polynomial.a1
                    + object.polynomial.a0;

                x_obj.push_back(static_cast<double>(r));
                y_obj.push_back(static_cast<double>(y));
            }

            //plt::plot(x_obj, y_obj, {{"label", "Guardrail"}});
        }
    }
    
    /* Plot */
    plt::grid();
    plt::legend();
    plt::xlabel("x [m]");
    plt::ylabel("y [m]");
    plt::axis("equal");
    plt::savefig(std::to_string(radar_index) + "_" + std::to_string(idx) + ".jpg");
    //plt::show();
}

static measurements::radar::RadarDetection ConvertDetection(const Detection & detection_raw, const size_t id) {
    measurements::radar::RadarDetection radar_detection;

    radar_detection.id = id;

    radar_detection.range = detection_raw.range;
    radar_detection.range_std = 0.1f;
    radar_detection.range_rate = detection_raw.range_rate;
    radar_detection.range_rate_std = 0.01f;
    radar_detection.azimuth = detection_raw.azimuth;
    radar_detection.azimuth_std = 0.05f;

    radar_detection.x = detection_raw.range * std::cos(detection_raw.azimuth);
    radar_detection.x_std = std::pow(std::cos(detection_raw.azimuth) * radar_detection.range_std, 2.0f)
        + std::pow(std::sin(detection_raw.azimuth) * radar_detection.range_std * radar_detection.range, 2.0f);
    radar_detection.y = detection_raw.range * std::sin(detection_raw.azimuth);
    radar_detection.y_std = std::pow(std::sin(detection_raw.azimuth) * radar_detection.range_std, 2.0f)
        + std::pow(std::cos(detection_raw.azimuth) * radar_detection.range_std * radar_detection.range, 2.0f);
    radar_detection.y = detection_raw.range * std::sin(detection_raw.azimuth);
    radar_detection.z = 0.0f;

    radar_detection.dealiasing_status = measurements::radar::DealiasingStatus::MovingObjectDealiased;

    return radar_detection;
}

static measurements::radar::SensorOrigin ConvertSensorOrigin(const SensorData & sensor_data) {
    measurements::radar::SensorOrigin sensor_origin;

    sensor_origin.x = sensor_data.sensor_location_x;
    sensor_origin.y = sensor_data.sensor_location_y;
    sensor_origin.z = 0.0f;
    sensor_origin.roll = sensor_data.roll;
    sensor_origin.pitch = sensor_data.pitch;
    sensor_origin.yaw = sensor_data.yaw;

    return sensor_origin;
}
