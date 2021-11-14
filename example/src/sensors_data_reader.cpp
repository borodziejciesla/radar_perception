#include "sensors_data_reader.hpp"

void SensorsDataReader::Open(const std::string file_path) {
    csv_reader_.Open(file_path);

    sensors_data_.clear();

    auto read_line = csv_reader_.GetNextLine();
    while (!read_line.empty()) {
        sensors_data_.push_back(ConvertLineToSensorData(read_line));
        read_line = csv_reader_.GetNextLine();
    }
}

void SensorsDataReader::Close(void) {
    csv_reader_.Close();
}

const std::vector<SensorData> & SensorsDataReader::GetSensorsData(void) {
    return sensors_data_;
}

SensorData SensorsDataReader::ConvertLineToSensorData(const std::vector<float> & line_data) {
    SensorData sensor_data;

    sensor_data.sensor_index = static_cast<size_t>(line_data.at(0u));
    sensor_data.update_interval = line_data.at(1u);
    sensor_data.sensor_location_x = line_data.at(2u);
    sensor_data.sensor_location_y = line_data.at(3u);
    sensor_data.yaw = line_data.at(4u);
    sensor_data.pitch = line_data.at(5u);
    sensor_data.roll = line_data.at(6u);
    sensor_data.detection_probability = line_data.at(7u);
    sensor_data.azimuth_resolution = line_data.at(8u);
    sensor_data.range_resolution = line_data.at(9u);
    sensor_data.range_rate_resolution = line_data.at(10u);
    sensor_data.max_num_detections = static_cast<size_t>(line_data.at(11u));

    return sensor_data;
}
