#include "detections_reader.hpp"

#include <algorithm>

void DetectionsReader::Open(const std::string file_path) {
    csv_reader_.Open(file_path);

    auto read_line = csv_reader_.GetNextLine();
    while (!read_line.empty()) {
        auto sensor_index = read_line.at(2u);
        auto scan_index = static_cast<size_t>(read_line.at(0u));
        auto time_stamp = static_cast<double>(read_line.at(1u));

        max_index_ = std::max(max_index_, scan_index);

        if (scans_data_.find(sensor_index) == scans_data_.end()) {
            scans_data_[sensor_index] = std::vector<Scan>();
            Scan scan;
            scan.scan_index = scan_index;
            scan.time_stamp = time_stamp;
            scans_data_[sensor_index].push_back(scan);
        } else if (scans_data_[sensor_index].back().scan_index != scan_index) {
            Scan scan;
            scan.scan_index = scan_index;
            scan.time_stamp = time_stamp;
            scans_data_[sensor_index].push_back(scan);
        }

        scans_data_[sensor_index].back().detections.push_back(ConvertLineToDetection(read_line));
        read_line = csv_reader_.GetNextLine();
    }
}

void DetectionsReader::Close(void) {
    csv_reader_.Close();
}

const std::map<size_t, Scan> & DetectionsReader::GetDetectionsByIndex(const size_t index) {
    data_for_index_.clear();

    if (index < max_index_) {
        for (const auto & element : scans_data_)
            data_for_index_[element.first] = element.second.at(index - 1u);
    }

    return data_for_index_;
}

size_t DetectionsReader::GetNumberOfIndex(void) const {
    return max_index_;
}

Detection DetectionsReader::ConvertLineToDetection(const std::vector<float> & line_data) {
    Detection detection;

    detection.range = line_data.at(3u);
    detection.azimuth = line_data.at(4u);
    detection.range_rate = line_data.at(5u);
    detection.object_id = static_cast<size_t>(line_data.at(6u));

    return detection;
}
