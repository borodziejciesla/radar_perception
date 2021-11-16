#ifndef EXAMPLE_INCLUDE_DETECTIONS_READER_HPP_
#define EXAMPLE_INCLUDE_DETECTIONS_READER_HPP_

#include <map>

#include "detection.hpp"

#include "csv_reader.hpp"

class DetectionsReader
{
    public:
        DetectionsReader(void) = default;
        DetectionsReader(const DetectionsReader & arg) = delete;
        DetectionsReader(DetectionsReader && arg) = delete;
        ~DetectionsReader(void) = default;

        void Open(const std::string file_path);
        void Close(void);

        const std::map<size_t, Scan> & GetDetectionsByIndex(const size_t index);
        size_t GetNumberOfIndex(void) const;
    
    private:
        Detection ConvertLineToDetection(const std::vector<float> & line_data);

        CsvReader csv_reader_;
        std::map<size_t, std::vector<Scan>> scans_data_;
        std::map<size_t, Scan> data_for_index_;
        size_t max_index_ = 0u;
};

#endif //   EXAMPLE_INCLUDE_DETECTIONS_READER_HPP_
