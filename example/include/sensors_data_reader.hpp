#ifndef EXAMPLE_INCLUDE_SENSOR_DATA_READER_HPP_
#define EXAMPLE_INCLUDE_SENSOR_DATA_READER_HPP_

#include "sensor_data.hpp"

#include "csv_reader.hpp"

class SensorsDataReader
{
    public:
        SensorsDataReader(void) = default;
        SensorsDataReader(const SensorsDataReader & arg) = delete;
        SensorsDataReader(SensorsDataReader && arg) = delete;
        ~SensorsDataReader(void) = default;

        void Open(const std::string file_path);
        void Close(void);

        const std::vector<SensorData> & GetSensorsData(void);
    
    private:
        SensorData ConvertLineToSensorData(const std::vector<float> & line_data);

        CsvReader csv_reader_;
        std::vector<SensorData> sensors_data_;
};

#endif //   EXAMPLE_INCLUDE_SENSOR_DATA_READER_HPP_
