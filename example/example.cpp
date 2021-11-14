#include <cstdlib>
#include <string>

//#include "radar_processor.hpp"

#include "sensors_data_reader.hpp"

int main() {
    std::string sensors_file = "/home/maciek/Desktop/sensors.csv";

    SensorsDataReader reader;
    reader.Open(sensors_file);

    auto sensors_dara = reader.GetSensorsData();

    reader.Close();

    return EXIT_SUCCESS;
}