#include <cstdlib>
#include <string>

//#include "radar_processor.hpp"

#include "csv_reader.hpp"

int main() {
    std::string sensors_file = "/home/maciek/Desktop/sensors.csv";

    CsvReader reader;
    reader.Open(sensors_file);

    auto header = reader.GetHeader();
    auto line = reader.GetNextLine();

    reader.Close();

    return EXIT_SUCCESS;
}