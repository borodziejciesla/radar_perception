#ifndef EXAMPLE_INCLUDE_CSV_READER_HPP_
#define EXAMPLE_INCLUDE_CSV_READER_HPP_

#include <fstream>
#include <string>
#include <vector>

class CsvReader
{
    public:
        CsvReader(void) = default;
        CsvReader(const CsvReader & arg) = delete;
        CsvReader(CsvReader && arg) = delete;
        ~CsvReader(void) = default;

        void Open(std::string file_path);
        void Close(void);
        bool IsOpen(void) const;

        const std::vector<std::string> & GetHeader(void) const;
        const std::vector<float> & GetNextLine(void);

    private:
        void ReadDataLine(void);

        std::ifstream file_;
        std::string line_string_;
        std::vector<std::string> header_;
        std::vector<std::string> line_strings_;
        std::vector<float> line_;
};

#endif  //  EXAMPLE_INCLUDE_CSV_READER_HPP_
