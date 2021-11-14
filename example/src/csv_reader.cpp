#include "csv_reader.hpp"

void CsvReader::Open(std::string file_path) {
    file_.open(file_path, std::ios::out);

    if (IsOpen()) {
        getline(file_, line_string_);
        ReadDataLine();
        header_ = line_strings_;
    }
}

void CsvReader::Close(void) {
    file_.close();
}

bool CsvReader::IsOpen(void) const {
    return file_.is_open();
}

const std::vector<std::string> & CsvReader::GetHeader(void) const {
    return header_;
}

const std::vector<float> & CsvReader::GetNextLine(void) {
    line_.clear();
    
    if (IsOpen()) {
        if (getline(file_, line_string_)) {
            ReadDataLine();

            for (const auto & element : line_strings_)
                line_.push_back(std::atof(element.c_str()));
        } else {
            file_.close();
        }
    }

    return line_;
}

void CsvReader::ReadDataLine(void) {
    line_strings_.clear();

    auto element_start = line_string_.begin();
    auto element_end = line_string_.begin();
    size_t pos = 0u;

    while (element_end != line_string_.end()) {
        if (auto index = line_string_.find(",", pos); index != std::string::npos) {
            element_end = element_start + (index - pos);
            auto len = element_end - element_start;
            auto element = line_string_.substr(pos, len);
            line_strings_.push_back(element);
            pos += len + 1u;
            element_start = element_end + 1;
        } else if (auto index = line_string_.find("\r", pos); index != std::string::npos) {
            element_end = element_start + (index - pos);
            auto len = element_end - element_start;
            auto element = line_string_.substr(pos, len);
            line_strings_.push_back(element);
            break;
        } else {
            break;
        }
    }
}
