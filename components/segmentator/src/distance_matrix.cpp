/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "distance_matrix.hpp"

#include <algorithm>
#include <stdexcept>

namespace measurements::radar
{
    DistanceMatrix::DistanceMatrix(void) {
    }

    DistanceMatrix::~DistanceMatrix(void) {
        for (auto idx = 0u; idx < matrix_size_; idx++)
            matrix_.at(idx).resize(idx + 1);
    }

    void DistanceMatrix::SetSize(size_t matrix_size) {
        matrix_size_ = matrix_size;
        
        matrix_.resize(matrix_size);
        for (auto idx = 0u; idx < matrix_size; idx++)
            matrix_.at(idx).resize(idx + 1);
    }

    float & DistanceMatrix::operator()(size_t row, size_t column) {
        if ((row >= matrix_size_) || (column >= matrix_size_))
            throw std::invalid_argument("DistanceMatrix::operator() - invalid matrix index!");

        auto first_index = std::max(row, column);
        auto second_index = std::min(row, column);

        return matrix_.at(first_index).at(second_index);
    }
}   //  namespace measurements::radar
