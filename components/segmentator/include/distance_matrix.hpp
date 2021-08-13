/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_SEGMENTATOR_SRC_DISTANCE_MATRIX_HPP_
#define COMPONENTS_SEGMENTATOR_SRC_DISTANCE_MATRIX_HPP_

#include <vector>

#include "distance_matrix.hpp"

namespace measurements::radar
{
    class DistanceMatrix
    {
        public:
            DistanceMatrix(void);
            ~DistanceMatrix(void);
            
            void SetSize(size_t matrix_size);
            float & operator()(size_t row, size_t column);
            
        private:
            size_t matrix_size_ = 0u;
            std::vector<std::vector<float>> matrix_;
    };
}   // namespace measurements::radar

#endif // COMPONENTS_SEGMENTATOR_SRC_DISTANCE_MATRIX_HPP_
