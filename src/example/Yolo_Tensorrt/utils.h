
#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <numeric>

float computeMean(const std::vector<uint16_t>& values) {
    if (values.empty()) {
        throw std::domain_error("Mean of an empty vector is undefined");
    }
    
    // Compute the sum of the elements
    uint64_t sum = std::accumulate(values.begin(), values.end(), uint64_t(0));
    
    // Compute the mean
    return static_cast<float>(sum) / values.size();
}

template <typename T>
T computeMedian(std::vector<T> &vec) {
    size_t size = vec.size();
    if (size == 0) {
        throw std::domain_error("Median of an empty vector is undefined");
    }
    size_t mid = size / 2;
    std::nth_element(vec.begin(), vec.begin() + mid, vec.end());
    if (size % 2 == 0) {
        T mid1 = vec[mid];
        std::nth_element(vec.begin(), vec.begin() + mid - 1, vec.end());
        T mid2 = vec[mid - 1];
        return (mid1 + mid2) / 2;
    } else {
        return vec[mid];
    }
}

// Template to filter values in a vector based on a threshold
template <typename I16, typename F>
void filter_depth_threshold(std::vector<I16> &values, I16 median, F threshold) {
    values.erase(std::remove_if(values.begin(), values.end(), [median, threshold](I16 value) {
        return std::abs(static_cast<I16>(value) - static_cast<I16>(median)) > threshold;
    }), values.end());
}

#endif // UTILS_H
