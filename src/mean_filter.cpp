#include "mean_filter.h"

namespace DX11_NOA {
MeanFilter::MeanFilter(const uint8_t window_size) : window_size_(window_size) {
    window_size_ = std::max(window_size_, static_cast<uint8_t>(1));
    sum_ = 0.0;

    initialized_ = true;
}

double MeanFilter::GetMax() {
    if (max_deque_.empty()) {
        return std::numeric_limits<double>::infinity();
    } else {
        return max_deque_.front().first;
    }
}

double MeanFilter::GetMin() {
    if (min_deque_.empty()) {
        return -std::numeric_limits<double>::infinity();
    } else {
        return min_deque_.front().first;
    }
}

void MeanFilter::MaintainSequence(const double input) {
    if (value_seq_.size() == window_size_) {
        PopValue();
    }

    PushValue(input);
}

void MeanFilter::PopValue() {
    sum_ -= value_seq_.front();
    value_seq_.pop_front();

    if (max_deque_.front().second > 0) {
        max_deque_.front().second--;
    } else {
        max_deque_.pop_front();
    }

    if (min_deque_.front().second > 0) {
        min_deque_.front().second--;
    } else {
        min_deque_.pop_front();
    }
}

void MeanFilter::PushValue(const double input) {
    value_seq_.push_back(input);
    sum_ += input;

    int count = 0;

    while (!min_deque_.empty() && min_deque_.back().first > input) {
        count += (min_deque_.back().second + 1);
        min_deque_.pop_back();
    }
    min_deque_.emplace_back(input, count);

    count = 0;

    while (!max_deque_.empty() && max_deque_.back().first < input) {
        count += (max_deque_.back().second + 1);
        max_deque_.pop_back();
    }
    max_deque_.emplace_back(input, count);
}

double MeanFilter::Update(const double input) {
    MaintainSequence(input);

    return sum_ / static_cast<double>(value_seq_.size());
}

double MeanFilter::UpdateRemoveExtremes(const double input) {
    MaintainSequence(input);
    if (value_seq_.size() > 2) {
        return (sum_ - GetMin() - GetMax()) / static_cast<double>(value_seq_.size() - 2);
    } else {
        return sum_ / static_cast<double>(value_seq_.size());
    }
}

bool MeanFilter::FilterList(const std::vector<double>& input_list, const bool remove_extreme,
                            std::vector<double>& output_list) {
    if(true == input_list.empty()){
        return false;
    }
    output_list.clear();
    output_list.resize(input_list.size(), 0.0);
    for (uint8_t i = 0; i < input_list.size(); ++i) {
        output_list[i] = FilterListStep(input_list, remove_extreme, i);
    }
    return true;
}

double MeanFilter::FilterListStep(const std::vector<double>& input_list, const bool remove_extreme, const uint8_t i) {
    uint8_t mid_index = window_size_ / 2;
    uint8_t index_start = 0, index_end = 0;
    if (i < mid_index) {
        index_start = i;
        index_end = static_cast<uint8_t>(i + window_size_ - mid_index - 1);
    } else {
        index_start = static_cast<uint8_t>(i - mid_index);
        index_end = static_cast<uint8_t>(index_start + window_size_ - 1);
    }
    index_start = std::max(static_cast<uint8_t>(0), std::min(index_start, static_cast<uint8_t>(input_list.size() - 1)));
    index_end = std::max(index_start, std::min(index_end, static_cast<uint8_t>(input_list.size() - 1)));

    if(index_end <= index_start){
        return 0.0;
    }
    double sum = 0.0;
    double max_val = input_list[index_start];
    double min_val = max_val;
    for (size_t i = index_start; i <= index_end; ++i) {
        sum += input_list[i];
        if (max_val < input_list[i]) {
            max_val = input_list[i];
        }
        if (min_val > input_list[i]) {
            min_val = input_list[i];
        }
    }
    double total_length = static_cast<double>(index_end - index_start + 1);
    if (remove_extreme && total_length > 2.0) {
        sum -= (max_val + min_val);
        total_length -= 2.0;
    }
    return sum / total_length;
}
}  // namespace DX11_NOA