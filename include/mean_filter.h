#pragma once

#include <algorithm>
#include <deque>
#include <limits>
#include <utility>
#include <vector>
#include <iostream>
#include <iomanip>


#include "CommonDataType.h"

namespace DX11_NOA {
    class MeanFilter {
 public:
  
  explicit MeanFilter(const uint8_t window_size);

  
  ~MeanFilter() = default;

  void set_window_size(const uint8_t window_size) {
    window_size_ = std::max(window_size, static_cast<uint8_t>(1));
  }

  
  double Update(const double input);

  
  double UpdateRemoveExtremes(const double input);

  
  double GetMax();

  
  double GetMin();

  bool FilterList(const std::vector<double>& input_list,
                  const bool remove_extreme, std::vector<double>& output_list);

  double FilterListStep(const std::vector<double>& input_list,
                        const bool remove_extreme, const uint8_t i);

 private:
  
  bool initialized_;

  
  uint8_t window_size_;

  
  std::deque<double> value_seq_;

  
  std::deque<std::pair<double, int>> min_deque_;
  
  std::deque<std::pair<double, int>> max_deque_;

  
  double sum_;

  void MaintainSequence(const double input);
  void PopValue();
  void PushValue(const double input);
};

}  // namespace DX11_NOA
