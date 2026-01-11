//
// Created by ch on 2025/9/25.
//

#pragma once

#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace rm2_common
{
class LinearInterp
{
public:
  explicit LinearInterp(rclcpp::Node::SharedPtr node) : node_(node) {}
  void init(std::vector<double> input_vector, std::vector<double> output_vector)
  {
    if (input_vector.size() != output_vector.size())
    {
      RCLCPP_ERROR(node_->get_logger(), "Linear interpolation ERROR: The sizes of input_vector and output_vector are not equal.");
    }
    for (int i = 0; i < (int)input_vector.size(); i++)
    {
      if (!input_vector_.empty())
        if (input_vector[i] < input_vector_.back())
        {
          RCLCPP_ERROR(node_->get_logger(),"Linear interpolation ERROR: Please sort the point's abscissa from smallest to largest. %lf < %lf",
            input_vector[i], input_vector_.back());
          return;
        }
      input_vector_.push_back(input_vector[i]);
      output_vector_.push_back(output_vector[i]);
    }
  }
  double output(double input)
  {
    if (input >= input_vector_.back())
      return output_vector_.back();
    else if (input <= input_vector_.front())
      return output_vector_.front();
    for (size_t i = 0; i < input_vector_.size(); i++)
    {
      if (input >= input_vector_[i] && input <= input_vector_[i + 1])
        return output_vector_[i] +
               ((output_vector_[i + 1] - output_vector_[i]) / (input_vector_[i + 1] - input_vector_[i])) *
                   (input - input_vector_[i]);
    }
    RCLCPP_ERROR(node_->get_logger(),"The point's abscissa aren't sorted from smallest to largest.");
    return 0;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::vector<double> input_vector_;
  std::vector<double> output_vector_;
};
}  // namespace rm2_common
