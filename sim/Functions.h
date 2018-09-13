#ifndef __MSS_FUNCTIONS_H__
#define __MSS_FUNCTIONS_H__
#include "dart/dart.hpp"

namespace MSS
{

// Utilities
std::vector<double> split_to_double(const std::string& input, int num);
Eigen::Vector3d string_to_vector3d(const std::string& input);
Eigen::VectorXd string_to_vectorXd(const std::string& input, int n);
Eigen::Matrix3d string_to_matrix3d(const std::string& input);

}

#endif