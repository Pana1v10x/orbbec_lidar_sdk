#pragma once
#include <string>
#include <unordered_map>
#include <stdexcept>
namespace ob_lidar::detail {
// this module is used to hardcode the parameters for lidar
class LidarParameter {
   public:
    LidarParameter();

    virtual ~LidarParameter();
    template <typename T>
    void setParameter(const std::string &key, const T &value) {
        if constexpr (std::is_same_v<T, int>) {
            int_parameters_[key] = value;
        } else if constexpr (std::is_same_v<T, double>) {
            double_parameters_[key] = value;
        } else if constexpr (std::is_same_v<T, std::string>) {
            string_parameters_[key] = value;
        } else {
            throw std::invalid_argument("Unsupported parameter type");
        }
    }
    template <typename T>
    T getParameter(const std::string &key) {
        if constexpr (std::is_same_v<T, int>) {
            return int_parameters_[key];
        } else if constexpr (std::is_same_v<T, double>) {
            return double_parameters_[key];
        } else if constexpr (std::is_same_v<T, std::string>) {
            return string_parameters_[key];
        } else {
            throw std::invalid_argument("Unsupported parameter type");
        }
    }

    bool hasParameter(const std::string &key);

   protected:
    std::unordered_map<std::string, int> int_parameters_;
    std::unordered_map<std::string, double> double_parameters_;
    std::unordered_map<std::string, std::string> string_parameters_;
};

}  // namespace ob_lidar::detail
