#include <gtest/gtest.h>
#if __GNUC__ >= 9
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

#include <nlohmann/json.hpp>
#include <toml++/toml.hpp>

#include "orbbec_lidar/orbbec_lidar.hpp"

namespace ob = ob_lidar;

#if __GNUC__ >= 9
namespace fs = std::filesystem;
#else
namespace fs = std::experimental::filesystem;
#endif

TEST(Device, empty)
{
}
