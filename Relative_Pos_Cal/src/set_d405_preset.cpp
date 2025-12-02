#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv)
{
  std::string json_path;

  if (argc >= 2)
  {
    json_path = argv[1];
  }
  else
  {
    // 默认路径你自己改，比如放在 /home/orangepi/d405_preset.json
    json_path = "/home/orangepi/catkin_ws/src/Relative_Pos_Cal/src/1.json";
  }

  try
  {
    // 1. 查找设备
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0)
    {
      std::cerr << "No RealSense device found!" << std::endl;
      return 1;
    }

    rs2::device dev = devices.front();
    std::cout << "Using device: "
              << dev.get_info(RS2_CAMERA_INFO_NAME)
              << " (SN: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
              << ")" << std::endl;

    // 2. 检查并打开 advanced mode
    if (!dev.is<rs400::advanced_mode>())
    {
      std::cerr << "Device does not support advanced mode!" << std::endl;
      return 1;
    }

    rs400::advanced_mode adv = dev.as<rs400::advanced_mode>();

    // 3. 读取 JSON 文件
    std::ifstream file(json_path);
    if (!file.good())
    {
      std::cerr << "Failed to open JSON preset file: " << json_path << std::endl;
      return 1;
    }

    std::string json_content((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());

    // 4. 加载 JSON 到设备
    adv.load_json(json_content);
    std::cout << "Preset loaded from: " << json_path << std::endl;

    // 5. 结束
    return 0;
  }
  catch (const rs2::error &e)
  {
    std::cerr << "RealSense error calling " << e.get_failed_function()
              << "(" << e.get_failed_args() << "): " << e.what() << std::endl;
    return 1;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }
}
