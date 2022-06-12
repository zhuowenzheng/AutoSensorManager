#include <iostream>
#include <string>
using namespace std;

/******************************************************************************
 * 创建一个完整的C++应用程序，用于对智能车所安装的传感器参数进行管理。
 * 用循环语句构建程序主菜单框架，通过输入菜单项标识符（命令编号或菜单文本中的首字符）执行菜单项所关联的功能。
 * 设计并实现传感器模板类Sensor 、传感器参数基类SensorParameter 、
 * 传感器参数派生类CameraParameter、LidarParameter、RadarParameter 、
 * 特化传感器类Camera Lidar Radar 以及传感器管理类SensorManager 。编制一个完整的C++应用程序。
 *
 * @affiliation 同济大学
 * @author 1951710 郑焯文
 * @date Jun 2022
 * @version 1.0
 ******************************************************************************/



// 传感器模板类
template <class Sensor>

// 传感器参数基类
// 数据成员包括id、名称、位置（3维坐标x,y,z），姿态角（roll, pitch, yaw）
class SensorParameter
{
public:
    SensorParameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw)
    {
        this->id = id;
        this->name = name;
        this->x = x;
        this->y = y;
        this->z = z;
        this->roll = roll;
        this->pitch = pitch;
        this->yaw = yaw;
    }
    int id;
    string name;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

// class CameraParameter
// 继承传感器参数基类，分辨率（integer, integer），帧率（integer），视场角(integer: 0-180)，畸变参数（float point[5]），色彩位数（integer）
class CameraParameter : public SensorParameter
{
public:
    CameraParameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw, int resolution_x, int resolution_y, int frame_rate, int view_angle, float distortion_parameter[5], int color_bits) : SensorParameter(id, name, x, y, z, roll, pitch, yaw)
    {
        this->resolution_x = resolution_x;
        this->resolution_y = resolution_y;
        this->frame_rate = frame_rate;
        this->view_angle = view_angle;
        for (int i = 0; i < 5; i++)
        {
            this->distortion_parameter[i] = distortion_parameter[i];
        }
        this->color_bits = color_bits;
    }
    int resolution_x;
    int resolution_y;
    int frame_rate;
    int view_angle;
    float distortion_parameter[5];
    int color_bits;
};

// class LidarParameter
// 继承SensorParameter，参数：线数（integer），视场角（integer: 0-45），旋转频率（integer），水平视场角（integer：0-360）
class LidarParameter : public SensorParameter
{
public:
    LidarParameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw, int line_num, int view_angle, int rotate_rate, int horizontal_view_angle) : SensorParameter(id, name, x, y, z, roll, pitch, yaw)
    {
        this->line_num = line_num;
        this->view_angle = view_angle;
        this->rotate_rate = rotate_rate;
        this->horizontal_view_angle = horizontal_view_angle;
    }
    int line_num;
    int view_angle;
    int rotate_rate;
    int horizontal_view_angle;
};

// class RadarParameter
// 继承SensorParameter，参数：分辨率（float point），视场角（integer：0-60），速度精度（float point），探测模式（string）
class RadarParameter : public SensorParameter
{
public:
    RadarParameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw, float resolution[2], int view_angle, float speed_accuracy[2], string detect_mode) : SensorParameter(id, name, x, y, z, roll, pitch, yaw)
    {
        this->resolution[0] = resolution[0];
        this->resolution[1] = resolution[1];
        this->view_angle = view_angle;
        this->speed_accuracy[0] = speed_accuracy[0];
        this->speed_accuracy[1] = speed_accuracy[1];
        this->detect_mode = detect_mode;
    }
    float resolution[2];
    int view_angle;
    float speed_accuracy[2];
    string detect_mode;
};
// 特化传感器类 Camera Lidar Radar

// 传感器管理类 SensorManager


int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
