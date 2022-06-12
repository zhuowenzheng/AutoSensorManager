#include <iostream>
#include <string>
#include "Sensor.h"
#include "SensorParameter.h"

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
    //[]操作符重载用于访问数组元素
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
    //[]操作符重载用于访问LidarParameter数组元素
    LidarParameter& operator[](int index)
    {
        return *(this + index);
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
    float resolution[2]{};
    int view_angle;
    float speed_accuracy[2]{};
    string detect_mode;
};

// 特化传感器类 Sensor<CameraParameter>
// 用对应的模板参数定义动态传感器参数的动态数组；
// 对数据的操作应主要包括：添加、显示以及当前标志等数据成员的设置及获取等。
// 其中，添加是将键盘输入的参数构造成对应的传感器参数类对象中；显示是将当前的传感器参数按格式输出到屏幕上。
// 根据需要定义相应的构造函数及其他成员函数；
template <>
class Sensor<CameraParameter>
{
public:
    Sensor(CameraParameter currentParameter) : current_parameter(currentParameter) {
        this->parameter_num = 0;
        this->current_parameter = new CameraParameter[this->parameter_num];
    }
    ~Sensor()
    {
        delete[] this->parameter;
    }
    void add_parameter(CameraParameter cameraParameter)
    {
        cout << "请输入传感器编号：";
        cin >> cameraParameter.id;
        cout << "请输入传感器名称：";
        cin >> cameraParameter.name;
        cout << "请输入传感器的位置（x, y, z）：";
        cin >> cameraParameter.x >> cameraParameter.y >> cameraParameter.z;
        cout << "请输入传感器的旋转角度（roll, pitch, yaw）：";
        cin >> cameraParameter.roll >> cameraParameter.pitch >> cameraParameter.yaw;
        cout << "请输入传感器的分辨率（x, y）：";
        cin >> cameraParameter.resolution_x >> cameraParameter.resolution_y;
        cout << "请输入传感器的帧率：";
        cin >> cameraParameter.frame_rate;
        cout << "请输入传感器的视场角：";
        cin >> cameraParameter.view_angle;
        cout << "请输入传感器的畸变参数（k1, k2, k3, k4, k5）：";
        cin >> cameraParameter.distortion_parameter[0] >> cameraParameter.distortion_parameter[1] >> cameraParameter.distortion_parameter[2] >> cameraParameter.distortion_parameter[3] >> cameraParameter.distortion_parameter[4];
        cout << "请输入传感器的颜色位数：";
        cin >> cameraParameter.color_bits;
    }
    void show_parameter()
    {
        cout << "传感器编号\t传感器名称\t传感器位置\t传感器旋转角度\t传感器分辨率\t传感器帧率\t传感器视场角\t传感器颜色位数" << endl;
        for (int i = 0; i < this->parameter_num; i++)
        {
            cout << this->parameter[i].id << "\t" << this->parameter[i].name << "\t" << this->parameter[i].x << "\t" << this->parameter[i].y << "\t" << this->parameter[i].z << "\t" << this->parameter[i].roll << "\t" << this->parameter[i].pitch << "\t" << this->parameter[i].yaw << "\t" << this->parameter[i].resolution_x << "\t" << this->parameter[i].resolution_y << "\t" << this->parameter[i].frame_rate << "\t" << this->parameter[i].view_angle << "\t" << this->parameter[i].color_bits << endl;
        }
    }
    void set_current_parameter(int id)
    {
        this->current_parameter = this->parameter[id];
    }
    CameraParameter get_current_parameter()
    {
        return this->current_parameter;
    }
    int get_parameter_num() const
    {
        return this->parameter_num;
    }
    CameraParameter *get_parameter()
    {
        return this->parameter;
    }
private:
    int parameter_num;
    CameraParameter *parameter;
    CameraParameter current_parameter;
};


//特化传感器类 Sensor<LidarParameter>
template <>
class Sensor<LidarParameter>
{
public:
    Sensor(LidarParameter currentParameter) : current_parameter(currentParameter) {
        this->parameter_num = 0;
        this->current_parameter = new LidarParameter[this->parameter_num];
    }
    ~Sensor()
    {
        delete[] this->parameter;
    }
    void add_parameter(LidarParameter lidarParameter)
    {
        cout << "请输入传感器编号：";
        cin >> lidarParameter.id;
        cout << "请输入传感器名称：";
        cin >> lidarParameter.name;
        cout << "请输入传感器的位置（x, y, z）：";
        cin >> lidarParameter.x >> lidarParameter.y >> lidarParameter.z;
        cout << "请输入传感器的旋转角度（roll, pitch, yaw）：";
        cin >> lidarParameter.roll >> lidarParameter.pitch >> lidarParameter.yaw;
        cout << "请输入传感器的视场角：";
        cin >> lidarParameter.view_angle;
        cout << "请输入传感器的水平视场角：";
        cin >> lidarParameter.horizontal_view_angle;
        // 导入数组
        this->parameter[this->parameter_num++] = lidarParameter;
    }
    void show_parameter()
    {
        cout << "传感器编号\t传感器名称\t传感器位置\t传感器旋转角度\t传感器线数\t传感器视场角\t传感器旋转频率\t传感器水平视场角" << endl;
        for (int i = 0; i < this->parameter_num; i++)
        {
            cout<<this->parameter[i].id<<"\t"<<this->parameter[i].name<<"\t"<<this->parameter[i].x<<"\t"<<this->parameter[i].y<<"\t"<<this->parameter[i].z<<"\t"<<this->parameter[i].roll<<"\t"<<this->parameter[i].pitch<<"\t"<<this->parameter[i].yaw<<"\t"<<this->parameter[i].view_angle<<"\t"<<this->parameter[i].horizontal_view_angle<<endl;
        }
    }
    void set_current_parameter(int id)
    {
        this->current_parameter = this->parameter[id];
    }
    LidarParameter get_current_parameter()
    {
        return this->current_parameter;
    }
    int get_parameter_num() const
    {
        return this->parameter_num;
    }
    LidarParameter *get_parameter()
    {
        return this->parameter;
    }
private:
    int parameter_num;
    LidarParameter *parameter;
    LidarParameter current_parameter;
};

// Radar
template <>
class Sensor<RadarParameter>
{
public:
    Sensor(RadarParameter currentParameter) : current_parameter(currentParameter) {
        this->parameter_num = 0;
        this->current_parameter = new RadarParameter[this->parameter_num];
    }
    ~Sensor()
    {
        delete[] this->parameter;
    }
    void add_parameter(RadarParameter radarParameter)
    {
        //参数：分辨率（float），视场角（integer：0-60），速度精度（float point），探测模式（string）
        cout << "请输入传感器编号：";
        cin >> radarParameter.id;
        cout << "请输入传感器名称：";
        cin >> radarParameter.name;
        cout << "请输入传感器的位置（x, y, z）：";
        cin >> radarParameter.x >> radarParameter.y >> radarParameter.z;
        cout << "请输入传感器的旋转角度（roll, pitch, yaw）：";
        cin >> radarParameter.roll >> radarParameter.pitch >> radarParameter.yaw;
        cout << "请输入传感器的分辨率(res_x,res_y)：";
        cin >> radarParameter.resolution[0] >> radarParameter.resolution[1];
        cout << "请输入传感器的视场角：";
        cin >> radarParameter.view_angle;
        cout << "请输入传感器的速度精度：";
        cin >> radarParameter.speed_accuracy[0] >> radarParameter.speed_accuracy[1];
        cout << "请输入传感器的探测模式：";
        cin >> radarParameter.detect_mode;
        // 导入数组
        this->parameter[this->parameter_num++] = radarParameter;
    }
    void show_parameter()
    {
        cout << "传感器编号\t传感器名称\t传感器位置\t传感器旋转角度\t传感器分辨率\t传感器视场角\t传感器速度精度\t传感器探测模式" << endl;
        for (int i = 0; i < this->parameter_num; i++)
        {
            cout<<this->parameter[i].id<<"\t"<<this->parameter[i].name<<"\t"<<this->parameter[i].x<<"\t"<<this->parameter[i].y<<"\t"<<this->parameter[i].z<<"\t"<<this->parameter[i].roll<<"\t"<<this->parameter[i].pitch<<"\t"<<this->parameter[i].yaw<<"\t"<<this->parameter[i].resolution[0]<<"\t"<<this->parameter[i].resolution[1]<<"\t"<<this->parameter[i].view_angle<<"\t"<<this->parameter[i].speed_accuracy[0]<<"\t"<<this->parameter[i].speed_accuracy[1]<<"\t"<<this->parameter[i].detect_mode<<endl;
        }

    }
    void set_current_parameter(int id)
    {
        this->current_parameter = this->parameter[id];
    }
    RadarParameter get_current_parameter()
    {
        return this->current_parameter;
    }
    int get_parameter_num() const
    {
        return this->parameter_num;
    }
    RadarParameter *get_parameter()
    {
        return this->parameter;
    }
private:
    int parameter_num;
    RadarParameter *parameter;
    RadarParameter current_parameter;

};

// 传感器管理类 SensorManager
/***
 * 数据成员包括三种特化传感器成员变量，其他数据成员还应有车辆ID，传感器种类，传感器数量，以及其他必要的相关数据等。
 * 对数据的操作应主要包括：添加、查找、删除、列表、统计和保存。
 * 其中，添加可以从文件读入或从键盘输入；
 * 查找 是按传感器ID进行查找。如果查到，则返回传感器所在数组中的下标序号，否则返回-1。
 * 删除 是将按传感器ID查到的传感器数据进行删除，删除时是将传感器类对象中的当前标志设置为“删除”标注。
 * 列表 是将传感器ID、传感器类型、传感器名称以及所有传感器信息按格式输出到屏幕上，要求开列表头。
 * 统计 操作包括：显示某个或多个传感器的ID，并能够按种类显示；统计传感器所在位置(x,y,z)。
 * 保存 是将SensorManager 类中的车辆ID、传感器数量、种类、以及动态数组中的所有未被删除的传感器数据保存到文件中（注意：请文件添加时需要按照保存的格式读入数据）。
 * 根据需要定义其他必要的成员函数，如构造函数、析构函数等。
 */
template <class Sensor>
class SensorManager
{
public:
    SensorManager()
    {
        this->sensor_num = 0;
        this->sensor_type = 0;
        this->sensor_list = NULL;
    }
    ~SensorManager()
    {
        if (this->sensor_list != NULL)
        {
            delete[] this->sensor_list;
        }
    }
    void add_sensor()
    {
        Sensor *sensor = new Sensor();
        sensor->add_sensor();
        this->sensor_list[this->sensor_num++] = sensor;
    }
    int find_sensor(int id)
    {
        for (int i = 0; i < this->sensor_num; i++)
        {
            if (this->sensor_list[i]->get_id() == id)
            {
                this->sensor_list[i]->show_sensor();
                return i;
            }
        }
        cout << "没有找到该传感器" << endl;
        return -1;
    }
    void delete_sensor(int id)
    {
        for (int i = 0; i < this->sensor_num; i++)
        {
            if (this->sensor_list[i]->get_id() == id)
            {
                this->sensor_list[i]->delete_sensor();
                this->sensor_list[i]->sensor_parameter_flag = 0;
                return;
            }
        }
        cout << "没有找到该传感器" << endl;
    }

protected:
    CameraParameter *camera_list;
    LidarParameter *lidar_list;
    RadarParameter *radar_list;
    int sensor_num;
    int sensor_type;
    Sensor *sensor_list;
};

int main() {

    return 0;
}
