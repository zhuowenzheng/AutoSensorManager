#include <iostream>
#include <string>
#include <fstream>
#include "Sensor.cpp"
#include "SensorParameter.cpp"

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
    Sensor();
    Sensor(const CameraParameter& currentParameter) : current_parameter(currentParameter) {
        if(parameters.empty())
        {
            Sensor<CameraParameter>::parameters.reserve(10);
        }
        this->current_parameter = currentParameter;
        Sensor<CameraParameter>::parameters.push_back(currentParameter);
    }
    ~Sensor()
    {
        parameters.clear();
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
        this->parameters.push_back(cameraParameter);
    }
    // 显示当前传感器参数
    void show_parameter()
    {
        cout << "传感器编号\t传感器名称\t传感器位置\t传感器旋转角度\t传感器分辨率\t传感器帧率\t传感器视场角\t传感器颜色位数" << endl;
        for (int i = 0; i < this->parameters.size(); i++)
        {
            cout << this->parameters[i].id << "\t" << this->parameters[i].name << "\t" << this->parameters[i].x << "\t" << this->parameters[i].y << "\t" << this->parameters[i].z << "\t" << this->parameters[i].roll << "\t" << this->parameters[i].pitch << "\t" << this->parameters[i].yaw << "\t" << this->parameters[i].resolution_x << "\t" << this->parameters[i].resolution_y << "\t" << this->parameters[i].frame_rate << "\t" << this->parameters[i].view_angle << "\t" << this->parameters[i].color_bits << endl;
        }
    }
    // 设置当前传感器参数
    void set_current_parameter(int id)
    {
        this->current_parameter.id = id;
    }

    // 获取当前传感器的标志
    int get_sensor_parameter_flag() const
    {
        return this->sensor_parameter_flag;
    }
    CameraParameter get_current_parameter()
    {
        return this->current_parameter;
    }
    CameraParameter get_parameter(int id)
    {
        vector<CameraParameter>::iterator it;
        for (it = this->parameters.begin(); it != this->parameters.end(); it++)
        {
            if (it->id == id)
            {
                return *it;
            }
        }
    }

private:
    static vector<CameraParameter> parameters;
    CameraParameter current_parameter;
    int sensor_parameter_flag{};

};


//特化传感器类 Sensor<LidarParameter>
template <> class Sensor<LidarParameter>
{
public:
    Sensor();
    Sensor(const LidarParameter& currentParameter) : current_parameter(currentParameter) {
        if(parameters.empty())
        {
            Sensor<LidarParameter>::parameters.reserve(10);
        }
        this->current_parameter = currentParameter;
        Sensor<LidarParameter>::parameters.push_back(currentParameter);
    }

    ~Sensor()
    {
        parameters.clear();
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
        this->parameters.push_back(lidarParameter);
    }
    void show_parameter()
    {
        cout << "传感器编号\t传感器名称\t传感器位置\t传感器旋转角度\t传感器线数\t传感器视场角\t传感器旋转频率\t传感器水平视场角" << endl;
        for (int i = 0; i < this->parameters.size(); i++)
        {
            cout << this->parameters[i].id << "\t" << this->parameters[i].name << "\t" << this->parameters[i].x << "\t" << this->parameters[i].y << "\t" << this->parameters[i].z << "\t" << this->parameters[i].roll << "\t" << this->parameters[i].pitch << "\t" << this->parameters[i].yaw << "\t" << this->parameters[i].view_angle << "\t" << this->parameters[i].horizontal_view_angle << endl;
        }
    }
    void set_current_parameter(int id)
    {
        this->current_parameter = this->parameters[id];
    }
    LidarParameter get_current_parameter()
    {
        return this->current_parameter;
    }
    LidarParameter get_parameter(int id)
    {
        vector<LidarParameter>::iterator it;
        for (it = this->parameters.begin(); it != this->parameters.end(); it++)
        {
            if (it->id == id)
            {
                return *it;
            }
        }
    }
private:
    vector<LidarParameter> parameters;
    LidarParameter current_parameter;
    int sensor_parameter_flag{};
};

// Radar
template <> class Sensor<RadarParameter>
{
public:
    Sensor();
    Sensor(const RadarParameter& currentParameter) : current_parameter(currentParameter) {
        if(parameters.empty())
        {
            Sensor<RadarParameter>::parameters.reserve(10);
        }
        this->current_parameter = currentParameter;
        Sensor<RadarParameter>::parameters.push_back(currentParameter);
    }
    ~Sensor()
    {
        parameters.clear();
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
        this->parameters.push_back(radarParameter);
    }
    void show_parameter()
    {
        cout << "传感器编号\t传感器名称\t传感器位置\t传感器旋转角度\t传感器分辨率\t传感器视场角\t传感器速度精度\t传感器探测模式" << endl;
        for (int i = 0; i < parameters.size(); i++)
        {
            cout << this->parameters[i].id << "\t" << this->parameters[i].name << "\t" << this->parameters[i].x << "\t" << this->parameters[i].y << "\t" << this->parameters[i].z << "\t" << this->parameters[i].roll << "\t" << this->parameters[i].pitch << "\t" << this->parameters[i].yaw << "\t" << this->parameters[i].resolution[0] << "\t" << this->parameters[i].resolution[1] << "\t" << this->parameters[i].view_angle << "\t" << this->parameters[i].speed_accuracy[0] << "\t" << this->parameters[i].speed_accuracy[1] << "\t" << this->parameters[i].detect_mode << endl;
        }

    }
    void set_current_parameter(int id)
    {
        this->current_parameter = this->parameters[id];
    }
    RadarParameter get_current_parameter()
    {
        return this->current_parameter;
    }
    RadarParameter get_parameter(int id)
    {
        vector<RadarParameter>::iterator it;
        for (it = this->parameters.begin(); it != this->parameters.end(); it++)
        {
            if (it->id == id)
            {
                return *it;
            }
        }
    }
private:
    static vector<RadarParameter> parameters;
    RadarParameter current_parameter;
    int sensor_parameter_flag;
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

class SensorManager
{
public:
    SensorManager(){}
    SensorManager(string Car_ID)
    {
        this->Car_ID = Car_ID;
        this->sensor_num = 0;
        this->sensor_type = 0;

    }
    ~SensorManager()
    {

    }
    // 添加传感器
    void add_sensor()
    {

    }
    // 查找传感器
    int find_sensor(int id)
    {

    }
    // 删除传感器
    void delete_sensor(int id)
    {

    }
    // 列表
    void list_sensor()
    {
        cout << "传感器ID\t传感器类型\t传感器名称\t传感器参数" << endl;

    }
    // 统计
    void statistic_sensor()
    {
        cout << "请输入要统计的传感器类型：" << endl;
        cout << "1.Camera" << endl;
        cout << "2.Lidar" << endl;
        cout << "3.Radar" << endl;
        int type;
        cin >> type;
        switch (type)
        {
        case 1:
            this->statistic_camera();
            break;
        case 2:
            this->statistic_lidar();
            break;
        case 3:
            this->statistic_radar();
            break;
        }
    }
    // 保存
    void save_sensor()
    {
        ofstream outfile;
        outfile.open("sensor.txt", ios::out);
        outfile << this->sensor_num << endl;
        outfile << this->sensor_type << endl;

        outfile.close();
    }
    void statistic_camera(){}
    void statistic_lidar(){}
    void statistic_radar(){}

private:
    Sensor<CameraParameter> camera_list;
    Sensor<LidarParameter> lidar_list;
    Sensor<RadarParameter> radar_list;
    int sensor_num;
    int sensor_type;
    string Car_ID;
};

int main() {
    // 操作界面
    cout<<"---------------------------------------"<<endl;
    cout<<"|           汽车传感器管理系统            |"<<endl;
    cout<<"|      Automobile Sensor Manager       |"<<endl;
    cout<<"|            by Alex Zheng             |"<<endl;
    cout<<"---------------------------------------"<<endl;
    cout<<"| 请选择要操作的功能:                     |"<<endl;
    cout<<"| 1.从输入添加传感器                     |"<<endl;
    cout<<"| 2.显示所有传感器                       |"<<endl;
    cout<<"| 3.显示在线的传感器                     |"<<endl;
    cout<<"| 4.查找指定传感器参数                    |"<<endl;
    cout<<"| 5.删除指定传感器                       |"<<endl;
    cout<<"| 6.传感器参数统计                       |"<<endl;
    cout<<"| 7.保存传感器参数到文件                  |"<<endl;
    cout<<"| 8.退出系统                            |"<<endl;
    cout<<"---------------------------------------"<<endl;
    int flag = 0;
    SensorManager sensor_manager();
    while (1) {
        cout << "请输入您的选择：";
        cin >> flag;
        switch (flag) {
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            case 5:
                break;
            case 6:
                break;
            case 7:
                break;
            case 8: {
                exit(0);
            }
        }
        return 0;
    }
}