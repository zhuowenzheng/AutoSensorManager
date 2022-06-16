#include <iostream>
#include <string>
#include <fstream>
#include <map>
#include <sstream>
#include "Sensor.cpp"
#include "SensorParameter.cpp"
#include "sensorErrorException.cpp"
//#include

using namespace std;

/******************************************************************************
 * @Requirements:
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
    CameraParameter(){}
    CameraParameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw, int resolution_x, int resolution_y, int frame_rate, int view_angle, const float distortion_parameter[5], int color_bits) : SensorParameter(id, name, x, y, z, roll, pitch, yaw)
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

    int resolution_x{};
    int resolution_y{};
    int frame_rate{};
    int view_angle{};
    float distortion_parameter[5]{};
    int color_bits{};
    // 重载>>操作符
    friend istream& operator>>(istream& is, CameraParameter& camera_parameter){
        is >> camera_parameter.id >> camera_parameter.name >> camera_parameter.x >> camera_parameter.y >> camera_parameter.z >> camera_parameter.roll >> camera_parameter.pitch >> camera_parameter.yaw >> camera_parameter.resolution_x >> camera_parameter.resolution_y >> camera_parameter.frame_rate >> camera_parameter.view_angle >> camera_parameter.distortion_parameter[0] >> camera_parameter.distortion_parameter[1] >> camera_parameter.distortion_parameter[2] >> camera_parameter.distortion_parameter[3] >> camera_parameter.distortion_parameter[4] >> camera_parameter.color_bits;
        return is;
    }
    // 重载<<操作符
    friend ostream& operator<<(ostream& os, const CameraParameter& camera_parameter){
        os << camera_parameter.id << " " << camera_parameter.name << " " << camera_parameter.x << " " << camera_parameter.y << " " << camera_parameter.z << " " << camera_parameter.roll << " " << camera_parameter.pitch << " " << camera_parameter.yaw << " " << camera_parameter.resolution_x << " " << camera_parameter.resolution_y << " " << camera_parameter.frame_rate << " " << camera_parameter.view_angle << " " << camera_parameter.distortion_parameter[0] << " " << camera_parameter.distortion_parameter[1] << " " << camera_parameter.distortion_parameter[2] << " " << camera_parameter.distortion_parameter[3] << " " << camera_parameter.distortion_parameter[4] << " " << camera_parameter.color_bits<<endl;
        return os;
    }
};

// class LidarParameter
// 继承SensorParameter，参数：线数（integer），视场角（integer: 0-45），旋转频率（integer），水平视场角（integer：0-360）
class LidarParameter : public SensorParameter
{
public:
    LidarParameter() {}
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

    // 重载>>操作符
    friend istream& operator>>(istream& is, LidarParameter& lidar_parameter){
        is >> lidar_parameter.id >> lidar_parameter.name >> lidar_parameter.x >> lidar_parameter.y >> lidar_parameter.z >> lidar_parameter.roll >> lidar_parameter.pitch >> lidar_parameter.yaw >> lidar_parameter.line_num >> lidar_parameter.view_angle >> lidar_parameter.rotate_rate >> lidar_parameter.horizontal_view_angle;
        return is;
    }
    // 重载<<操作符
    friend ostream& operator<<(ostream& os, const LidarParameter& lidar_parameter){
        os << lidar_parameter.id << " " << lidar_parameter.name << " " << lidar_parameter.x << " " << lidar_parameter.y << " " << lidar_parameter.z << " " << lidar_parameter.roll << " " << lidar_parameter.pitch << " " << lidar_parameter.yaw << " " << lidar_parameter.line_num << " " << lidar_parameter.view_angle << " " << lidar_parameter.rotate_rate << " " << lidar_parameter.horizontal_view_angle;
        return os;
    }

};

// class RadarParameter
// 继承SensorParameter，参数：分辨率（float point），视场角（integer：0-60），速度精度（float point），探测模式（string）
class RadarParameter : public SensorParameter
{
public:
    RadarParameter() {}
    RadarParameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw, const float resolution[2], int view_angle, float speed_accuracy[2], string detect_mode) : SensorParameter(id, name, x, y, z, roll, pitch, yaw)
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

    // 重载>>操作符
    friend istream& operator>>(istream& is, RadarParameter& radar_parameter){
        is >> radar_parameter.id >> radar_parameter.name >> radar_parameter.x >> radar_parameter.y >> radar_parameter.z >> radar_parameter.roll >> radar_parameter.pitch >> radar_parameter.yaw >> radar_parameter.resolution[0] >> radar_parameter.resolution[1] >> radar_parameter.view_angle >> radar_parameter.speed_accuracy[0] >> radar_parameter.speed_accuracy[1] >> radar_parameter.detect_mode;
        return is;
    }
    // 重载<<操作符
    friend ostream& operator<<(ostream& os, const RadarParameter& radar_parameter){
        os << radar_parameter.id << " " << radar_parameter.name << " " << radar_parameter.x << " " << radar_parameter.y << " " << radar_parameter.z << " " << radar_parameter.roll << " " << radar_parameter.pitch << " " << radar_parameter.yaw << " " << radar_parameter.resolution[0] << " " << radar_parameter.resolution[1] << " " << radar_parameter.view_angle << " " << radar_parameter.speed_accuracy[0] << " " << radar_parameter.speed_accuracy[1] << " " << radar_parameter.detect_mode;
        return os;
    }
};

// 特化传感器类 Sensor<CameraParameter>
// 用对应的模板参数定义动态传感器参数的动态数组；
// 对数据的操作应主要包括：添加、显示以及当前标志等数据成员的设置及获取等。
// 其中，添加是将键盘输入的参数构造成对应的传感器参数类对象中；显示是将当前的传感器参数按格式输出到屏幕上。
// 根据需要定义相应的构造函数及其他成员函数；
static int num_camera = 0;

template <>
class Sensor<CameraParameter>
{
public:
    Sensor(){}
    // 拷贝构造函数
    Sensor(CameraParameter &camera_parameter)
    {
        this->camera_parameter = &camera_parameter;
    }
    ~Sensor()
    {
        //parameters.clear();
        delete camera_parameter;
    }
    void add_parameter(CameraParameter cameraParameter)
    {
        //this->parameters.push_back(cameraParameter);
        if(this->camera_parameter == nullptr)
        {
            this->camera_parameter = new CameraParameter[reserved_sensor_num];
        }
        this->current_parameter = cameraParameter;
        if(num_camera>=reserved_sensor_num)//如果没有空间，则扩展数组
        {
            reserved_sensor_num*=2;
            //parameters.reserve(reserved_sensor_num);
            CameraParameter *temp = new CameraParameter[reserved_sensor_num];
            for(int i=0;i<num_camera;i++)
            {
                temp[i] = camera_parameter[i];
            }
            camera_parameter = temp;
            delete [] temp;
            //camera_parameter = (CameraParameter*)realloc(camera_parameter, reserved_sensor_num);
        }
        *(camera_parameter+num_camera++) = cameraParameter;
        // 传感器有效标签，默认为true=1
        is_effective[cameraParameter.id] = 1;
    }
    // 显示当前传感器参数
    void show_parameter()
    {
        cout << "\033[0;34m 编号\t名称\t位置\t旋转角度\t分辨率\t帧率\t视场角\t颜色位数 \033[0m" << endl;
        for (int i = 0; i < num_camera; i++)
        {
            cout << camera_parameter[i].id<<"\t"<<camera_parameter[i].name << "\t" << camera_parameter[i].x << "," << camera_parameter[i].y << "," << camera_parameter[i].z << "\t" << camera_parameter[i].roll << "," << camera_parameter[i].pitch << "," << camera_parameter[i].yaw << "\t" << camera_parameter[i].resolution_x << "," << camera_parameter[i].resolution_y << "\t" << camera_parameter[i].frame_rate << "\t" << camera_parameter[i].view_angle << "\t" << camera_parameter[i].color_bits << endl;
            //cout << parameters[i].id << "\t" << parameters[i].name << "\t" << parameters[i].x << "\t" << parameters[i].y << "\t" << parameters[i].z << "\t" << parameters[i].roll << "\t" << parameters[i].pitch << "\t" << parameters[i].yaw << "\t" << parameters[i].resolution_x << "\t" << parameters[i].resolution_y << "\t" << parameters[i].frame_rate << "\t" << parameters[i].view_angle << "\t" << parameters[i].color_bits << endl;
        }
    }
    // 设置当前传感器参数
    void set_current_parameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw, int resolution_x, int resolution_y, int frame_rate, int view_angle, int color_bits)
    {
        this->current_parameter.id = id;
        this->current_parameter.name = name;
        this->current_parameter.x = x;
        this->current_parameter.y = y;
        this->current_parameter.z = z;
        this->current_parameter.roll = roll;
        this->current_parameter.pitch = pitch;
        this->current_parameter.yaw = yaw;
        this->current_parameter.resolution_x = resolution_x;
        this->current_parameter.resolution_y = resolution_y;
        this->current_parameter.frame_rate = frame_rate;
        this->current_parameter.view_angle = view_angle;
        this->current_parameter.color_bits = color_bits;
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
        for(int i=0;i<num_camera;i++)
        {
            if(camera_parameter[i].id == id)
            {
                return camera_parameter[i];
            }
        }
    }
    int get_sensor_num(){
        return num_camera;
    }

    int get_online_sensors_num(){
        int num = 0;
        for(int i=0;i<num_camera;i++)
        {
            //cout<<"num_camera:"<<num_camera<<endl;
            if(is_effective[camera_parameter[i].id]==1)
            {
                //cout<<"reached "<<i<<endl;
                num++;
            }
        }
        return num;
    }
    //vector实现的备份
   /* vector<CameraParameter> get_parameters(){
        return this->parameters;
    }*/

    void list_sensors(){
        cout<<"\033[0;34m Camera编号\tCamera名称\tCamera位置 \033[0m"<<endl;
        for (int i = 0; i < num_camera;)
        {
            //未被删除
            if (is_effective[camera_parameter[i].id]!=-1)
            {
                cout << camera_parameter[i].id << "\t" << camera_parameter[i].name << "\t" << camera_parameter[i].x << "," << camera_parameter[i].y << "," << camera_parameter[i].z << endl;
                i++;
            }
            //cout << camera_parameter[i].id<<"\t"<<camera_parameter[i].name << "\t" << camera_parameter[i].x << "," << camera_parameter[i].y << "," << camera_parameter[i].z << endl;
            //cout << this->parameters[i].id << "\t" << this->parameters[i].name<<"\t"<<parameters[i].x<<" "<<parameters[i].y<<" "<<parameters[i].z<< endl;
        }
    }
    void list_online_sensors(){
        cout<<"\033[0;34m Camera编号\tCamera名称\tCamera位置 \033[0m"<<endl;
        for (int i = 0; i < get_online_sensors_num(); )
        {
            // 在线且未被删除
            if(is_effective[camera_parameter[i].id]==1)
            {
                cout << camera_parameter[i].id<<"\t"<<camera_parameter[i].name << "\t" << camera_parameter[i].x << "," << camera_parameter[i].y << "," << camera_parameter[i].z << endl;
                //cout << this->parameters[i].id << "\t" << this->parameters[i].name<<"\t"<<parameters[i].x<<" "<<parameters[i].y<<" "<<parameters[i].z<< endl;
                i++;
            }
        }
    }


    CameraParameter *camera_parameter = nullptr;
    map<int,int> is_effective;//-1:deleted 0:offline 1:online
private:
    CameraParameter current_parameter;
    int sensor_parameter_flag;
    int reserved_sensor_num = 10;
};

static int num_lidar = 0;
// 特化传感器类 Sensor<LidarParameter>
template <> class Sensor<LidarParameter>
{
public:
    Sensor(){}
    // 拷贝构造函数
    Sensor(LidarParameter &lidarParameter)
    {
        this->lidar_parameter = &lidarParameter;
    }
    ~Sensor()
    {
        delete [] lidar_parameter;
    }
    void add_parameter(LidarParameter lidarParameter)
    {
        // 导入
        if(this->lidar_parameter == nullptr)
        {
            this->lidar_parameter = new LidarParameter[reserved_sensor_num];
        }
        this->current_parameter = lidarParameter;
        if(num_lidar>=reserved_sensor_num)
        {
            reserved_sensor_num*=2;
            //parameters.reserve(reserved_sensor_num);
            LidarParameter *temp = new LidarParameter[reserved_sensor_num];
            for(int i=0;i<num_lidar;i++)
            {
                temp[i] = lidar_parameter[i];
            }
            lidar_parameter = temp;
            delete [] temp;

        }
        *(lidar_parameter+num_lidar++) = lidarParameter;
        is_effective[lidarParameter.id] = 1;
    }
    void show_parameter()
    {
        cout << "\033[0;34m 编号\t名称\t位置\t旋转角度\t线数\t视场角\t旋转频率\t水平视场角 \033[0m" << endl;
        for (int i = 0; i < num_lidar; i++)
        {
            cout << this->lidar_parameter[i].id << "\t" << this->lidar_parameter[i].name << "\t" << this->lidar_parameter[i].x << "\t" << this->lidar_parameter[i].y << "\t" << this->lidar_parameter[i].z << "\t" << this->lidar_parameter[i].roll << "\t" << this->lidar_parameter[i].pitch << "\t" << this->lidar_parameter[i].yaw << "\t" << this->lidar_parameter[i].view_angle << "\t" << this->lidar_parameter[i].horizontal_view_angle << endl;
        }
    }
    void set_current_parameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw, int line_num, int view_angle,int rotate_rate,int horizontal_view_angle)
    {
        this->current_parameter.id = id;
        this->current_parameter.name = name;
        this->current_parameter.x = x;
        this->current_parameter.y = y;
        this->current_parameter.z = z;
        this->current_parameter.roll = roll;
        this->current_parameter.pitch = pitch;
        this->current_parameter.yaw = yaw;
        this->current_parameter.line_num = line_num;
        this->current_parameter.view_angle = view_angle;
        this->current_parameter.rotate_rate = rotate_rate;
        this->current_parameter.horizontal_view_angle = horizontal_view_angle;
    }

    LidarParameter get_current_parameter()
    {
        return this->current_parameter;
    }
    LidarParameter get_parameter(int id)
    {
        for(int i=0;i<num_lidar;i++)
        {
            if(lidar_parameter[i].id == id)
            {
                return lidar_parameter[i];
            }
        }
    }
    int get_sensor_num(){
        return num_lidar;
    }

    int get_online_sensor_num(){
        int num = 0;
        for(int i=0;i<num_lidar;i++)
        {
            if(is_effective[lidar_parameter[i].id]==1)
            {
                num++;
            }
        }
        return num;
    }
    /*vector<LidarParameter> get_parameters()
    {
        return this->parameters;
    }*/
    void list_sensors(){
        cout<<"\033[0;34m Lidar编号\tLidar名称\tLidar位置 \033[0m"<<endl;
        for (int i = 0; i < num_lidar;){
            if(lidar_parameter[i].id!=-1){
                cout<<lidar_parameter[i].id<<"\t"<<lidar_parameter[i].name<<"\t"<<lidar_parameter[i].x<<","<<lidar_parameter[i].y<<","<<lidar_parameter[i].z<<endl;
                i++;
            }
        }
    }

    void list_online_sensors(){
        cout<<"\033[0;34m Lidar编号\tLidar名称\tLidar位置 \033[0m"<<endl;
        for (int i = 0; i < num_lidar;){
            if(is_effective[lidar_parameter[i].id]==1){
                cout << lidar_parameter[i].id<<"\t"<<lidar_parameter[i].name << "\t" << lidar_parameter[i].x << "," << lidar_parameter[i].y << "," << lidar_parameter[i].z << endl;
                i++;
            }
        }
    }

    LidarParameter* lidar_parameter = nullptr;
    map<int,int> is_effective;
private:
    LidarParameter current_parameter;
    int sensor_parameter_flag;
    int reserved_sensor_num = 10;
};

// 特化传感器类 Sensor<RadarParameter>
static int num_radar = 0;
template <> class Sensor<RadarParameter>
{
public:
    Sensor(){}
    Sensor(RadarParameter &radarParameter)
    {
        this->radar_parameter = &radarParameter;
    }
    ~Sensor()
    {
        delete [] radar_parameter;
    }
    void add_parameter(RadarParameter radarParameter)
    {
        if(this->radar_parameter == nullptr)
        {
            this->radar_parameter = new RadarParameter[reserved_sensor_num];
        }
        this->current_parameter = radarParameter;
        if(num_radar>=reserved_sensor_num)
        {
            reserved_sensor_num*=2;
            RadarParameter *temp = new RadarParameter[reserved_sensor_num];
            for(int i=0;i<num_radar;i++)
            {
                temp[i] = radar_parameter[i];
            }
            radar_parameter = temp;
            delete [] temp;
        }
        *(radar_parameter+num_radar++) = radarParameter;
        is_effective[radarParameter.id] = 1;
    }
    void show_parameter() {

        cout << "\033[0;34m 编号\t名称\t位置\t旋转角度\t分辨率\t视场角\t速度精度\t探测模式 \033[0m" << endl;
        for (int i = 0; i < num_radar; i++) {

            cout << radar_parameter[i].id << "\t" << radar_parameter[i].name << "\t" << radar_parameter[i].x << "\t"
                 << radar_parameter[i].y << "\t" << radar_parameter[i].z << "\t" << radar_parameter[i].roll << "\t"
                 << radar_parameter[i].pitch << "\t" << radar_parameter[i].yaw << "\t" << radar_parameter[i].resolution[0]<<","<<radar_parameter[i].resolution[1]
                 << "\t" << radar_parameter[i].view_angle << "\t" << radar_parameter[i].speed_accuracy[0]<<","<<radar_parameter[i].speed_accuracy[1] << "\t"
                 << radar_parameter[i].detect_mode << endl;
        }
    }
    void set_current_parameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw, int resolution_x, int resolution_y, int view_angle, int speed_accuracy_x,int speed_accuracy_y, int detect_mode)
    {
        this->current_parameter.id = id;
        this->current_parameter.name = name;
        this->current_parameter.x = x;
        this->current_parameter.y = y;
        this->current_parameter.z = z;
        this->current_parameter.roll = roll;
        this->current_parameter.pitch = pitch;
        this->current_parameter.yaw = yaw;
        this->current_parameter.resolution[0] = resolution_x;
        this->current_parameter.resolution[1] = resolution_y;
        this->current_parameter.view_angle = view_angle;
        this->current_parameter.speed_accuracy[0] = speed_accuracy_x;
        this->current_parameter.speed_accuracy[1] = speed_accuracy_y;
        this->current_parameter.detect_mode = detect_mode;
    }
    RadarParameter get_current_parameter()
    {
        return this->current_parameter;
    }
    RadarParameter get_parameter(int id)
    {
        for(int i=0;i<num_radar;i++)
        {
            if(radar_parameter[i].id == id)
            {
                return radar_parameter[i];
            }
        }
    }
    int get_sensor_num(){
        return num_radar;
    }

    int get_online_sensor_num(){
        int num = 0;
        for(int i=0;i<num_radar;i++)
        {
            if(is_effective[radar_parameter[i].id]==1)
            {
                num++;
            }
        }
        return num;
    }
   /* vector<RadarParameter> get_parameters()
    {
        return this->parameters;
    }*/
    void list_sensors(){
        cout<<"\033[0;34m Radar编号\tRadar名称\tRadar位置 \033[0m"<<endl;
        for(int i = 0;i<num_radar;){
            if(is_effective[radar_parameter[i].id]==1){
                cout << radar_parameter[i].id<<"\t"<<radar_parameter[i].name << "\t" << radar_parameter[i].x << "," << radar_parameter[i].y << "," << radar_parameter[i].z << endl;
                i++;
            }
        }
    }

    void list_online_sensors(){
        cout<<"\033[0;34m Radar编号\tRadar名称\tRadar位置 \033[0m"<<endl;
        for(int i = 0;i<num_radar;){
            if(is_effective[radar_parameter[i].id]==1){
                cout<<radar_parameter[i].id<<"\t"<<radar_parameter[i].name<<"\t"<<radar_parameter[i].x<<"\t"<<radar_parameter[i].y<<"\t"<<radar_parameter[i].z<<endl;
                i++;
            }
        }
    }


    RadarParameter *radar_parameter = nullptr;
    map<int,int> is_effective; //标志，由于指针在Parameter上，故用map实现
private:
    RadarParameter current_parameter;
    int sensor_parameter_flag;
    int reserved_sensor_num = 10;
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
    SensorManager();
    SensorManager(string Car_ID)
    {
        this->Car_ID = Car_ID;
        this->sensor_num = 0;
        this->sensor_type = 0;
    }
    ~SensorManager(){}
    // 文件导入camera

    ifstream & seek_to_line(ifstream & in, int line)
    //将打开的文件in，定位到line行。
    {
        int i;
        char buf[1024];
        in.seekg(0, ios::beg);  //定位到文件开始。
        for (i = 0; i < line; i++)
        {
            in.getline(buf, sizeof(buf));//读取行。
        }
        return in;
    }
    void add_camera_from_file(const string& file_path){
        ifstream file;
        string line;
        file.open(file_path, ios::in);
        if(!file.is_open()){
            cout<<"\033[0;31m 文件打开失败 \033[0m"<<endl;
            return;
        }
        cout<<"\033[0;32m 文件读取成功 \033[0m"<<endl;
        seek_to_line(file,2);
        //从文件中读取数据，放入temp_camera中，一行数据对应一个CameraParameter对象，数据成员之间以空格分隔
        while(getline(file,line)){
            CameraParameter temp_camera;
            istringstream iss(line);
            iss>>temp_camera;
            sensor_num++;
            cout<<temp_camera<<endl;
            camera_list.add_parameter(temp_camera);
        }
        file.close();
        cout<<"\033[0;32m 文件关闭 \033[0m"<<endl;
    }
    //文件导入lidar
    void add_lidar_from_file(string file_path){
        ifstream file;
        string line;
        file.open(file_path, ios::in);
        if(!file.is_open()){
            cout<<"\033[0;31m 文件打开失败 \033[0m"<<endl;
            return;
        }
        cout<<"\033[0;32m 文件读取成功 \033[0m"<<endl;
        seek_to_line(file,2);
        //从文件中读取数据，放入temp_lidar中，一行数据对应一个LidarParameter对象，数据成员之间以空格分隔
        while(getline(file,line)){
            LidarParameter temp_lidar;
            istringstream iss(line);
            iss>>temp_lidar;
            sensor_num++;
            cout<<temp_lidar<<endl;
            lidar_list.add_parameter(temp_lidar);
        }
        file.close();
        cout<<"\033[0;32m 文件关闭 \033[0m"<<endl;
    }
    // 文件导入radar
    void add_radar_from_file(string file_path){
        ifstream file;
        string line;
        file.open(file_path, ios::in);
        if(!file.is_open()){
            cout<<"\033[0;31m 文件打开失败 \033[0m"<<endl;
            return;
        }
        cout<<"\033[0;32m 文件读取成功 \033[0m"<<endl;
        seek_to_line(file,2);
        //从文件中读取数据，放入temp_radar中，一行数据对应一个RadarParameter对象，数据成员之间以空格分隔
        while(getline(file,line)){
            RadarParameter temp_radar;
            istringstream iss(line);
            iss>>temp_radar;
            sensor_num++;
            cout<<temp_radar<<endl;
            radar_list.add_parameter(temp_radar);
        }
        file.close();
        cout<<"\033[0;32m 文件关闭 \033[0m"<<endl;
    }

    void add_sensor_from_istream()
    {
        int id;
        string name;
        float x, y, z;
        float roll, pitch, yaw;
        cout << "请输入传感器类型（1:Camera;2:Lidar;3:Radar）：";
        cin >> this->sensor_type;
        switch (this->sensor_type) {
        case 1: {
            //Camera
            //分辨率（integer, integer），帧率（integer），视场角(integer: 0-180)，畸变参数（float point[5]），色彩位数（integer）
            int resolution_x, resolution_y, frame_rate, view_angle, color_bits;
            float distortion_parameter[5];
            cout << "请输入传感器编号：";
            cin >> id;
            cout << "请输入传感器名称：";
            cin >> name;
            cout << "请输入传感器的位置（x, y, z）：";
            cin >> x >> y >> z;
            cout << "请输入传感器的旋转角度（roll, pitch, yaw）：";
            cin >> roll >> pitch >> yaw;
            cout << "请输入传感器的分辨率（x, y）：";
            cin >> resolution_x >> resolution_y;
            cout << "请输入传感器的帧率：";
            cin >> frame_rate;
            cout << "请输入传感器的视场角：";
            cin >> view_angle;
            cout << "请输入传感器的畸变参数（k1, k2, k3, k4, k5）：";
            cin >> distortion_parameter[0] >> distortion_parameter[1] >> distortion_parameter[2] >> distortion_parameter[3] >> distortion_parameter[4];
            cout << "请输入传感器的颜色位数：";
            cin >> color_bits;
            CameraParameter camera(id, name, x, y, z, roll, pitch, yaw, resolution_x, resolution_y, frame_rate, view_angle, distortion_parameter, color_bits);
            this->camera_list.add_parameter(camera);
            sensor_num++;
            cout<< "[INFO] 添加Camera: "<<id<<":"<<name<<"成功！" << endl<<endl;
            sensor_type=0;
            break;
        }
        case 2:{
            //Lidar
            //线数（integer），视场角（integer: 0-45），旋转频率（integer），水平视场角（integer：0-360）
            int line_num, view_angle, rotation_rate, horizontal_view_angle;

            cout << "请输入传感器编号：";
            cin >> id;
            cout << "请输入传感器名称：";
            cin >> name;
            cout << "请输入传感器的位置（x, y, z）：";
            cin >> x >> y >> z;
            cout << "请输入传感器的旋转角度（roll, pitch, yaw）：";
            cin >> roll >> pitch >> yaw;
            cout << "请输入传感器的线数：";
            cin >> line_num;
            cout << "请输入传感器的视场角：";
            cin >> view_angle;
            cout << "请输入传感器的旋转频率：";
            cin >> rotation_rate;
            cout << "请输入传感器的水平视场角：";
            cin >> horizontal_view_angle;
            LidarParameter lidar(id, name, x, y, z, roll, pitch, yaw, line_num, view_angle, rotation_rate, horizontal_view_angle);
            lidar_list.add_parameter(lidar);
            sensor_num++;
            cout<< "[INFO] 添加Lidar: "<<id<<":"<<name<<"成功！" << endl<<endl;
            sensor_type=0;
            break;

        }
        case 3:{
            //Radar
            float resolution[2];
            float view_angle;
            float speed_accuracy[2];
            string detect_mode;
            //参数：分辨率（float），视场角（integer：0-60），速度精度（float point），探测模式（string）
            cout << "请输入传感器编号：";
            cin >> id;
            cout << "请输入传感器名称：";
            cin >> name;
            cout << "请输入传感器的位置（x, y, z）：";
            cin >> x >> y >> z;
            cout << "请输入传感器的旋转角度（roll, pitch, yaw）：";
            cin >> roll >> pitch >> yaw;
            cout << "请输入传感器的分辨率(res_x,res_y)：";
            cin >> resolution[0] >> resolution[1];
            cout << "请输入传感器的视场角：";
            cin >> view_angle;
            cout << "请输入传感器的速度精度：";
            cin >> speed_accuracy[0] >> speed_accuracy[1];
            cout << "请输入传感器的探测模式：";
            cin >> detect_mode;
            RadarParameter radar(id, name, x, y, z, roll, pitch, yaw, resolution, view_angle, speed_accuracy, detect_mode);
            radar_list.add_parameter(radar);
            sensor_num++;
            cout<< "[INFO] 添加Radar: "<<id<<":"<<name<<"成功！" << endl<<endl;
            sensor_type=0;
            break;
        }
        }

    }


    // 查找传感器
    int find_sensor(int id,string* sensor_type)
    {
        // 遍历camera_list、lidar_list、radar_list，查找id
        int condition;
        switch (condition) {
            case 1:
                //camera
                for(int i=0;i<camera_list.get_sensor_num();i++)
                {
                    if(camera_list.camera_parameter[i].id==id)
                    {
                        *sensor_type = "camera";
                        return i;
                    }
                }
                break;
            case 2:
                //lidar
                for(int i=0;i<lidar_list.get_sensor_num();i++)
                {
                    if(lidar_list.lidar_parameter[i].id==id)
                    {
                        *sensor_type = "lidar";
                        return i;
                    }
                }
                break;
            case 3:
                //radar
                for(int i=0;i<radar_list.get_sensor_num();i++)
                {
                    if(radar_list.radar_parameter[i].id==id)
                    {
                        *sensor_type = "radar";
                        return i;
                    }
                }
                break;
        }
        return -1;
    }
    // 删除传感器
    void delete_sensor(int id)
    {
        //main中已对id存在性进行检查
        // 遍历camera_list、lidar_list、radar_list，查找id并删除
        for(int i = 0; i < camera_list.get_sensor_num(); i++)
        {
            if(camera_list.camera_parameter[i].id == id)
            {
                // 删除camera_list中的传感器
                string name = camera_list.camera_parameter[i].name;
                camera_list.is_effective[camera_list.camera_parameter[i].id] = -1;
                camera_list.camera_parameter[i].id = -1;
                camera_list.camera_parameter[i].name = "";
                camera_list.camera_parameter[i].x = 0;
                camera_list.camera_parameter[i].y = 0;
                camera_list.camera_parameter[i].z = 0;
                camera_list.camera_parameter[i].roll = 0;
                camera_list.camera_parameter[i].pitch = 0;
                camera_list.camera_parameter[i].yaw = 0;
                camera_list.camera_parameter[i].resolution_x = 0;
                camera_list.camera_parameter[i].resolution_y = 0;
                camera_list.camera_parameter[i].frame_rate = 0;
                camera_list.camera_parameter[i].view_angle = 0;
                camera_list.camera_parameter[i].color_bits = 0;
                num_camera--;
                sensor_num--;
                cout<< "[INFO] 删除Camera: "<<id<<":"<<name<<"成功！" <<endl;
                return;
            }
        }
        // lidar
        for(int i =0;i< lidar_list.get_sensor_num();i++){
            if(lidar_list.lidar_parameter[i].id==id){
                // 删除lidar_list中的传感器
                string name = lidar_list.lidar_parameter[i].name;
                lidar_list.is_effective[lidar_list.lidar_parameter[i].id] = -1;
                lidar_list.lidar_parameter[i].id = -1;
                lidar_list.lidar_parameter[i].name = "";
                lidar_list.lidar_parameter[i].x = 0;
                lidar_list.lidar_parameter[i].y = 0;
                lidar_list.lidar_parameter[i].z = 0;
                lidar_list.lidar_parameter[i].roll = 0;
                lidar_list.lidar_parameter[i].pitch = 0;
                lidar_list.lidar_parameter[i].yaw = 0;
                lidar_list.lidar_parameter[i].line_num = 0;
                lidar_list.lidar_parameter[i].view_angle = 0;
                lidar_list.lidar_parameter[i].horizontal_view_angle = 0;
                lidar_list.lidar_parameter[i].rotate_rate = 0;

                num_lidar--;
                sensor_num--;
                cout<< "[INFO] 删除Lidar: "<<id<<":"<<name<<"成功！" <<endl;
                return;
            }
        }
        //radar
        for(int i =0;i< radar_list.get_sensor_num();i++){
            if(radar_list.radar_parameter[i].id==id){
                // 删除radar_list中的传感器
                string name = radar_list.radar_parameter[i].name;
                radar_list.is_effective[radar_list.radar_parameter[i].id] = -1;//-1表示deleted
                radar_list.radar_parameter[i].id = -1;
                radar_list.radar_parameter[i].name = "";
                radar_list.radar_parameter[i].x = 0;
                radar_list.radar_parameter[i].y = 0;
                radar_list.radar_parameter[i].z = 0;
                radar_list.radar_parameter[i].roll = 0;
                radar_list.radar_parameter[i].pitch = 0;
                radar_list.radar_parameter[i].yaw = 0;
                radar_list.radar_parameter[i].resolution[0] = 0;
                radar_list.radar_parameter[i].resolution[1] = 0;
                radar_list.radar_parameter[i].view_angle = 0;
                radar_list.radar_parameter[i].speed_accuracy[0] = 0;
                radar_list.radar_parameter[i].speed_accuracy[1] = 0;
                radar_list.radar_parameter[i].detect_mode="";

                num_radar--;
                sensor_num--;
                cout<< "[INFO] 删除Radar: "<<id<<":"<<name<<"成功！" <<endl;
                return;
            }
        }
    }
    // 列表
    void list_all_sensor()
    {
        cout<<"-----------------传感器列表-----------------"<<endl;
        cout<<"现共有"<<sensor_num<<"个传感器："<<endl;
        cout<<"其中Camera:"<<camera_list.get_sensor_num()<<"个, ";
        cout<<"Lidar:"<<lidar_list.get_sensor_num()<<"个, ";
        cout<<"Radar:"<<radar_list.get_sensor_num()<<"个."<<endl;
        camera_list.list_sensors();
        lidar_list.list_sensors();
        radar_list.list_sensors();
        cout<<"-------------------------------------------"<<endl;

    }
    // 显示在线传感器
    void list_online_sensor(){
        cout<<"-----------------在线传感器列表-----------------"<<endl;
        cout<<"现共有"<<camera_list.get_online_sensors_num()+lidar_list.get_online_sensor_num()+ radar_list.get_online_sensor_num()<<"个传感器在线："<<endl;
        cout<<"其中Camera:"<<camera_list.get_online_sensors_num()<<"个, ";
        cout<<"Lidar:"<<lidar_list.get_online_sensor_num()<<"个, ";
        cout<<"Radar:"<<radar_list.get_online_sensor_num()<<"个."<<endl;
        camera_list.list_online_sensors();
        lidar_list.list_online_sensors();
        radar_list.list_online_sensors();
        cout<<"-------------------------------------------"<<endl;
    }
    // 统计指定传感器
    void statistic_specified_sensors(int sensor_id[],int count){
        cout<<"-----------------传感器统计-----------------"<<endl;
        cout<<"统计传感器："<<endl;
        cout<<count<<endl;
        string types[count];
        int camera_num = 0;
        int lidar_num = 0;
        int radar_num = 0;
        for(int i =0;i<count;i++){
            find_sensor(sensor_id[i],&types[i]);
            if(types[i]=="camera"){
                camera_num++;
            }
            else if(types[i]=="lidar"){
                lidar_num++;
            }
            else if(types[i]=="radar"){
                radar_num++;
            }
        }
        if(camera_num>0){
            cout<<"Camera:"<<camera_num<<"个"<<endl;
            cout<<"ID"<<"\t"<<"名称"<<"\t"<<"位置"<<endl;

            for(int i =0;i<count;i++){
                if(types[i]=="camera")
                {
                cout<<camera_list.camera_parameter[i].id<<"\t"<<camera_list.camera_parameter[i].name<<"\t";
                cout<<"("<<camera_list.camera_parameter[i].x<<","<<camera_list.camera_parameter[i].y<<","<<camera_list.camera_parameter[i].z<<")"<<endl<<endl;
                }
            }
        }
        if(lidar_num>0){
            cout<<"Lidar:"<<lidar_num<<"个"<<endl;
            cout<<"ID"<<"\t"<<"名称"<<"\t"<<"位置"<<endl;

            for(int i =0;i<count;i++){
                if(types[i]=="lidar")
                {
                cout<<lidar_list.lidar_parameter[i].id<<"\t"<<lidar_list.lidar_parameter[i].name<<"\t";
                cout<<"("<<lidar_list.lidar_parameter[i].x<<","<<lidar_list.lidar_parameter[i].y<<","<<lidar_list.lidar_parameter[i].z<<")"<<endl<<endl;
                }
            }
        }
        if(radar_num>0){
            cout<<"Radar:"<<radar_num<<"个"<<endl;
            cout<<"ID"<<"\t"<<"名称"<<"\t"<<"位置"<<endl;

            for(int i =0;i<count;i++){
                if(types[i]=="radar")
                {
                cout<<radar_list.radar_parameter[i].id<<"\t"<<radar_list.radar_parameter[i].name<<"\t";
                cout<<"("<<radar_list.radar_parameter[i].x<<","<<radar_list.radar_parameter[i].y<<","<<radar_list.radar_parameter[i].z<<")"<<endl<<endl;
                }
            }
        }

    }
    void statistic_sensor_parameter()
    {
        cout<<"-----------------传感器参数统计-----------------"<<endl;
        cout << "请输入要统计的传感器类型：" << endl;
        cout << "1.Camera" << endl;
        cout << "2.Lidar" << endl;
        cout << "3.Radar" << endl;
        int type;
        cin >> type;
        switch (type)
        {
        case 1:
        {
            cout<<"---------------------------------Camera---------------------------------"<<endl;
            camera_list.show_parameter();
            cout<<"----------------------------------END-----------------------------------"<<endl<<endl;
            break;
        }

        case 2:
        {
            cout<<"---------------------------------Lidar---------------------------------"<<endl;
            lidar_list.show_parameter();
            cout<<"----------------------------------END-----------------------------------"<<endl<<endl;
            break;
        }
        case 3:
        {

            cout<<"---------------------------------Radar---------------------------------"<<endl;
            radar_list.show_parameter();
            cout<<"----------------------------------END-----------------------------------"<<endl<<endl;
            break;
        }
        }
    }
    // 保存
    void save_parameter_tofile(string file_directory,string carID)
    {
        string file_directory_prefix = file_directory+"/"+carID+"_";
        fstream outfile;
        outfile.open(file_directory_prefix+"camera.txt", ios::out);
        // 写入camera参数
        outfile << "#" << carID << endl;
        outfile << "##"<< camera_list.get_sensor_num() << endl;
        for(int i = 0; i < camera_list.get_sensor_num(); ){
            // Camera参数继承传感器参数基类，分辨率（integer, integer），帧率（integer），视场角(integer: 0-180)，畸变参数（float point[5]），色彩位数（integer）
            if(camera_list.is_effective[camera_list.camera_parameter[i].id]){
                outfile << camera_list.camera_parameter[i]<< endl;
                i++;
            }
        }
        outfile.close();
        cout<<"[INFO] 保存Camera参数成功！"<<endl;
        // 写入lidar参数
        // 继承SensorParameter，参数：线数（integer），视场角（integer: 0-45），旋转频率（integer），水平视场角（integer：0-360）
        outfile.open(file_directory_prefix+"lidar.txt",ios::out);
        outfile << "#" << carID << endl;
        outfile << "##"<< lidar_list.get_sensor_num() << endl;
        for(int i = 0; i < lidar_list.get_sensor_num(); ) {
            if(lidar_list.is_effective[lidar_list.lidar_parameter[i].id]){
                outfile << lidar_list.lidar_parameter[i];
                i++;
            }
        }
        outfile.close();
        cout<<"[INFO] 保存Lidar参数成功！"<<endl;
        // 写入radar参数
        // 继承SensorParameter，参数：分辨率（float point），视场角（integer：0-60），速度精度（float point），探测模式（string）
        outfile.open(file_directory_prefix+"radar.txt",ios::out);
        outfile <<"#" << carID << endl;
        outfile <<"##"<< radar_list.get_sensor_num() << endl;
        for(int i = 0; i < radar_list.get_sensor_num(); ) {
            if(radar_list.is_effective[radar_list.radar_parameter[i].id]){
                outfile << radar_list.radar_parameter[i];
                i++;
            }
        }
        outfile.close();
        cout<<"[INFO] 保存Radar参数成功！"<<endl;
    }

    string get_Car_ID(){
        return Car_ID;
    }

private:
    Sensor<CameraParameter> camera_list;
    Sensor<LidarParameter> lidar_list;
    Sensor<RadarParameter> radar_list;
    int sensor_num = 0;
    int sensor_type;
    string Car_ID;
};

void ui_initialize(){
    // 操作界面
    cout<<"---------------------------------------"<<endl;
    cout<<"|           汽车传感器管理系统            |"<<endl;
    cout<<"|      Automobile Sensor Manager       |"<<endl;
    cout<<"|            by Alex Zheng             |"<<endl;
    cout<<"---------------------------------------"<<endl;
    cout<<"| 请选择要操作的功能:                     |"<<endl;
    cout<<"| 1.从文件添加传感器参数                  |"<<endl;
    cout<<"| 2.从输入添加传感器                     |"<<endl;
    cout<<"| 3.列表显示所有传感器                    |"<<endl;
    cout<<"| 4.显示所有在线传感器                    |"<<endl;
    cout<<"| 5.查找指定传感器参数                    |"<<endl;
    cout<<"| 6.删除指定传感器                       |"<<endl;
    cout<<"| 7.传感器参数统计                       |"<<endl;
    cout<<"| 8.保存传感器参数到文件                  |"<<endl;
    cout<<"| 9.退出系统                            |"<<endl;
    cout<<"---------------------------------------"<<endl;
}

int main() {
    ui_initialize();
    SensorManager sensor_manager("Default-Car");
    while (1) {
        int choice = 0;
        ui_initialize();
        cout << "请输入您的选择：";
        cin >> choice;
        switch (choice) {
            case 1: {
                bool file_input = true;
                while (file_input) {
                    cout << "请输入要添加的传感器类型：" << endl;
                    cout << "1.Camera" << endl;
                    cout << "2.Lidar" << endl;
                    cout << "3.Radar" << endl;
                    cout << "4.退出" << endl;
                    int type;
                    cin.clear();
                    cin.ignore();
                    cin.sync();
                    cin >> type;
                    switch (type) {
                        case 1: {
                            cout << "请输入要添加的Camera参数文件路径：" << endl;
                            string file_directory_camera;
                            cin.clear();
                            cin.ignore();
                            cin.sync();
                            cin >> file_directory_camera;
                            sensor_manager.add_camera_from_file(file_directory_camera);
                            type = 4;
                            break;
                        }
                        case 2: {
                            cout << "请输入要添加的Lidar参数文件路径：" << endl;
                            string file_directory_lidar;
                            cin.clear();
                            cin.ignore();
                            cin.sync();
                            cin >> file_directory_lidar;
                            sensor_manager.add_lidar_from_file(file_directory_lidar);
                            type = 4;
                            break;
                        }
                        case 3: {
                            cout << "请输入要添加的Radar参数文件路径：" << endl;
                            string file_directory_radar;
                            cin.clear();
                            cin.ignore();
                            cin.sync();
                            cin >> file_directory_radar;
                            sensor_manager.add_radar_from_file(file_directory_radar);
                            type = 4;
                            break;
                        }
                        case 4: {
                            file_input = false;
                            cout << "[INFO] 退出添加传感器参数操作界面" << endl;
                            break;
                        }
                        default: {
                            cout << "[ERROR] 输入错误，请重新输入" << endl;
                            cin.clear();
                            cin.ignore();
                            cin.sync();
                            type = 4;
                            break;
                        }
                    }
                }
                break;
            }
            case 2:
            {
                sensor_manager.add_sensor_from_istream();
                break;
            }
            case 3:
            {
                sensor_manager.list_all_sensor();
                break;
            }
            case 4: {
                sensor_manager.list_online_sensor();
                break;
            }
            case 5:{
                cout<<"请输入要查找的传感器id："<<endl;
                int sensor_id;
                string sensor_type;
                cin>>sensor_id;
                sensor_manager.find_sensor(sensor_id,&sensor_type);
                break;
            }
            case 6:{
                cout<<"请输入要删除的传感器id："<<endl;
                int sensor_id;
                cin>>sensor_id;
                // 判断传感器是否存在
                string temp;
                if(sensor_manager.find_sensor(sensor_id,&temp)!=-1){
                    sensor_manager.delete_sensor(sensor_id);
                }
                else{
                    cout<<"[ERROR] 传感器不存在！"<<endl;
                }
                break;
            }
            case 7:{
                //显示某个或多个传感器的ID，并能够按种类显示；统计传感器所在位置
                cout<<"请输入要统计的传感器ID,以空格分隔："<<endl;
                int sensor_id[num_camera+num_lidar+num_radar];
                int count = 0;
                for(int i=0;i<num_camera+num_lidar+num_radar;i++){
                    cin>>sensor_id[i];
                    if(sensor_id[i]==-1){
                        break;
                    }
                    count++;
                }
                sensor_manager.statistic_specified_sensors(sensor_id,count);
                break;
            }
            case 8:{
                sensor_manager.save_parameter_tofile("/Users/alexzheng/Desktop",sensor_manager.get_Car_ID());
                break;
            }

            case 9: {
                cout << "\033[0;33m Developer:alexzheng@tongji.edu.cn; AlexZ 2022. All rights reserved. \033[0m" << endl;
                cout << "\033[0;33m [INFO] 感谢使用智能汽车传感器管理系统，您已退出！\033[0m" << endl;
                exit(0);
            }
        }
    }
}

// 未完成之任务：
// 1.限定参数范围 exception
