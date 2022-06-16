#include <iostream>
#include <string>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>
#include "Sensor.cpp"
#include "SensorParameter.cpp"
#include "rangeExceedException.cpp"
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
 * @version 1.1
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
    // copy constructor
    ~CameraParameter(){}
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
    ~LidarParameter(){}
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
    // copy constructor`

    ~RadarParameter(){}
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
    void show_parameter(CameraParameter* cam_param)
    {
        cout << "\033[0;34m 编号\t名称\t位置\t旋转角度\t分辨率\t帧率\t视场角\t颜色位数 \033[0m" << endl;
        for (int i = 0; i < num_camera; i++)
        {
            cout << cam_param[i].id<<"\t"<<cam_param[i].name << "\t" << cam_param[i].x << "," << cam_param[i].y << "," << cam_param[i].z << "\t" << cam_param[i].roll << "," << cam_param[i].pitch << "," << cam_param[i].yaw << "\t" << cam_param[i].resolution_x << "," << cam_param[i].resolution_y << "\t" << cam_param[i].frame_rate << "\t" << cam_param[i].view_angle << "\t" << cam_param[i].color_bits << endl;
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
        for (int i = 0; i < num_camera;) {
            //未被删除
            if (is_effective[camera_parameter[i].id] != -1 && camera_parameter[i].id != -1) {
                cout << camera_parameter[i].id << "\t" << camera_parameter[i].name << "\t" << camera_parameter[i].x
                     << "," << camera_parameter[i].y << "," << camera_parameter[i].z << endl;
                i++;
            }
        }
    }
    void list_online_sensors(){
        cout<<"\033[0;34m Camera编号\tCamera名称\tCamera位置 \033[0m"<<endl;
        for (int i = 0; i < get_sensor_num();i++)
        {
            // 在线且未被删除
            if(is_effective[camera_parameter[i].id]==1)
            {
                cout << camera_parameter[i].id<<"\t"<<camera_parameter[i].name << "\t" << camera_parameter[i].x << "," << camera_parameter[i].y << "," << camera_parameter[i].z << endl;
                //cout << this->parameters[i].id << "\t" << this->parameters[i].name<<"\t"<<parameters[i].x<<" "<<parameters[i].y<<" "<<parameters[i].z<< endl;
            }
        }
    }

    void modify_parameter(int i) {
        // 选择需要修改的内容
        int choice;
        cout << "请选择需要修改的内容：" << endl;
        cout << "1.名称";
        cout << "2.位置";
        cout << "3.参数";
        cout << "4.在线状态";
        cout << "5.退出"<<endl;
        cin >> choice;
        switch (choice) {
            case 1:
                cout << "原名称："<<camera_parameter[i].name<<endl;
                cout << "请输入新的名称：" << endl;
                cin >> camera_parameter[i].name;
                cout << "修改成功！Camera " << camera_parameter[i].id << "的名称更新为" << camera_parameter[i].name << "!" << endl;
                break;
            case 2:
                cout << "原位置："<<camera_parameter[i].x<<","<<camera_parameter[i].y<<","<<camera_parameter[i].z<<endl;
                cout << "请输入新的位置：" << endl;
                cin >> camera_parameter[i].x >> camera_parameter[i].y >> camera_parameter[i].z;
                cout << "修改成功！Camera " << camera_parameter[i].id << "的位置更新为" << camera_parameter[i].x << ","
                     << camera_parameter[i].y << "," << camera_parameter[i].z << "!" << endl;
                break;
            case 3:{
                cout << " 请输入想要修改的参数编号：" << endl;
                cout << " 1.姿态角(roll,pitch,yaw)" << endl;
                cout << " 2.分辨率(resolution_x,resolution_y)" << endl;
                cout << " 3.帧率(frame_rate)" << endl;
                cout << " 4.视场角(view_angle)" << endl;
                cout << " 5.畸变参数(distortion_parameter)" << endl;
                cout << " 6.颜色位数(color_bits)" << endl;
                cout << " 7.退出" << endl;
                int parameter_id;
                cin >> parameter_id;
                switch (parameter_id) {
                    case 1:
                        cout << "原姿态角：" << camera_parameter[i].roll << "," << camera_parameter[i].pitch << "," << camera_parameter[i].yaw << endl;
                        cout << " 请输入新的姿态角：" << endl;
                        cin >> camera_parameter[i].roll >> camera_parameter[i].pitch >> camera_parameter[i].yaw;
                        cout << " 修改成功！Camera " << camera_parameter[i].id << "的姿态角更新为" << camera_parameter[i].roll
                             << "," << camera_parameter[i].pitch << "," << camera_parameter[i].yaw << "!" << endl;
                        break;
                    case 2:
                        cout << "原分辨率：" << camera_parameter[i].resolution_x << "," << camera_parameter[i].resolution_y << endl;
                        cout << " 请输入新的分辨率：" << endl;
                        cin >> camera_parameter[i].resolution_x >> camera_parameter[i].resolution_y;
                        cout << " 修改成功！Camera " << camera_parameter[i].id << "的分辨率更新为"
                             << camera_parameter[i].resolution_x << "," << camera_parameter[i].resolution_y << "!"
                             << endl;
                        break;
                    case 3:
                        cout << "原帧率：" << camera_parameter[i].frame_rate << endl;
                        cout << " 请输入新的帧率：" << endl;
                        cin >> camera_parameter[i].frame_rate;
                        cout << " 修改成功！Camera " << camera_parameter[i].id << "的帧率更新为" << camera_parameter[i].frame_rate
                             << "!" << endl;
                        break;
                    case 4:{
                        bool flag = true;
                        int view_angle;
                        cout<<"原视场角："<<camera_parameter[i].view_angle<<endl;
                        while(flag){
                            cout << "请输入新的视场角(range：0-180)：";
                            try{
                                cin >> view_angle;
                                if(view_angle < 0 || view_angle > 180){
                                    throw rangeExceedException();
                                }
                                flag = false;
                            }catch (exception& e){
                                cout << e.what() << endl;
                            }
                        }
                        camera_parameter[i].view_angle = view_angle;
                        cout << " 修改成功！Camera " << camera_parameter[i].id << "的视场角更新为" << camera_parameter[i].view_angle<< "!" << endl;
                        break;
                    }
                    case 5:
                        cout << "原畸变参数：" << camera_parameter[i].distortion_parameter << endl;
                        cout << " 请输入新的畸变参数：" << endl;
                        for (float &j: camera_parameter[i].distortion_parameter)
                            cin >> j;
                        cout << " 修改成功！Camera " << camera_parameter[i].id << "的畸变参数更新为"
                             << camera_parameter[i].distortion_parameter << "!" << endl;
                        break;
                    case 6:
                        cout << "原颜色位数：" << camera_parameter[i].color_bits << endl;
                        cout << " 请输入新的颜色位数：" << endl;
                        cin >> camera_parameter[i].color_bits;
                        cout << " 修改成功！Camera " << camera_parameter[i].id << "的颜色位数更新为"
                             << camera_parameter[i].color_bits << "!" << endl;
                        break;
                    case 7:
                        break;
                }
                break;
            }
            case 4:{
              //判断当前在线状态，若在线，则设置为不在线，若不在线，则设置为在线
              string online_status = is_effective[camera_parameter[i].id]==1?"在线":"不在线";
              cout<<"当前在线状态为："<<online_status<<"，是否改变状态？(y/n)"<<endl;
              char c;
              cin>>c;
              if(c=='y'){
                  if (online_status == "在线") {
                      is_effective[camera_parameter[i].id] = 0;
                      cout << "Camera " << camera_parameter[i].id << "的在线状态更新为不在线!" << endl;
                  } else {
                      is_effective[camera_parameter[i].id] = 1;
                      cout << "Camera " << camera_parameter[i].id << "的在线状态更新为在线!" << endl;
                  }
              } else if(c=='n'){
                  cout<<"[INFO] 放弃更改！"<<endl;
                  break;
              } else {
                  cout<<"[ERROR] 输入错误！"<<endl;
                  break;
              }
              break;
            }
            case 5:
                break;
            default:
                cout << "[ERROR] 输入错误！" << endl;
                break;
        }
    }

    void sort_parameter(int param) {
        //按照指定的参数降序排序
        // param: 1.resolution_x,resolution_y 2.frame_rate 3.view_angle 4.color_bits
        CameraParameter *temp_camera_parameter = new CameraParameter;
        temp_camera_parameter = camera_parameter;
        switch (param) {
            case 1:{
                sort(temp_camera_parameter, temp_camera_parameter + num_camera,
                     [](const CameraParameter &a, const CameraParameter &b) {
                         return a.resolution_x*a.resolution_y > b.resolution_x*b.resolution_y;
                     });
                show_parameter(temp_camera_parameter);
                break;
            }
            case 2:{
                sort(temp_camera_parameter, temp_camera_parameter + num_camera,
                     [](const CameraParameter &a, const CameraParameter &b) {
                         return a.frame_rate > b.frame_rate;
                     });
                show_parameter(temp_camera_parameter);
                break;
            }
            case 3:
                sort(temp_camera_parameter, temp_camera_parameter + num_camera,
                     [](const CameraParameter &a, const CameraParameter &b) {
                         return a.view_angle > b.view_angle;
                     });
                break;
            case 4:
                sort(temp_camera_parameter, temp_camera_parameter + num_camera,
                     [](const CameraParameter &a, const CameraParameter &b) {
                         return a.color_bits > b.color_bits;
                     });
                break;
        }
        delete[] temp_camera_parameter;
    }

    CameraParameter *camera_parameter = nullptr;
    map<int,int> is_effective;//-1:deleted 0:offline 1:online

private:
    CameraParameter current_parameter;
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
        delete lidar_parameter;
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
    void show_parameter(LidarParameter* lidar_param)
    {
        cout << "\033[0;34m 编号\t名称\t位置\t旋转角度\t线数\t视场角\t旋转频率\t水平视场角 \033[0m" << endl;
        for (int i = 0; i < num_lidar; i++)
        {
            cout << lidar_param[i].id << "\t" << lidar_param[i].name << "\t" << lidar_param[i].x << "\t" << lidar_param[i].y << "\t" << lidar_param[i].z << "\t" << lidar_param[i].roll << "\t" << lidar_param[i].pitch << "\t" << lidar_param[i].yaw << "\t" << lidar_param[i].view_angle << "\t" << lidar_param[i].horizontal_view_angle << endl;
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
        for (int i = 0; i < num_lidar; i++){
            if(is_effective[lidar_parameter[i].id]==1){
                cout << lidar_parameter[i].id<<"\t"<<lidar_parameter[i].name << "\t" << lidar_parameter[i].x << "," << lidar_parameter[i].y << "," << lidar_parameter[i].z << endl;
            }
        }
    }

    void modify_parameter(int i){
        // 选择需要修改的内容
        int choice;
        cout << "请选择需要修改的内容：" << endl;
        cout << "1.名称";
        cout << "2.位置";
        cout << "3.参数";
        cout << "4.在线状态";
        cout << "5.退出"<<endl;
        cin >> choice;
        switch (choice)
        {
        case 1:
            cout << "原名称：" << lidar_parameter[i].name << endl;
            cout << "请输入新的名称：" << endl;
            cin >> lidar_parameter[i].name;
            cout << "修改成功！Lidar"<<lidar_parameter[i].id<<"的名称更新为：" << lidar_parameter[i].name << endl;
            break;
        case 2:
            cout << "原位置：" << lidar_parameter[i].x << "," << lidar_parameter[i].y << "," << lidar_parameter[i].z << endl;
            cout << "请输入新的位置：" << endl;
            cin >> lidar_parameter[i].x >> lidar_parameter[i].y >> lidar_parameter[i].z;
            cout << "修改成功！Lidar"<<lidar_parameter[i].id<<"的位置更新为：" << lidar_parameter[i].x << "," << lidar_parameter[i].y << "," << lidar_parameter[i].z << endl;
            break;
        case 3:{
            cout << " 请输入想要修改的参数编号：" << endl;
            cout << " 1.姿态角(roll,pitch,yaw)" << endl;
            cout << " 2.线数(line_num)" << endl;
            cout << " 3.视场角(view_angle)" << endl;
            cout << " 4.旋转频率(rotate_rate)" << endl;
            cout << " 5.水平视场角(horizontal_view_angle)" << endl;
            cout << " 6.退出" << endl;
            int parameter_id;
            cin >> parameter_id;
            switch (parameter_id)
            {
            case 1:
                cout << "原姿态角：" << lidar_parameter[i].roll << "," << lidar_parameter[i].pitch << "," << lidar_parameter[i].yaw << endl;
                cout << "请输入新的姿态角：" << endl;
                cin >> lidar_parameter[i].roll >> lidar_parameter[i].pitch >> lidar_parameter[i].yaw;
                cout << "修改成功！Lidar"<<lidar_parameter[i].id<<"的姿态角更新为：" << lidar_parameter[i].roll << "," << lidar_parameter[i].pitch << "," << lidar_parameter[i].yaw << endl;
                break;
            case 2:
                cout << "原线数：" << lidar_parameter[i].line_num << endl;
                cout << "请输入新的线数：" << endl;
                cin >> lidar_parameter[i].line_num;
                cout << "修改成功！Lidar"<<lidar_parameter[i].id<<"的线数更新为：" << lidar_parameter[i].line_num << endl;
                break;
            case 3:{
                bool flag = true;
                int view_angle;
                cout<<"原视场角："<<lidar_parameter[i].view_angle<<endl;
                while(flag){
                    cout << "请输入新的视场角(range：0-45)：";
                    try{
                        cin >> view_angle;
                        if(view_angle < 0 || view_angle > 45){
                            throw rangeExceedException();
                        }
                        flag = false;
                    }catch (exception& e){
                        cout << e.what() << endl;
                    }
                }
                lidar_parameter[i].view_angle = view_angle;
                cout << "修改成功！Lidar"<<lidar_parameter[i].id<<"的视场角更新为：" << lidar_parameter[i].view_angle << endl;
                break;
            }
            case 4:
                cout << "原旋转频率：" << lidar_parameter[i].rotate_rate << endl;
                cout << "请输入新的旋转频率：" << endl;
                cin >> lidar_parameter[i].rotate_rate;
                cout << "修改成功！Lidar"<<lidar_parameter[i].id<<"的旋转频率更新为：" << lidar_parameter[i].rotate_rate << endl;
                break;
            case 5:{
                bool flag = true;
                int horizontal_view_angle;
                cout<<"原水平视场角："<<lidar_parameter[i].horizontal_view_angle<<endl;
                while(flag){
                    cout << "请输入新的水平视场角(range：0-360)：";
                    try{
                        cin >> horizontal_view_angle;
                        if(horizontal_view_angle < 0 || horizontal_view_angle > 360){
                            throw rangeExceedException();
                        }
                        flag = false;
                    }catch (exception& e){
                        cout << e.what() << endl;
                    }
                }
                lidar_parameter[i].horizontal_view_angle = horizontal_view_angle;
                cout << "修改成功！Lidar"<<lidar_parameter[i].id<<"的水平视角更新为：" << lidar_parameter[i].horizontal_view_angle << endl;
                break;
            }
            case 6:
                break;
            default:
                cout << "[ERROR] 输入错误！" << endl;
                break;
            }
            break;
        }
        case 4:{
            string online_status = is_effective[lidar_parameter[i].id]==1?"在线":"不在线";
            cout<<"当前在线状态为："<<online_status<<"，是否改变状态？(y/n)"<<endl;
            char c;
            cin>>c;
            if(c=='y'){
                if (online_status == "在线") {
                    is_effective[lidar_parameter[i].id] = 0;
                    cout << "Lidar " << lidar_parameter[i].id << "的在线状态更新为不在线!" << endl;
                } else {
                    is_effective[lidar_parameter[i].id] = 1;
                    cout << "Lidar " << lidar_parameter[i].id << "的在线状态更新为在线!" << endl;
                }
            }else if(c=='n'){
                cout<<"[INFO] 放弃更改！"<<endl;
                break;
            }else {
                cout<<"[ERROR] 输入错误！"<<endl;
                break;}
            break;
        }
        case 5:
            break;
        default:
            cout << "[ERROR] 输入错误！" << endl;
            break;
        }
    }

    LidarParameter* lidar_parameter = nullptr;
    map<int,int> is_effective;

    void sort_parameter(int param){
        //按照指定的参数降序排序
        // param: 1.线数 2.视场角 3.旋转频率 4.水平视场角
        LidarParameter* temp_lidar_parameter = new LidarParameter;
        temp_lidar_parameter = lidar_parameter;
        switch (param) {
            case 1: {
                sort(temp_lidar_parameter, temp_lidar_parameter + num_lidar, [](LidarParameter a, LidarParameter b) {
                    return a.line_num > b.line_num;
                });
                show_parameter(temp_lidar_parameter);
                break;
            }
            case 2:{
                sort(temp_lidar_parameter, temp_lidar_parameter + num_lidar, [](LidarParameter a, LidarParameter b) {
                    return a.view_angle > b.view_angle;
                });
                show_parameter(temp_lidar_parameter);
                break;
            }
            case 3:{
                sort(temp_lidar_parameter, temp_lidar_parameter + num_lidar, [](LidarParameter a, LidarParameter b) {
                    return a.rotate_rate > b.rotate_rate;
                });
                show_parameter(temp_lidar_parameter);
                break;
            }
            case 4:{
                sort(temp_lidar_parameter, temp_lidar_parameter + num_lidar, [](LidarParameter a, LidarParameter b) {
                    return a.horizontal_view_angle > b.horizontal_view_angle;
                });
                show_parameter(temp_lidar_parameter);
            }
        }
        delete temp_lidar_parameter;
    }

private:
    LidarParameter current_parameter;
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
    void show_parameter(RadarParameter* radar_param) {

        cout << "\033[0;34m 编号\t名称\t位置\t旋转角度\t分辨率\t视场角\t速度精度\t探测模式 \033[0m" << endl;
        for (int i = 0; i < num_radar; i++) {

            cout << radar_param[i].id << "\t" << radar_param[i].name << "\t" << radar_param[i].x << "\t"
                 << radar_param[i].y << "\t" << radar_param[i].z << "\t" << radar_param[i].roll << "\t"
                 << radar_param[i].pitch << "\t" << radar_param[i].yaw << "\t" << radar_param[i].resolution[0]<<","<<radar_param[i].resolution[1]
                 << "\t" << radar_param[i].view_angle << "\t" << radar_param[i].speed_accuracy[0]<<","<<radar_param[i].speed_accuracy[1] << "\t"
                 << radar_param[i].detect_mode << endl;
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
    //返回radar总数
    int get_sensor_num(){
        return num_radar;
    }
    //获取在线radar数目
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
    //列出所有radar
    void list_sensors(){
        cout<<"\033[0;34m Radar编号\tRadar名称\tRadar位置 \033[0m"<<endl;
        for(int i = 0;i<num_radar;){
            if(is_effective[radar_parameter[i].id]==1){
                cout << radar_parameter[i].id<<"\t"<<radar_parameter[i].name << "\t" << radar_parameter[i].x << "," << radar_parameter[i].y << "," << radar_parameter[i].z << endl;
                i++;
            }
        }
    }
    //列出在线的radar
    void list_online_sensors(){
        cout<<"\033[0;34m Radar编号\tRadar名称\tRadar位置 \033[0m"<<endl;
        for(int i = 0;i<num_radar;i++){
            if(is_effective[radar_parameter[i].id]==1){
                cout<<radar_parameter[i].id<<"\t"<<radar_parameter[i].name<<"\t"<<radar_parameter[i].x<<","<<radar_parameter[i].y<<","<<radar_parameter[i].z<<endl;

            }
        }
    }
    //修改radar参数
    void modify_parameter(int i){
        int choice;
        cout << "请选择需要修改的内容：" << endl;
        cout << "1.名称";
        cout << "2.位置";
        cout << "3.参数";
        cout << "4.在线状态";
        cout << "5.退出"<<endl;
        cin >> choice;
        switch(choice){
            case 1:
                cout << "原名称：" << radar_parameter[i].name << endl;
                cout << "请输入新的名称：" << endl;
                cin >> radar_parameter[i].name;
                cout << "修改成功！Radar "<<radar_parameter[i].id<<"的名称更新为"<<radar_parameter[i].name<<endl;
                break;
            case 2:
                cout << "原位置：" << radar_parameter[i].x << "," << radar_parameter[i].y << "," << radar_parameter[i].z << endl;
                cout << "请输入新的位置：" << endl;
                cin >> radar_parameter[i].x >> radar_parameter[i].y >> radar_parameter[i].z;
                cout << "修改成功！Radar "<<radar_parameter[i].id<<"的位置更新为"<<radar_parameter[i].x<<","<<radar_parameter[i].y<<","<<radar_parameter[i].z<<endl;
                break;
            case 3:
                cout << " 请输入想要修改的参数编号：" << endl;
                cout << " 1.姿态角(roll,pitch,yaw)" << endl;
                cout << " 2.分辨率(x,y)" << endl;
                cout << " 3.视场角(view_angle)" << endl;
                cout << " 4.速度精度(x,y)" << endl;
                cout << " 5.检测模式(detect_mode)" << endl;
                cout << " 6.退出" << endl;
                int parameter_id;
                cin >> parameter_id;
                switch(parameter_id){
                    case 1:
                        cout << "原姿态角：" << radar_parameter[i].roll << "," << radar_parameter[i].pitch << "," << radar_parameter[i].yaw << endl;
                        cout << "请输入新的姿态角：" << endl;
                        cin >> radar_parameter[i].roll >> radar_parameter[i].pitch >> radar_parameter[i].yaw;
                        cout << "修改成功！Radar "<<radar_parameter[i].id<<"的姿态角更新为"<<radar_parameter[i].roll<<","<<radar_parameter[i].pitch<<","<<radar_parameter[i].yaw<<endl;
                        break;
                    case 2:
                        cout << "原分辨率：" << radar_parameter[i].resolution[0] << "," << radar_parameter[i].resolution[1] << endl;
                        cout << "请输入新的分辨率：" << endl;
                        cin >> radar_parameter[i].resolution[0] >> radar_parameter[i].resolution[1];
                        cout << "修改成功！Radar "<<radar_parameter[i].id<<"的分辨率更新为"<<radar_parameter[i].resolution[0]<<","<<radar_parameter[i].resolution[1]<<endl;
                        break;
                    case 3:{
                        bool flag = true;
                        int view_angle;
                        cout << "原视场角：" << radar_parameter[i].view_angle << endl;
                        while(flag){
                            cout << "请输入新的视场角(range：0-60)：";
                            try{
                                cin >> view_angle;
                                if(view_angle < 0 || view_angle > 60){
                                    throw rangeExceedException();
                                }
                                flag = false;
                            }catch (exception& e){
                                cout << e.what() << endl;
                            }
                        }
                        cin >> radar_parameter[i].view_angle;
                        cout << "修改成功！Radar "<<radar_parameter[i].id<<"的视场角更新为"<<radar_parameter[i].view_angle<<endl;
                        break;
                    }
                    case 4:
                        cout << "原速度精度：" << radar_parameter[i].speed_accuracy[0] << "," << radar_parameter[i].speed_accuracy[1] << endl;
                        cout << "请输入新的速度精度：" << endl;
                        cin >> radar_parameter[i].speed_accuracy[0] >> radar_parameter[i].speed_accuracy[1];
                        cout << "修改成功！Radar "<<radar_parameter[i].id<<"的速度精度更新为"<<radar_parameter[i].speed_accuracy[0]<<","<<radar_parameter[i].speed_accuracy[1]<<endl;
                        break;
                    case 5:
                        cout << "原检测模式：" << radar_parameter[i].detect_mode << endl;
                        cout << "请输入新的检测模式：" << endl;
                        cin >> radar_parameter[i].detect_mode;
                        cout << "修改成功！Radar "<<radar_parameter[i].id<<"的检测模式更新为"<<radar_parameter[i].detect_mode<<endl;
                        break;
                    case 6:
                        break;
                    default:
                        cout << "[ERROR] 输入错误！" << endl;
                        break;
                }
                break;
            case 5:{
                string online_status = is_effective[radar_parameter[i].id]==1?"在线":"不在线";
                cout<<"当前在线状态为："<<online_status<<"，是否改变状态？(y/n)"<<endl;
                char c;
                cin>>c;
                if(c=='y'){
                    if (online_status == "在线") {
                        is_effective[radar_parameter[i].id] = 0;
                        cout << "Radar " << radar_parameter[i].id << "的在线状态更新为不在线!" << endl;
                    } else {
                        is_effective[radar_parameter[i].id] = 1;
                        cout << "Radar " << radar_parameter[i].id << "的在线状态更新为在线!" << endl;
                    }
                }else if(c=='n'){
                    cout<<"[INFO] 放弃更改！"<<endl;
                    break;
                }else {
                    cout<<"[ERROR] 输入错误！"<<endl;
                    break;}
                break;
            }
            case 6:
                break;
            default:
                cout << "[ERROR] 输入错误！" << endl;
                break;
        }
    }

    RadarParameter *radar_parameter = nullptr;
    map<int,int> is_effective; //标志，由于指针在Parameter上，故用map实现
    void sort_parameter(int param){
        //按照指定的参数降序排序
        // param: 1.分辨率 2.视场角
        RadarParameter* temp_radar_parameter = new RadarParameter;
        temp_radar_parameter = radar_parameter;
        switch (param) {
            case 1:{
                sort(temp_radar_parameter,temp_radar_parameter+num_radar,[](const RadarParameter &a,const RadarParameter &b){
                    return a.resolution[0]*a.resolution[1]>b.resolution[0]*b.resolution[1];
                });
                show_parameter(temp_radar_parameter);
                break;
            }
            case 2:{
                sort(temp_radar_parameter,temp_radar_parameter+num_radar,[](const RadarParameter &a,const RadarParameter &b) {
                    return a.view_angle > b.view_angle;
                });
                show_parameter(temp_radar_parameter);
                break;
            }
        }
        delete temp_radar_parameter;
    };

private:
    RadarParameter current_parameter;
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
    SensorManager(){}
    SensorManager(string Car_ID)
    {
        this->Car_ID = Car_ID;
        this->sensor_num = 0;
    }
    ~SensorManager(){}

    //将打开的文件in定位到line行
    static ifstream & seek_to_line(ifstream & in, int line){
        int i;
        char buf[1024];
        in.seekg(0, ios::beg);  //定位到文件开始。
        for (i = 0; i < line; i++)
        {
            in.getline(buf, sizeof(buf));//读取行。
        }
        return in;
    }
    // 文件导入camera
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
            cout<<"\033[0;32m 成功添加下列传感器 \033[0m"<<endl;
            cout<<"[Camera] "<<temp_camera.id<<":"<<temp_camera.name<<endl;
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
            cout<<"\033[0;32m 成功添加下列传感器 \033[0m"<<endl;
            cout<<"[Lidar] "<<temp_lidar.id<<":"<<temp_lidar.name<<endl;
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
            cout<<"\033[0;32m 成功添加下列传感器 \033[0m"<<endl;
            cout<<"[Radar] "<<temp_radar.id<<":"<<temp_radar.name<<endl;
            radar_list.add_parameter(temp_radar);
        }
        file.close();
        cout<<"\033[0;32m 文件关闭 \033[0m"<<endl;
    }
    // 从键盘输入流新增传感器
    void add_sensor_from_istream()
    {
        int id;
        string name;
        float x, y, z;
        float roll, pitch, yaw;
        int sensor_type;
        cout << "请输入传感器类型（1:Camera;2:Lidar;3:Radar）：";
        cin >> sensor_type;
        switch (sensor_type) {
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
            //视场角范围：0-180 try throw-exception if out of range
            bool flag = true;
            while(flag){
                cout << "请输入传感器的视场角(range：0-180)：";
                try{
                    cin >> view_angle;
                    if(view_angle < 0 || view_angle > 180){
                        throw rangeExceedException();
                    }
                    flag = false;
                }catch (exception& e){
                    cout << e.what() << endl;
                }
            }
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
            bool flag1 = true;
            while(flag1){
                cout << "请输入传感器的视场角(range：0-45)：";
                try{
                    cin >> view_angle;
                    if(view_angle < 0 || view_angle > 45){
                        throw rangeExceedException();
                    }
                    flag1 = false;
                }catch (exception& e){
                    cout << e.what() << endl;
                }
            }
            cout << "请输入传感器的旋转频率：";
            cin >> rotation_rate;
            bool flag2 = true;
            while(flag2){
                cout << "请输入传感器的水平视场角(range：0-360)：";
                try{
                    cin >> horizontal_view_angle;
                    if(horizontal_view_angle < 0 || horizontal_view_angle > 360){
                        throw rangeExceedException();
                    }
                    flag2 = false;
                }catch (exception& e){
                    cout << e.what() << endl;
                }
            }
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
            bool flag3 = true;
            while(flag3){
                cout << "请输入传感器的视场角(range：0-60)：";
                try{
                    cin >> view_angle;
                    if(view_angle < 0 || view_angle > 45){
                        throw rangeExceedException();
                    }
                    flag3 = false;
                }catch (exception& e){
                    cout << e.what() << endl;
                }
            }
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
        // id 具有唯一性

        //camera
        for(int i=0;i<camera_list.get_sensor_num();i++)
        {
            if(camera_list.camera_parameter[i].id==id)
            {
                *sensor_type = "camera";
                return i;
            }
        }

        //lidar
        for(int i=0;i<lidar_list.get_sensor_num();i++)
        {
            if(lidar_list.lidar_parameter[i].id==id)
            {
                *sensor_type = "lidar";
                return i;
            }
        }
        for(int i=0;i<radar_list.get_sensor_num();i++)
        {
            if(radar_list.radar_parameter[i].id==id)
            {
                *sensor_type = "radar";
                return i;
            }
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

                //数组移动
                for(int j = i; j < camera_list.get_sensor_num(); j++)
                {
                    camera_list.camera_parameter[j] = camera_list.camera_parameter[j + 1];
                }

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

                for(int j = i; j < lidar_list.get_sensor_num(); j++)
                {
                    lidar_list.lidar_parameter[j] = lidar_list.lidar_parameter[j + 1];
                }
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

                for(int j = i; j < radar_list.get_sensor_num(); j++)
                {
                    radar_list.radar_parameter[j] = radar_list.radar_parameter[j + 1];
                }
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
    void statistic_specified_sensors(vector<int> sensor_id,int count){
        cout<<"-----------------传感器统计-----------------"<<endl;
        cout<<"统计传感器："<<endl;
        cout<<count<<endl;
        string types[count];
        int camera_num = 0;
        int lidar_num = 0;
        int radar_num = 0;
        for(int i =0;i<count;i++){
            find_sensor(sensor_id[i],&types[i]);
            cout<<"传感器"<<sensor_id[i]<<"："<<types[i]<<endl;
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
            //enumerate all camera
            for(int i =0;i<sensor_num;i++){
                for(int n =0; n<count;n++)
                    if(types[n]=="camera"&&camera_list.camera_parameter[i].id!=0)
                    {
                        cout<<camera_list.camera_parameter[i].id<<"\t"<<camera_list.camera_parameter[i].name<<"\t";
                        cout<<"("<<camera_list.camera_parameter[i].x<<","<<camera_list.camera_parameter[i].y<<","<<camera_list.camera_parameter[i].z<<")"<<endl;
                        break;
                    }
            }
        }
        if(lidar_num>0){
            cout<<"Lidar:"<<lidar_num<<"个"<<endl;
            cout<<"ID"<<"\t"<<"名称"<<"\t"<<"位置"<<endl;
            for(int j = 0;j<sensor_num;j++){
                for(int n =0;n<count;n++){
                    if(types[n]=="lidar"&&lidar_list.lidar_parameter[j].id!=0)
                    {
                        cout<<lidar_list.lidar_parameter[j].id<<"\t"<<lidar_list.lidar_parameter[j].name<<"\t";
                        cout<<"("<<lidar_list.lidar_parameter[j].x<<","<<lidar_list.lidar_parameter[j].y<<","<<lidar_list.lidar_parameter[j].z<<")"<<endl;
                        break;
                    }
                }
            }
        }
        if(radar_num>0){
            cout<<"Radar:"<<radar_num<<"个"<<endl;
            cout<<"ID"<<"\t"<<"名称"<<"\t"<<"位置"<<endl;
            for(int k = 0;k<sensor_num;k++){
                for(int n = 0;n<count;n++)
                    if(types[n]=="radar"&&radar_list.radar_parameter[k].id!=0)
                    {
                       cout<<radar_list.radar_parameter[k].id<<"\t"<<radar_list.radar_parameter[k].name<<"\t";
                       cout<<"("<<radar_list.radar_parameter[k].x<<","<<radar_list.radar_parameter[k].y<<","<<radar_list.radar_parameter[k].z<<")"<<endl;
                       break;
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
            camera_list.show_parameter(camera_list.camera_parameter);
            cout<<"----------------------------------END-----------------------------------"<<endl<<endl;
            break;
        }

        case 2:
        {
            cout<<"---------------------------------Lidar---------------------------------"<<endl;
            lidar_list.show_parameter(lidar_list.lidar_parameter);
            cout<<"----------------------------------END-----------------------------------"<<endl<<endl;
            break;
        }
        case 3:
        {

            cout<<"---------------------------------Radar---------------------------------"<<endl;
            radar_list.show_parameter(radar_list.radar_parameter);
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
                outfile << camera_list.camera_parameter[i];
                i++;
            }
        }
        outfile.close();
        cout<<"[INFO] 保存Camera参数成功！文件路径："<< file_directory_prefix+"camera.txt" <<endl;
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
        cout<<"[INFO] 保存Lidar参数成功！文件路径："<< file_directory_prefix+"lidar.txt" <<endl;
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
        cout<<"[INFO] 保存Radar参数成功！文件路径："<< file_directory_prefix+"radar.txt" <<endl;
    }

    // 修改传感器参数，需将原有参数显示出来，然后选择修改输入新的参数
    void modify_sensor_parameter(int sensor_id) {
        cout << "-----------------修改传感器参数-----------------" << endl;
        string sensor_type;
        int sensor_serial_num = find_sensor(sensor_id, &sensor_type);
        if(sensor_serial_num == -1) {
            cout << "[ERROR] 传感器ID不存在！" << endl;
            return;
        }
        if(sensor_type == "camera") {
            camera_list.modify_parameter(sensor_serial_num);
        } else if(sensor_type == "lidar") {
            lidar_list.modify_parameter(sensor_serial_num);
        } else if(sensor_type == "radar") {
            radar_list.modify_parameter(sensor_serial_num);
        }
    }
    string get_Car_ID(){
        return Car_ID;
    }

    void sort_sensor(int sensor_type_id, int parameter_id){
        /**
         * @param sensor_type_id: 传感器类型ID
         * @param parameter_id: 传感器参数ID
         * @brief 将传感器类型ID和传感器参数ID作为排序条件，对传感器参数进行排序并显示
         * @return 无
         * @algorithm 快速排序
         * */
         cout<<"-----------------传感器参数排序-----------------"<<endl;
         switch(sensor_type_id){
             case 1:{
                 camera_list.sort_parameter(parameter_id);
                 break;
             }
             case 2:{
                 lidar_list.sort_parameter(parameter_id);
                 break;
             }
             case 3:{
                 radar_list.sort_parameter(parameter_id);
                 break;
             }
         }
    }

private:
    Sensor<CameraParameter> camera_list;
    Sensor<LidarParameter> lidar_list;
    Sensor<RadarParameter> radar_list;
    int sensor_num = 0;
    string Car_ID;
};

//用户主界面
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
    cout<<"| 5.查找指定传感器                       |"<<endl;
    cout<<"| 6.删除指定传感器                       |"<<endl;
    cout<<"| 7.指定传感器参数统计                    |"<<endl;
    cout<<"| 8.传感器参数分类统计                    |"<<endl;
    cout<<"| 9.保存传感器参数到文件                  |"<<endl;
    cout<<"| 10.修改传感器参数                      |"<<endl;
    cout<<"| 11.根据参数对传感器筛选排序              |"<<endl;
    cout<<"| 12.退出系统                           |"<<endl;
    cout<<"---------------------------------------"<<endl;
}

//快速测试接口：在此放置需要测试的内容
void fast_test_api(SensorManager sensor_manager){
    sensor_manager.add_camera_from_file("/Users/alexzheng/Desktop/Default-Car_camera.txt");
    sensor_manager.add_lidar_from_file("/Users/alexzheng/Desktop/Default-Car_lidar.txt");
    sensor_manager.add_radar_from_file("/Users/alexzheng/Desktop/Default-Car_radar.txt");
}

int main() {
    SensorManager sensor_manager("Default-Car");
    //fast_test_api(sensor_manager);//测试用
    while (1) {
        int choice = 0;
        ui_initialize();
        cout << "请输入您的选择：";
        cin >> choice;
        switch (choice) {
            // 从文件中添加传感器参数
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
            // 从键盘输入流添加传感器参数
            case 2:
            {
                sensor_manager.add_sensor_from_istream();
                break;
            }
            // 显示所有传感器
            case 3:
            {
                sensor_manager.list_all_sensor();
                break;
            }
            // 显示所有在线传感器
            case 4: {
                sensor_manager.list_online_sensor();
                break;
            }
            //查找传感器
            case 5:{
                cout<<"请输入要查找的传感器id："<<endl;
                int sensor_id;
                string sensor_type;
                cin>>sensor_id;
                sensor_manager.find_sensor(sensor_id,&sensor_type);
                if(sensor_type=="camera")
                    cout<<"查找到传感器"<<sensor_id<<":Camera"<<endl;
                else if(sensor_type=="lidar")
                    cout<<"查找到传感器"<<sensor_id<<":Lidar"<<endl;
                else if(sensor_type=="radar"){
                    cout<<"查找到传感器"<<sensor_id<<":Radar"<<endl;
                }
                else{
                    cout<<"[ERROR] 查找失败，该传感器不存在"<<endl;
                }
                break;
            }
            //删除传感器
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
            //显示某个或多个传感器的ID，并能够按种类显示；统计传感器所在位置
            case 7:{

                cout<<"请输入要统计的传感器ID,以空格分隔："<<endl;
                vector<int> sensor_id_list;
                int count = 0;
                string line;
                cin.sync();
                cin.clear();
                cin.ignore();
                getline(cin,line);
                cout<<line<<endl;
                istringstream ss(line);
                string temp;
                while(getline(ss,temp,' ')){
                    sensor_id_list.push_back(stoi(temp));
                    count++;
                }
                // 统计选定传感器
                sensor_manager.statistic_specified_sensors(sensor_id_list,count);
                break;
            }
            //按种类显示统计该种类所有传感器参数详情
            case 8:{

                sensor_manager.statistic_sensor_parameter();
                break;
            }
            //保存传感器参数到文件，并显示保存的路径，分类保存
            case 9:{
                sensor_manager.save_parameter_tofile("/Users/alexzheng/Desktop",sensor_manager.get_Car_ID());
                break;
            }
            //修改传感器参数
            case 10:{

                cout<<"请输入要修改的传感器id："<<endl;
                int sensor_to_change_id;
                cin>>sensor_to_change_id;
                string sensor_to_change_type;
                sensor_manager.modify_sensor_parameter(sensor_to_change_id);
                break;
            }
            //根据指定参数对传感器排序
            case 11:{
                int sensor_type,sensor_parameter;
                cout<<"请输入要筛选的传感器种类："<<endl;
                cout<<"1.Camera 2.Lidar 3.Radar";
                cin>>sensor_type;
                cout<<"请输入要筛选的传感器参数："<<endl;
                switch(sensor_type){
                    case 1: {
                        cout<<"1.分辨率 2.帧率 3.视场角 4.颜色位数"<<endl;
                        cin>>sensor_parameter;
                        sensor_manager.sort_sensor(sensor_type,sensor_parameter);
                        break;
                    }
                    case 2:{
                        cout<<"1.线数 2.视场角 3.旋转频率 4.水平视场角"<<endl;
                        cin>>sensor_parameter;
                        sensor_manager.sort_sensor(sensor_type,sensor_parameter);
                        break;
                    }
                    case 3:{
                        cout<<"1.分辨率 2.视场角"<<endl;
                        cin>>sensor_parameter;
                        sensor_manager.sort_sensor(sensor_type,sensor_parameter);
                        break;
                    }
                    default:{
                        cout<<"[ERROR] 传感器种类输入错误！"<<endl;
                        cin.clear();
                        cin.sync();
                        cin.ignore();
                        break;
                    }
                }
                break;
            }
            //系统退出入口
            case 12:{
                // 确认退出
                cout << "\033[0;33m 确认退出？如新加入的参数未保存您可能丢失参数数据，请确认保存传感器参数后再退出！(y/n) :\033[0m";
                char confirm;
                cin >> confirm;
                if (confirm == 'y') {
                    cout << "\033[0;33m [INFO] 感谢使用智能汽车传感器管理系统，您已退出！\033[0m" << endl;
                    cout << "\033[0;33m Developer:alexzheng@tongji.edu.cn; Alex Zheng 2022. All rights reserved. \033[0m" << endl;
                    exit(0);
                } else if (confirm == 'n') {
                    cout << "\033[0;33m [INFO] 取消退出 \033[0m" << endl;
                    break;
                } else {
                    cout << "\033[0;33m [ERROR] 输入错误，请重新输入 \033[0m" << endl;
                    break;
                }
            }
        }
    }
}
//Last Modified Date 2022.6.16