//
// Created by Alex Zheng on 13/6/2022.
//

#ifndef AUTOSENSORMANAGER_SENSORPARAMETER_H
#define AUTOSENSORMANAGER_SENSORPARAMETER_H
#include <iostream>
#include <string>
using namespace std;
// 传感器参数基类
// 数据成员包括id、名称、位置（3维坐标x,y,z），姿态角（roll, pitch, yaw）
// 根据需要，添加其他必要的成员函数，如构造函数、复制构造函数、>> 和<< 运算符重载函数等。
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
    // 拷贝构造函数
    SensorParameter(const SensorParameter& sensor_parameter)
    {
        this->id = sensor_parameter.id;
        this->name = sensor_parameter.name;
        this->x = sensor_parameter.x;
        this->y = sensor_parameter.y;
        this->z = sensor_parameter.z;
        this->roll = sensor_parameter.roll;
        this->pitch = sensor_parameter.pitch;
        this->yaw = sensor_parameter.yaw;
    }
    // 赋值运算符重载函数
    SensorParameter& operator=(const SensorParameter& sensor_parameter)
    {
        this->id = sensor_parameter.id;
        this->name = sensor_parameter.name;
        this->x = sensor_parameter.x;
        this->y = sensor_parameter.y;
        this->z = sensor_parameter.z;
        this->roll = sensor_parameter.roll;
        this->pitch = sensor_parameter.pitch;
        this->yaw = sensor_parameter.yaw;
        return *this;
    }
    // 输出运算符重载函数
    friend ostream& operator<<(ostream& os, const SensorParameter& sensor_parameter)
    {
        os << "id: " << sensor_parameter.id << endl;
        os << "name: " << sensor_parameter.name << endl;
        os << "x: " << sensor_parameter.x << endl;
        os << "y: " << sensor_parameter.y << endl;
        os << "z: " << sensor_parameter.z << endl;
        os << "roll: " << sensor_parameter.roll << endl;
        os << "pitch: " << sensor_parameter.pitch << endl;
        os << "yaw: " << sensor_parameter.yaw << endl;
        return os;
    }
    // 输入运算符重载函数
    friend istream& operator>>(istream& is, SensorParameter& sensor_parameter)
    {
        is >> sensor_parameter.id;
        is >> sensor_parameter.name;
        is >> sensor_parameter.x;
        is >> sensor_parameter.y;
        is >> sensor_parameter.z;
        is >> sensor_parameter.roll;
        is >> sensor_parameter.pitch;
        is >> sensor_parameter.yaw;
        return is;
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


#endif //AUTOSENSORMANAGER_SENSORPARAMETER_H
