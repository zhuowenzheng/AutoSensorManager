//
// Created by Alex Zheng on 13/6/2022.
//

#ifndef AUTOSENSORMANAGER_SENSOR_H
#define AUTOSENSORMANAGER_SENSOR_H

#include<vector>
using namespace std;

// 传感器模板类Sensor<T>
/**
 * 设计传感器模板类Sensor<T>用动态数组模型存取传感器参数，用模板参数 T 作为数组类型，
 * 定义指向由new分配的内存空间的指针。当内存空间大小不够时，则需增加，增加后的内存空间大小是原来的2倍。
 * 设置一个标志成员，用于区分当前传感器对象是否被删除、是否有效等情况，当前标志的值自定。
 * 实现基本的传感器操作函数，包括传感器基本参数的设置及获取，以满足扩展传感器类型的需求；
 * */
template <class T>
class Sensor
{
public:
    Sensor() {
        // 初始化传感器参数vector
        sensor_parameter.reserve(10);
        // 初始化传感器参数数组大小
        sensor_parameter_size = 10;
    }
    ~Sensor() {
        // 删除传感器参数数组指针
        sensor_parameter.clear();
    }
    // 设置传感器参数数组大小
    void setSensorParameterSize(int _sensor_parameter_size) {
        this->sensor_parameter_size = sensor_parameter_size;
        sensor_parameter.reserve(sensor_parameter_size);
    }
    // 获取传感器参数数组大小
    int getSensorParameterSize() {
        return sensor_parameter_size;
    }
    // 获取传感器参数数组实际大小
    int getSensorParameterActualSize() {
        return sensor_parameter.size();
    }
    // 设置传感器参数数组标志
    void setSensorParameterFlag(int _sensor_parameter_flag) {
        this->sensor_parameter_flag = _sensor_parameter_flag;
    }
    // 获取传感器参数数组标志
    int getSensorParameterFlag() {
        return sensor_parameter_flag;
    }
    // 传感器参数数组标志区分当前传感器对象是否被删除、是否有效等情况。未被删除且有效，则为1，否则为0。
    // 数据成员

private:
    vector<T> sensor_parameter;
    int sensor_parameter_size;
    int sensor_parameter_flag;
};


#endif //AUTOSENSORMANAGER_SENSOR_H