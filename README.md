# AutoSensorManager程序文档
同济大学 电子与信息工程学院  
C++面向对象程序设计课设 汽车传感器管理程序  
Alex Zheng

## 1.程序设计框架
### 1.1程序主菜单
系统的主菜单如下所示。包括了如下功能：
 * 从文件、键盘等输入流添加传感器；
 * 列表显示全部或在线的传感器；
 * 查找指定传感器
 * 删除指定传感器
 * 统计传感器参数：可自选按类或按类全部统计；
 * 保存传感器参数到txt文件
 * 修改现有传感器参数
 * 根据参数对传感器筛选排序
 
![终端UI主菜单](https://img-blog.csdnimg.cn/a44e51acbf5845faabd9d4849d57f2cc.png)

### 1.2运行逻辑
  利用循环语句和 `switch-case`结构构建程序主菜单框架，执行`main`函数时通过在终端输入对应功能编号执行菜单项所关联的功能或退出系统。


## 2.传感器参数类
### 2.1 传感器参数基类`SensorParameter`
 `SensorParameter`类中包含以下成员：
  * 默认构造函数`SensorParameter(){}`
  * 基本构造函数`SensorParameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw)`
  * 拷贝构造函数`SensorParameter(const SensorParameter& sensor_parameter)`
  * 析构函数`~SensorParameter(){}`
  * 输出运算符重载函数`friend ostream& operator<<(ostream& os, const SensorParameter& sensor_parameter)`，可用于输出数据成员的值。
  * 输入运算符重载函数`friend istream& operator>>(istream& is, SensorParameter& sensor_parameter)`，可用于为数据成员赋值。
  * 以及数据成员：
      * 传感器ID int `id`
      * 传感器名称 string `name`
      * 传感器位置 double `x,y,z`
      * 传感器姿态角 double `roll,pitch,yaw`
      
  该类的主要目的是派生出Camera, Lidar以及Radar三种种类传感器参数子类。
    
### 2.2 传感器参数派生类
#### 2.2.1 `CameraParameter`类
`CameraParameter`是`SensorParameter`的派生类，继承了`SensorParameter`类的数据成员及成员函数，并有如下专有数据成员：
  * 分辨率 int `resolution_x`,`resolution_y`
  * 帧率 int `frame_rate`
  * 视场角 int `view_angle`, 取值范围[0,180]
  * 畸变参数 float `distortion_parameter[5]`
  * 色彩位数 int `color_bits`
 同时有以下成员函数：
  * 输出运算符重载函数 `friend ostream& operator<<(ostream& os, const CameraParameter& camera_parameter)`，可用于输出包括基类中数据成员和专有数据成员的值。
  * 输入运算符重载函数 `friend istream& operator>>(istream& is, CameraParameter& camera_parameter)`，可用于为所有数据成员赋值。
  * 默认构造函数 `CameraParameter()`
  * 基本构造函数 `CameraParameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw, int line_num, int view_angle, int rotate_rate, int horizontal_view_angle) : SensorParameter(id, name, x, y, z, roll, pitch, yaw)`
  * 拷贝构造函数 `CameraParameter(CameraParameter& cp) : SensorParameter(cp.id, cp.name, cp.x, cp.y, cp.z, cp.roll, cp.pitch, cp.yaw)`
  * 析构函数 `~CameraParameter()`
  

   该类的目的是用于存储单个 `Camera` 的基本参数。
 
#### 2.2.2 `LidarParameter`类
`LidarParameter`是`SensorParameter`的派生类，继承了`SensorParameter`类的数据成员及成员函数，并有如下专有数据成员：
  * 分辨率 int `resolution_x`,`resolution_y`
  * 线数 int `line_num`
  * 视场角 int `view_angle`, 取值范围[0,45]
  * 旋转频率 int `rotate_rate`
  * 水平视场角 int `horizontal_view_angle`, 取值范围[0,360]
 同时有以下成员函数：
  * 输出运算符重载函数 `friend ostream& operator<<(ostream& os, const LidarParameter& lidar_parameter)`，可用于输出包括基类中数据成员和专有数据成员的值。
  * 输入运算符重载函数 `friend istream& operator>>(istream& is, LidarParameter& lidar_parameter)`，可用于为所有数据成员赋值。
  * 默认构造函数 `LidarParameter()`
  * 基本构造函数 `LidarParameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw, int line_num, int view_angle, int rotate_rate, int horizontal_view_angle) : SensorParameter(id, name, x, y, z, roll, pitch, yaw)`
  * 拷贝构造函数 `LidarParameter(LidarParameter& lp) : SensorParameter(lp.id, lp.name, lp.x, lp.y, lp.z, lp.roll, lp.pitch, lp.yaw)`
  * 析构函数 `~LidarParameter()`
  

   该类的目的是用于存储单个 `Lidar` 的基本参数。


#### 2.2.3 `RadarParameter`类
`RadarParameter`是`SensorParameter`的派生类，继承了`SensorParameter`类的数据成员及成员函数，并有如下专有数据成员：
  * 分辨率 int `resolution[2]`
  * 视场角 int `view_angle`，取值范围[0,60]
  * 速度精度 float `speed_accuracy`
  * 探测模式 string `detect_mode`
 同时有以下成员函数：
  * 输出运算符重载函数 `friend ostream& operator<<(ostream& os, const RadarParameter& radar_parameter)`，可用于输出包括基类中数据成员和专有数据成员的值。
  * 输入运算符重载函数 `friend istream& operator>>(istream& is, RadarParameter& radar_parameter)`，可用于为所有数据成员赋值。
  * 默认构造函数 `RadarParameter()`
  * 基本构造函数 `RadarParameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw, const float resolution[2], int view_angle, float speed_accuracy[2], string detect_mode) : SensorParameter(id, name, x, y, z, roll, pitch, yaw)`
  * 拷贝构造函数 `RadarParameter(RadarParameter& rp) : SensorParameter(rp.id, rp.name, rp.x, rp.y, rp.z, rp.roll, rp.pitch, rp.yaw)`
  * 析构函数 `~RadarParameter()`


   该类的目的是用于存储单个 `Radar` 的基本参数。

## 3. 传感器类
### 3.1 传感器模版类 `Sensor<T>`
   传感器类用动态数组模型存取传感器参数，用模板参数 `T` 作为数组类型，定义指向由 `new` 分配的内存空间的指针`T* sensor_parameter`。
  主要的成员函数如下：
* 添加传感器参数函数 `void add_parameter(T sensor_parameter)` 
    当内存空间大小不够时，则需增加，增加后的内存空间大小是原来的2倍。扩容过程在实现过程如下：
```
Sensor<T>::void add_parameter(T sensor_parameter) {
if (sensor_parameter_actual_size == sensor_parameter_size) {
            // 如果传感器参数数组实际大小等于传感器参数数组大小，则需增加传感器参数数组大小
            sensor_parameter_size *= 2;
            T *temp = new T[sensor_parameter_size];
            for (int i = 0; i < sensor_parameter_actual_size; i++) {
                temp[i] = sensor_parameter[i];
            }
            sensor_parameter = temp;
            delete[] temp;
        }
        sensor_parameter[sensor_parameter_actual_size] = sensor_parameter;
}
```
* 删除传感器参数函数 `void delete_parameter(int index)`
   给定下标 `index`,若在动态数组实际大小范围内，则删除对应元素，并将后续元素前一一位。
```
void delete_parameter(int index) {
        if (index < sensor_parameter_actual_size) {
            T *temp = new T[sensor_parameter_size];
            for (int i = 0; i < sensor_parameter_actual_size; i++) {
                if (i < index) {
                    temp[i] = sensor_parameter[i];
                } else if (i > index) {
                    temp[i] = sensor_parameter[i - 1];
                }
            }
            delete[] sensor_parameter;
        }
    }
```
* 设置传感器参数函数 `void set_parameter((int id, string name, double x, double y, double z, double roll, double pitch, double yaw,int index, T sensor_parameter)`
```
 void set_parameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw,int index, T sensor_parameter) {
        // 设置传感器参数
        this->sensor_parameter[index].name = name;
        this->sensor_parameter[index].x = x;
        this->sensor_parameter[index].y = y;
        this->sensor_parameter[index].z = z;
        this->sensor_parameter[index].roll = roll;
        this->sensor_parameter[index].pitch = pitch;
        this->sensor_parameter[index].yaw = yaw;
        this->sensor_parameter[index].id = id;
    }
```
* 显示传感器参数函数 `void show_parameter()`
```
void show_parameter() {
        for (int i = 0; i < sensor_parameter_actual_size; i++) {
            cout << sensor_parameter[i] << " ";
        }
        cout << endl;
    }
```

Sensor<T>中有一数据成员 `is_effective`, 用于记录每个sensor对应的状态。对每个sensor，被删除失效时对应值记为 `-1` ，离线时记为 `0` ，在线正常工作记为 `1` .

模版类中，用于储存数据成员的数据结构也可以重写为简单链表形式：
```
 // 将模板类中用于传感器参数类对象存取的数据模型由动态数组改为简单链表模型
    typedef struct sensor_parameter_node {
        T sensor_parameter_value;
        struct sensor_parameter_node *next;
    } sensor_parameter_node;
```


### 3.2 特化类
基于上述模版类Sensor<T>可特化出三个分别对应三种传感器的特化子类：
  * `Sensor<CameraParameter>`
  * `Sensor<LidarParameter>`
  * `Sensor<RadarParameter>`

三个特化子类的结构组成大体相似，在这里`Sensor<CameraParameter>`为例以统一进行说明，其余两个特化类与其构成基本一致，仅名字和数据处理函数细节由于参数类专有数据成员差别而有些许不同。

`Sensor<CameraParameter>`的数据成员如下：
* CameraParameter `current_parameter`，当前访问的参数类成员；
* int `reserved_sensor_num`, 数组的预留范围，默认预设值为10。同模版类一样，当动态数组储存的参数数目达到上界时，执行空间翻倍操作。
* CameraParameter `*camera_parameter`, 指向储存参数的动态数组，在类成员函数定义中使用new分配空间。
* map<int,int> `is_effective`,状态标志成员。其中键(key)为传感器的ID，值(Value)对应关系： -1 = deleted, 0 = offline, 1 = online。

除去构造函数、拷贝构造函数和析构函数外，特化类的主要成员函数有：
* 添加传感器参数函数 `void add_parameter` ，向参数类动态数组中增加新的传感器参数对象。
```
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
```
* 传感器参数输出函数 `void show_parameter(CameraParameter* cam_param)`，其作用是输出指针对应的动态数组下所有传感器的参数。

* 传感器参数设置函数 `void set_current_parameter(int id, string name, double x, double y, double z, double roll, double pitch, double yaw, int resolution_x, int resolution_y, int frame_rate, int view_angle, int color_bits)`

* 显示当前特化类下的所有传感器的函数 `void list_sensors()`，输出所有传感器的的ID、名称和位置。

* 显示当前特化类下所有在线传感器的函数 `void list_online_sensors()`, 输出在线传感器的ID、名称和位置。

* 传感器参数修改函数 `void modify_parameter(int i)`，其中i为要修改的传感器参数在动态数组中的下标，由传感器管理类的成员函数`int find_sensor(int sensor_id,string* sensor_type)`返回值确定。函数包含了用户交互逻辑命令行界面及`rangeExceedException`中的数据范围异常处理。
```
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
```

* 参数排序显示函数 `void sort_parameter(int param)` ，其中 `param` 为特化类专有参数的标签，在函数中分别指定。通过对可排序的部分参数进行排序，在终端输出排序后的传感器参数降序结果。
```
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
```
 

## 4. 传感器管理类 `SensorManager`

传感器管理类作用的对象单位是车辆，每个类对象负责一辆汽车上的所有传感器成员。它的数据成员有：
* Camera传感器类 Sensor<CameraParameter> `camera_list`;
* Lidar传感器类 Sensor<LidarParameter> `lidar_list`;
* Radar传感器类 Sensor<RadarParameter> `radar_list`;
* 当前车辆传感器总数 int `sensor_num` , 初始预设值为0;
* 车辆名称 string Car_ID;

主要成员函数及其功能如下：
* 构造函数 `SensorManager(string Car_ID)`;
* 析构函数 `~SensorManager()`
* 文件读入参数函数 `void add_camera_from_file(const string& file_path)`, `void add_lidar_from_file(const string& file_path)`,`void add_radar_from_file(const string& file_path)`，分别从存储camera、lidar和radar三种传感器参数的txt文件中读取参数并导入传感器动态数组中。
```
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

```


* 键盘终端输入添加传感器函数 `void add_sensor_from_istream()`，函数包含了用户交互逻辑命令行界面及`rangeExceedException`中的数据范围异常处理。
* 查找传感器接口 `int find_sensor(int id,string* sensor_type)`,调用三个特化类中的查找函数，若找到对应 `id`的传感器，返回对应类型传感器数组中的下标，同时用指针传参返回对应的类型字符串 `sensor_type`；未找到则函数返回 `-1` 。
* 删除传感器接口 `void delete_sensor(int id)`,调用三个特化类中的删除函数，若找到对应 `id`的传感器，将其删除。
* 显示传感器接口 `void list_all_sensor()`,显示所有未被删除的传感器的id、名称及位置信息，在终端输出。
* 显示在线传感器接口 `void list_online_sensor()`,显示所有在线的传感器的id、名称及位置信息，在终端输出。
* 统计指定传感器函数 `void statistic_specified_sensors(vector<int> sensor_id,int count)`，通过`getline`和 `istringstream` 在终端输入流获取某个或多个传感器的id，统计他们的类型及位置信息并在终端输出。
* 分类统计显示传感器参数接口 `void statistic_sensor_parameter()`,依照用户选择分类统计三种类型中每种类型的所有传感器，并在终端打印它们的详细参数。
* 保存参数到文件接口 `void save_parameter_tofile(string file_directory,string carID)`，将现有未被删除的所有传感器以统一格式输出到指定文件目录下，生成三个txt文件，分别对应三种传感器。
```
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
```


* 修改传感器参数接口 `void modify_sensor_parameter(int sensor_id)`，分别调用三个特化类中的修改函数，对指定传感器参数进行修改（名称，位置，参数，在线状态等）。
* 参数排序显示传感器参数接口 `void sort_sensor(int sensor_type_id, int parameter_id)`,分别调用三个特化类中的排序函数，依照某个选定的参数对一类传感器进行排序，并在终端打印结果。
## 5. 异常处理类
### 5.1 数据范围异常处理类 `rangeExceedError`
将std::exception中的虚成员函数what()重写到数据范围异常处理类`rangeExceedError`中, 用于处理参数输入范围限制越界的异常。
```
class rangeExceedException : public std::exception{
public:
    rangeExceedException() noexcept {}
    ~rangeExceedException() override = default;
    const char* what() const noexcept override{
        return "[EXCEPTION] 参数需设置在规定范围内！";
    }
private:
    char* msg;
};

```

### 5.2 其他异常情况的处理
在程序设计过程中，诸如流程逻辑、终端输入选择和其它类型的一些错误已由`switch-case`和`if-else`等结构自然解决。综合开发任务考虑，仅在数据范围异常处理上重写了 `what()` 进行异常处理，后续可考虑进一步完善各种种类的异常处理。
