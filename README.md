![2023-03-16 17-45-21屏幕截图](/home/lu/图片/2023-03-16 17-45-21屏幕截图.png)`

```c++
ros_natnet_client-master
    ├── CMakeLists.txt
    ├── include
    │   ├── Mocap.h
    │   └── NatNet
    │       ├── NatNetCAPI.h
    │       ├── NatNetClient.h
    │       ├── NatNetRepeater.h
    │       ├── NatNetRequests.h
    │       └── NatNetTypes.h
    ├── launch
    │   └── asl_optitrack_vr.launch
    ├── libNatnet
    │   └── libNatNet.so
    ├── package.xml
    ├── README.md
    └── src
        ├── Mocap.cc
        └── ros_natnet_client.cpp
```

### 环境依赖：

fmt安装8.1.1

sophus安装main.1.x
pangolin版本v0.3

直接clone的代码中catkin_simple功能包文件夹为空，需要自行从别的地方下载

clone得到的代码中和catkin_simple同级的cmakelist.txt文件需要删除

### 代码修改：

代码需要在nocap.h的头加入  
#define FMT_HEADER_ONLY
#include "fmt/format.h"
在camkelist.txt里面需要对加入fmt依赖
find_package(FMT REQUIRED)
target_link_libraries(ros_natnet_client Mocap ${catkin_LIBRARIES} NatNet.so ${Pangolin_LIBRARIES}  fmt)

### 其他注意：

libnatnet.so 需要软连接 到usr/lib 而不是直接cp
1.首先建立workspace/src
2.把ros_natnet_client放入src，
3.在src里面catkin_init_worksapce
4.catkin_simple安装src里
5.catkin_build 在workspace下
6.source devel/setup.bash
7.roslaunch ros_natnet_client  *****.launch

auto wlan0
allow-hotplug wlan0 # wlan0 WiFi card number
iface wlan0 inet static
address 192.168.3.247
netmask 255.255.255.0
broadcast 192.168.0.255
gateway 192.168.3.1
dns-nameservers 192.168.3.1
wpa-ssid "longmen-2016" 
wpa-psk "longmen317" 




