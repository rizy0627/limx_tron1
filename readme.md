# 编译
cd ~/planner/tron1/limx_ws
source /opt/ros/humble/setup.bash
export PATH=/usr/bin:$PATH
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 设置机器人
export ROBOT_TYPE=WF_TRON1B
    ros2 launch pointfoot_gazebo empty_world.launch.py
## rl
    source /opt/ros/humble/setup.bash
    source /usr/share/gazebo/setup.bash
    source install/setup.bash
    ros2 launch robot_hw pointfoot_hw_sim.launch.py
## 打开一个终端，运行机器人算法软件。如机器人摔倒状态，通过虚拟遥控器组合按键L2 + Y恢复。请根据机器人类型选择对应的启动命令：
### 为仿真需要，设置一个假SN
        setRobotSN WF_TRON1B_001
        setRobotSN PF_TRON1A_001
### 启动算法软件
        mroslaunch ${MROS_ETC_PATH}/tron1_controllers/tron1a_wheelfoot_controllers_sim.launch 
## 打开一个终端，下载并运行虚拟遥控器
### 运行虚拟遥控器
    cd ~/planner/tron1/limx_ws
    LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 ./robot-joystick/robot-joystick

# ctrl_ros2
## 编译
    cd ~/tron1_ws
    colcon build --packages-select tron1_ctrl_ros2
    source install/setup.bash
## 运行
    ros2 launch tron1_ctrl_ros2 tron1.launch.py

# 协议接口调用
## c++
### 安装依赖
        sudo apt-get install libboost-all-dev libwebsocketpp-dev nlohmann-json3-dev
### 编译代码
        g++ -std=c++11 -o websocket_client websocket_client.cpp -lssl -lcrypto -lboost_system -lpthread
### 运行程序
        ./websocket_client


