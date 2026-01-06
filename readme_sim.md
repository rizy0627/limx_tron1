1. 遥控控制到 /cmd_vel 话题的发送逻辑
在 src/tron1-rl-deploy-ros2/robot_hw/src/PointfootHardwareNode.cpp 文件中，实现了从底层 SDK 接收手柄数据并转换为 /cmd_vel 话题发布的逻辑。
    逻辑位置: subscribeSensorJoyCallback 函数。
    输入来源: 通过 SDK 接口 robot_->subscribeSensorJoy 获取手柄原始数据（轴数据 axes 和 按钮数据 buttons）。
    发送形式: ROS 2 消息类型 geometry_msgs::msg::Twist。
    映射逻辑:
        左摇杆垂直方向 (left_vertical): 控制 线速度 X (Linear X)，即前进/后退。系数为 0.5。
        左摇杆水平方向 (left_horizon): 控制 线速度 Y (Linear Y)，即左右平移。系数为 0.5。
        右摇杆水平方向 (right_horizon): 控制 角速度 Z (Angular Z)，即原地旋转。系数为 0.5。
    特殊指令:
        L1 + Y: 启动控制器 (startController)。
        L1 + X: 停止控制器 (stopController)。
代码片段示意:
    // 摇杆值直接乘以 0.5 转换为速度值 (m/s 或 rad/s)
    twist.linear.x = msg->axes[left_vertical] * 0.5;
    twist.linear.y = msg->axes[left_horizon] * 0.5;
    twist.angular.z = msg->axes[right_horizon] * 0.5;
    md_vel_pub_->publish(twist);

2. 遥控器到机器人的通信方式
这里存在两种不同的通信路径，取决于你是指“实体物理遥控器”还是“上位机/APP控制”。
    物理手柄 (Joystick) -> 机器人:
        通信方式: 通常是 2.4G 无线接收器 或 蓝牙 连接到底层硬件，然后通过 SDK (limxsdk) 将数据通过内部通信（可能是共享内存或UDP）回调给上层 ROS 节点。
        代码体现: PointfootHardwareNode 使用 robot_->subscribeSensorJoy(...) 接收数据。这意味着 ROS 节点并不直接连接手柄，而是“监听”机器人SDK转发过来的手柄数据。
    APP / 脚本 (WebSocket) -> 机器人:
        通信方式: WebSocket (TCP/IP)。
        协议: 根据你提供的 5_上层应用开发接口.md，使用 JSON 格式。
        流程: 客户端（如 websocket_client.cpp）发送 JSON 指令 -> 机器人内部运行的 WebSocket Server (监听 5000 端口) -> Server 解析 JSON -> 内部转换为控制指令（可能模拟成手柄信号，或直接控制运动管理器）。
总结: 在当前的 ROS 2 仿真/部署代码 (pointfoot_hw_sim.launch.py) 中，主要依赖的是 SDK 回调 (模拟物理手柄路径) 或 GUI 工具 (rqt_robot_steering) 直接发布话题，而不是 WebSocket。WebSocket 更多用于外部二次开发接口。

3. /cmd_vel 转换为关节运动控制的完整流程
这是强化学习 (RL) 控制的核心部分。代码位于 src/tron1-rl-deploy-ros2/robot_controllers/src/PointfootController.cpp。
整体流程如下：
    订阅指令 (Subscription):
        控制器内部创建了一个子节点 cmd_vel_node_，订阅 /cmd_vel。
        回调函数: cmdVelCallback。
        处理: 接收到的 Twist 消息会被截断 (Clip) 到 [-1.0, 1.0] 之间，并存入 commands_ 向量中（分别对应 X, Y, Yaw）。
    构建观测向量 (Observation):
        在 handleWalkMode -> computeObservation 中执行。
        RL 策略网络 (Policy Network) 需要一系列“状态”作为输入。其中用户指令是关键的一部分。
        commands_ 会被缩放 (乘以 userCmdCfg 中的配置，如 linVel_x 等)，然后拼接到观测向量 obs 中。
        obs 包含了：基体角速度、重力向量、关节位置/速度、用户指令 (scaled_commands)、步态相位等。
    神经网络推理 (Inference):
        在 computeActions 中执行。
        利用 ONNX Runtime (policySessionPtr_->Run)，将观测向量 obs 输入到预训练好的神经网络 (Policy) 中。
        输出: 网络输出一个 actions_ 向量，这代表了目标关节位置的残差（或特定动作空间的归一化值）。
    转换为关节电机指令 (Action to Joint Command):
        在 handleWalkMode 的最后阶段。
        反归一化: actions_ 经过缩放 (action_scale_pos)。
        PD 控制计算:
            目标位置 pos_des = 当前动作值 + 初始关节角度 (initJointAngles)。
            控制器不直接发电流，而是发送 位置 (Position) + 刚度 (Kp) + 阻尼 (Kd) 给底层驱动器。
        公式:
            $$ \tau = K_p \times (q_{des} - q) + K_d \times (\dot{q}{des} - \dot{q}) + \tau{ff} $$
            代码中通过 this->setJointCommandValue(...) 设置这些参数。