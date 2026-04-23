#include <ros/ros.h>
#include "PX4CtrlFSM.h"
#include <signal.h>
#include <std_msgs/String.h>

// 全局指针用于参数更新回调
Parameter_t* g_param_ptr = nullptr;

// JSON解析辅助函数
double parseJsonDouble(const std::string& json, const std::string& key, double default_val) {
    std::string search = "\"" + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return default_val;
    pos += search.length();
    // 跳过空格
    while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    size_t end = pos;
    while (end < json.length() && (isdigit(json[end]) || json[end] == '.' || json[end] == '-')) end++;
    if (end > pos) {
        return std::stod(json.substr(pos, end - pos));
    }
    return default_val;
}

// 动态参数更新回调
void paramUpdateCallback(const std_msgs::String::ConstPtr& msg) {
    if (g_param_ptr == nullptr) return;
    
    const std::string& json = msg->data;
    ROS_INFO("[px4ctrl] Received parameter update: %s", json.c_str());
    
    // 解析并更新参数
    bool updated = false;
    
    // 位置控制增益 Kp
    double kp0 = parseJsonDouble(json, "Kp0", -1);
    double kp1 = parseJsonDouble(json, "Kp1", -1);
    double kp2 = parseJsonDouble(json, "Kp2", -1);
    if (kp0 > 0) { g_param_ptr->gain.Kp0 = kp0; updated = true; }
    if (kp1 > 0) { g_param_ptr->gain.Kp1 = kp1; updated = true; }
    if (kp2 > 0) { g_param_ptr->gain.Kp2 = kp2; updated = true; }
    
    // 速度控制增益 Kv
    double kv0 = parseJsonDouble(json, "Kv0", -1);
    double kv1 = parseJsonDouble(json, "Kv1", -1);
    double kv2 = parseJsonDouble(json, "Kv2", -1);
    if (kv0 > 0) { g_param_ptr->gain.Kv0 = kv0; updated = true; }
    if (kv1 > 0) { g_param_ptr->gain.Kv1 = kv1; updated = true; }
    if (kv2 > 0) { g_param_ptr->gain.Kv2 = kv2; updated = true; }
    
    // 积分增益 Kvi
    double kvi0 = parseJsonDouble(json, "Kvi0", -1);
    double kvi1 = parseJsonDouble(json, "Kvi1", -1);
    double kvi2 = parseJsonDouble(json, "Kvi2", -1);
    if (kvi0 >= 0) { g_param_ptr->gain.Kvi0 = kvi0; updated = true; }
    if (kvi1 >= 0) { g_param_ptr->gain.Kvi1 = kvi1; updated = true; }
    if (kvi2 >= 0) { g_param_ptr->gain.Kvi2 = kvi2; updated = true; }
    
    // 角速度增益
    double kangR = parseJsonDouble(json, "KAngR", -1);
    double kangP = parseJsonDouble(json, "KAngP", -1);
    double kangY = parseJsonDouble(json, "KAngY", -1);
    if (kangR > 0) { g_param_ptr->gain.KAngR = kangR; updated = true; }
    if (kangP > 0) { g_param_ptr->gain.KAngP = kangP; updated = true; }
    if (kangY > 0) { g_param_ptr->gain.KAngY = kangY; updated = true; }
    
    if (updated) {
        ROS_INFO("[px4ctrl] Gains updated: Kp=[%.2f,%.2f,%.2f], Kv=[%.2f,%.2f,%.2f]",
                 g_param_ptr->gain.Kp0, g_param_ptr->gain.Kp1, g_param_ptr->gain.Kp2,
                 g_param_ptr->gain.Kv0, g_param_ptr->gain.Kv1, g_param_ptr->gain.Kv2);
    }
}

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;
    param.config_from_ros_handle(nh);
    
    // 设置全局参数指针，用于动态参数更新
    g_param_ptr = &param;

    // Controller controller(param);
    Controller controller(param);
    PX4CtrlFSM fsm(param, controller);
    
    // 动态参数更新订阅器 (用于自动调参)
    ros::Subscriber param_update_sub =
        nh.subscribe<std_msgs::String>("update_gains", 10, paramUpdateCallback);

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state",
                                         10,
                                         boost::bind(&State_Data_t::feed, &fsm.state_data, _1));

    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                 10,
                                                 boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub;
    if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }

    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land",
                                                  100,
                                                  boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());

    fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

    fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

    fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    ros::Duration(0.5).sleep();

    if (param.takeoff_land.no_RC)
    {
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        ROS_INFO("PX4CTRL] Waiting for RC");
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    int trials = 0;
    while (ros::ok() && !fsm.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connnect to PX4!!!");
    }

    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        fsm.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}
