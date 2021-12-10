#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <optimal_sensing_sim/output_ukf.h>
#include <optimal_sensing_sim/output_formation.h>
#include <optimal_sensing_sim/output_uavs_controller.h>
#include <optimal_sensing_sim/output_uavs_pose.h>
#include <std_msgs/Float64.h>
#include <string>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>
#include <deque>
#include <numeric>
#include <random>
#include <vector>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include <eigen_conversions/eigen_msg.h>
#define pi 3.14159265359

using namespace std;
////////////////////Global variable//////////////////
Eigen::VectorXd r_target,rs_1, rs_2, rs_3;
float fx = 381.36246688113556, fy = 381.36246688113556; //286.02185016085167
float cx = 320.5, cy = 240.5; 							//240.5 135.5
float image_width = 640, image_height = 480;			//480*270
float model_width = 4.6344, model_height = 1.5;
float target_offset = 1.2;
int callback_spin_count = 0;
float KPx ,KPy, KPz, KP_yaw;
float K_Formation_x ,K_Formation_y;
float weight_target;
float gama_1;
float desired_heading_1, desired_heading_2, desired_heading_3;
int loop_rate = 30;
bool formation_mode = false;
bool ukf_flag = false;
/////////////////////////////////////////////////////
bool measurement_flag = true;
int uav_1_idx = 0;
int uav_2_idx = 0;
int uav_3_idx = 0;
int target_idx = 0;
mavros_msgs::State current_1_state,current_2_state,current_3_state;
geometry_msgs::PoseStamped uav_1_mocap,uav_2_mocap,uav_3_mocap,target_pose;
geometry_msgs::Twist uav_1_mocap_vel,uav_2_mocap_vel,uav_3_mocap_vel;
optimal_sensing_sim::output_ukf estimated_target_state;

typedef struct
{
    double roll;
    double pitch;
    double yaw;
}rpy;

typedef struct
{
    float px;
    float py;
    float pz;
    float yaw;
}vir;

typedef struct
{
    float px;
    float py;
    float pz;
    float vx;
    float vy;
    float vz;
    float yaw;
}dynamic_vir;

typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
}dis;

int sign(double num)
{
    if(num >= 0) return 1;
    else if(num < 0) return -1;
    else return 0;
}

void mocap_cb(const gazebo_msgs::ModelStates::ConstPtr &msg){
    //gaz_state = *msg;
    while ((strcmp(msg->name[uav_1_idx].c_str(),"iris1") != 0 ))
    {
        if(uav_1_idx<(msg->name.size()-1))
            uav_1_idx++;
    }
    uav_1_mocap.pose = msg->pose[uav_1_idx];
    uav_1_mocap_vel = msg->twist[uav_1_idx];
    while ((strcmp(msg->name[uav_2_idx].c_str(),"iris2") != 0 ))
    {
        if(uav_2_idx<(msg->name.size()-1))
            uav_2_idx++;
    }
    uav_2_mocap.pose = msg->pose[uav_2_idx];
    uav_2_mocap_vel = msg->twist[uav_2_idx];
    while ((strcmp(msg->name[uav_3_idx].c_str(),"iris3") != 0 ))
    {
        if(uav_3_idx<(msg->name.size()-1))
            uav_3_idx++;
    }
    uav_3_mocap.pose = msg->pose[uav_3_idx];
    uav_3_mocap_vel = msg->twist[uav_3_idx];
    while ((strcmp(msg->name[target_idx].c_str(),"prius") != 0 ))
    {
        if(target_idx<(msg->name.size()-1))
            target_idx++;
    } 
    target_pose.pose = msg->pose[target_idx];
}

void estimated_data_cb(const optimal_sensing_sim::output_ukf::ConstPtr& msg) {
    estimated_target_state = *msg;
}

void state_1_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_1_state = *msg;
}

void state_2_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_2_state = *msg;
}

void state_3_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_3_state = *msg;
}



rpy quaternionToRPY(float quat_x, float quat_y, float quat_z, float quat_w)
{
    rpy rpy1;
    double roll, pitch, yaw;
    tf::Quaternion quat1(quat_x,quat_y,quat_z,quat_w);
    tf::Matrix3x3(quat1).getRPY(roll, pitch, yaw);
    rpy1.roll = roll;
    rpy1.pitch = pitch;
    rpy1.yaw = yaw;

    return rpy1;
}
rpy rpy_target,rpy_uav_1, rpy_uav_2, rpy_uav_3;

void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, rpy host_rpy,geometry_msgs::TwistStamped* vs, dis& dis_host)
{
    float ex, ey, ez, e_yaw;
    float ux, uy, uz, u_yaw;
    float local_x, local_y;

    local_x = cos(vir.yaw)*dis_host.x+sin(vir.yaw)*dis_host.y;
    local_y = -sin(vir.yaw)*dis_host.x+cos(vir.yaw)*dis_host.y;

    ex = vir.px - host_mocap.pose.position.x - local_x;
    ey = vir.py - host_mocap.pose.position.y - local_y;
    ez = vir.pz - host_mocap.pose.position.z - 0;
    e_yaw = vir.yaw - host_rpy.yaw;
    if(e_yaw>pi)
        e_yaw = e_yaw - 2*pi;
    else if(e_yaw<-pi)
        e_yaw = e_yaw + 2*pi;
    //ROS_INFO("e_yaw: %.3f",e_yaw);

    ux = KPx*ex;
    uy = KPy*ey;
    uz = KPz*ez;
    u_yaw = KP_yaw*e_yaw;

    vs->twist.linear.x = ux ;
    vs->twist.linear.y = uy ;
    vs->twist.linear.z = uz ;
    vs->twist.angular.z = u_yaw ;
}

void formation_control(dynamic_vir& dynamic_vir, geometry_msgs::PoseStamped& host_mocap, rpy host_rpy,geometry_msgs::TwistStamped* vs, dis& dis_host, geometry_msgs::PoseStamped& nbr_1_mocap, dis& dis_nbr_1, geometry_msgs::PoseStamped& nbr_2_mocap, dis& dis_nbr_2)
{
        float ex, ey, ez, e_yaw;
        float ux, uy, uz, u_yaw;
	float local_x, local_y;
	float local_x1, local_y1;
	float local_x2, local_y2;
	float dis_x1, dis_y1;
	float dis_x2, dis_y2;

	local_x = cos(dynamic_vir.yaw)*dis_host.x+sin(dynamic_vir.yaw)*dis_host.y;
	local_y = -sin(dynamic_vir.yaw)*dis_host.x+cos(dynamic_vir.yaw)*dis_host.y;

	dis_x1 = dis_host.x - dis_nbr_1.x;
	dis_y1 = dis_host.y - dis_nbr_1.y;

	local_x1 = cos(dynamic_vir.yaw)*dis_x1+sin(dynamic_vir.yaw)*dis_y1;
	local_y1 = -sin(dynamic_vir.yaw)*dis_x1+cos(dynamic_vir.yaw)*dis_y1;

	dis_x2 = dis_host.x - dis_nbr_2.x;
	dis_y2 = dis_host.y - dis_nbr_2.y;

	local_x2 = cos(dynamic_vir.yaw)*dis_x2+sin(dynamic_vir.yaw)*dis_y2;
	local_y2 = -sin(dynamic_vir.yaw)*dis_x2+cos(dynamic_vir.yaw)*dis_y2;

        ex = weight_target*(dynamic_vir.px - host_mocap.pose.position.x + local_x) + (nbr_1_mocap.pose.position.x - host_mocap.pose.position.x + local_x1) + (nbr_2_mocap.pose.position.x - host_mocap.pose.position.x + local_x2);
        ey = weight_target*(dynamic_vir.py - host_mocap.pose.position.y + local_y) + (nbr_1_mocap.pose.position.y - host_mocap.pose.position.y + local_y1) + (nbr_2_mocap.pose.position.y - host_mocap.pose.position.y + local_y2);
        ez = dis_host.z - host_mocap.pose.position.z - 0;
        e_yaw = dis_host.yaw - dynamic_vir.yaw - host_rpy.yaw;
        if(e_yaw>pi) e_yaw = e_yaw - 2*pi;
        else if(e_yaw<-pi) e_yaw = e_yaw + 2*pi;

        ux = K_Formation_x*ex + dynamic_vir.vx;
        uy = K_Formation_y*ey + dynamic_vir.vy;
        uz = KPz*ez + dynamic_vir.vz;
        u_yaw = KP_yaw*e_yaw;

	vs->twist.linear.x = ux;
	vs->twist.linear.y = uy;
	vs->twist.linear.z = uz;
        vs->twist.angular.z = u_yaw;

}

char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

int main(int argc, char **argv)
{
    string topic_box_uav1;
    ros::init(argc, argv, "formation_tracking");
    ros::NodeHandle nh;
    ros::Subscriber host_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",1,mocap_cb);
    ros::Subscriber estimated_sub = nh.subscribe<optimal_sensing_sim::output_ukf>("/estimated_data",1,estimated_data_cb);
    ros::Subscriber state_1_sub = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10, state_1_cb);
    ros::Subscriber state_2_sub = nh.subscribe<mavros_msgs::State>("/uav2/mavros/state", 10, state_2_cb);
    ros::Subscriber state_3_sub = nh.subscribe<mavros_msgs::State>("/uav3/mavros/state", 10, state_3_cb);
    ros::ServiceClient arming_1_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    ros::ServiceClient arming_2_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav2/mavros/cmd/arming");
    ros::ServiceClient arming_3_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav3/mavros/cmd/arming");
    ros::ServiceClient set_mode_1_client = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
    ros::ServiceClient set_mode_2_client = nh.serviceClient<mavros_msgs::SetMode>("/uav2/mavros/set_mode");
    ros::ServiceClient set_mode_3_client = nh.serviceClient<mavros_msgs::SetMode>("/uav3/mavros/set_mode");
    ros::Publisher local_vel_1_pub = nh.advertise<geometry_msgs::TwistStamped>("/uav1/mavros/setpoint_velocity/cmd_vel", 1);
    ros::Publisher local_vel_2_pub = nh.advertise<geometry_msgs::TwistStamped>("/uav2/mavros/setpoint_velocity/cmd_vel", 1);
    ros::Publisher local_vel_3_pub = nh.advertise<geometry_msgs::TwistStamped>("/uav3/mavros/setpoint_velocity/cmd_vel", 1);
    ros::Publisher formation_pub = nh.advertise<optimal_sensing_sim::output_formation>("/formation", 1);
    ros::Publisher formation_controller_pub = nh.advertise<optimal_sensing_sim::output_uavs_controller>("/formation_controller", 1);
    ros::Publisher uavs_pose_pub = nh.advertise<optimal_sensing_sim::output_uavs_pose>("/uavs_pose", 1);
    ros::Rate rate(loop_rate);

    while (ros::ok() && !current_1_state.connected && !current_2_state.connected && !current_3_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    r_target.setZero(3);
    rs_1.setZero(3);
    rs_2.setZero(3);
    rs_3.setZero(3);

    geometry_msgs::TwistStamped vs_1,vs_2,vs_3;
    vir vir_1,vir_2,vir_3;
    dynamic_vir vir_target;
    dis dis_hover,dis_1,dis_2,dis_3;

    ros::param::get("~KPx", KPx);
    ros::param::get("~KPy", KPy);
    ros::param::get("~KPz", KPz);
    ros::param::get("~KP_yaw", KP_yaw);
    ros::param::get("~K_Formation_x", K_Formation_x);
    ros::param::get("~K_Formation_y", K_Formation_y);
    ros::param::get("~weight_target", weight_target);
    ros::param::get("~gama_1", gama_1);
    ros::param::get("~desired_heading_1", desired_heading_1);
    ros::param::get("~desired_heading_2", desired_heading_2);
    ros::param::get("~desired_heading_3", desired_heading_3);
    ros::param::get("~dis_1_x", dis_1.x);
    ros::param::get("~dis_1_y", dis_1.y);
    ros::param::get("~dis_1_z", dis_1.z);
    ros::param::get("~dis_2_x", dis_2.x);
    ros::param::get("~dis_2_y", dis_2.y);
    ros::param::get("~dis_2_z", dis_2.z);
    ros::param::get("~dis_3_x", dis_3.x);
    ros::param::get("~dis_3_y", dis_3.y);
    ros::param::get("~dis_3_z", dis_3.z);

    vs_1.twist.linear.x = 0;
    vs_1.twist.linear.y = 0;
    vs_1.twist.linear.z = 0;
    vs_1.twist.angular.x = 0;
    vs_1.twist.angular.y = 0;
    vs_1.twist.angular.z = 0;

    vs_2.twist.linear.x = 0;
    vs_2.twist.linear.y = 0;
    vs_2.twist.linear.z = 0;
    vs_2.twist.angular.x = 0;
    vs_2.twist.angular.y = 0;
    vs_2.twist.angular.z = 0;

    vs_3.twist.linear.x = 0;
    vs_3.twist.linear.y = 0;
    vs_3.twist.linear.z = 0;
    vs_3.twist.angular.x = 0;
    vs_3.twist.angular.y = 0;
    vs_3.twist.angular.z = 0;

    dis_1.yaw = desired_heading_1;
    dis_2.yaw = desired_heading_2;
    dis_3.yaw = desired_heading_3;

    //send a few setpoints before starting
    for(int i = 50; ros::ok() && i > 0; --i){
        local_vel_1_pub.publish(vs_1);
        local_vel_2_pub.publish(vs_2);
        local_vel_3_pub.publish(vs_3);
        
        vir_1.px = uav_1_mocap.pose.position.x;
        vir_1.py = uav_1_mocap.pose.position.y;
        vir_1.pz = dis_1.z;
        vir_1.yaw = desired_heading_1;

        vir_2.px = uav_2_mocap.pose.position.x;
        vir_2.py = uav_2_mocap.pose.position.y;
        vir_2.pz = dis_2.z;
        vir_2.yaw = desired_heading_2;
        
        vir_3.px = uav_3_mocap.pose.position.x;
        vir_3.py = uav_3_mocap.pose.position.y;
        vir_3.pz = dis_3.z;
        vir_3.yaw = desired_heading_3;
        callback_spin_count++;
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request_1 = ros::Time::now();
    ros::Time last_request_2 = ros::Time::now();
    ros::Time last_request_3 = ros::Time::now();

    while(ros::ok()){
        nh.getParam("/ukf_flag",ukf_flag);
        optimal_sensing_sim::output_formation formation;
        optimal_sensing_sim::output_uavs_controller formation_controller;
        optimal_sensing_sim::output_uavs_pose uavs_pose;
        if (current_1_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request_1 > ros::Duration(5.0))) {
            if( set_mode_1_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request_1 = ros::Time::now();
        } else {
            if (!current_1_state.armed &&
                    (ros::Time::now() - last_request_1 > ros::Duration(5.0))) {
                if( arming_1_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request_1 = ros::Time::now();
            }
        }
        if (current_2_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request_2 > ros::Duration(5.0))) {
            if( set_mode_2_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request_2 = ros::Time::now();
        } else {
            if (!current_2_state.armed &&
                    (ros::Time::now() - last_request_2 > ros::Duration(5.0))) {
                if( arming_2_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request_2 = ros::Time::now();
            }
        }
        if (current_3_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request_3 > ros::Duration(5.0))) {
            if( set_mode_3_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request_3 = ros::Time::now();
        } else {
            if (!current_3_state.armed &&
                    (ros::Time::now() - last_request_3 > ros::Duration(5.0))) {
                if( arming_3_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request_3 = ros::Time::now();
            }
        }
        rpy_uav_1 = quaternionToRPY(uav_1_mocap.pose.orientation.x,uav_1_mocap.pose.orientation.y,uav_1_mocap.pose.orientation.z,uav_1_mocap.pose.orientation.w);
        rpy_uav_2 = quaternionToRPY(uav_2_mocap.pose.orientation.x,uav_2_mocap.pose.orientation.y,uav_2_mocap.pose.orientation.z,uav_2_mocap.pose.orientation.w);
        rpy_uav_3 = quaternionToRPY(uav_3_mocap.pose.orientation.x,uav_3_mocap.pose.orientation.y,uav_3_mocap.pose.orientation.z,uav_3_mocap.pose.orientation.w);
        rpy_target = quaternionToRPY(target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w);

        if(rpy_uav_1.yaw>pi) rpy_uav_1.yaw = rpy_uav_1.yaw - 2*pi;
        else if(rpy_uav_1.yaw<-pi) rpy_uav_1.yaw = rpy_uav_1.yaw + 2*pi;
        if(rpy_uav_2.yaw>pi) rpy_uav_2.yaw = rpy_uav_2.yaw - 2*pi;
        else if(rpy_uav_2.yaw<-pi) rpy_uav_2.yaw = rpy_uav_2.yaw + 2*pi;
        if(rpy_uav_3.yaw>pi) rpy_uav_3.yaw = rpy_uav_3.yaw - 2*pi;
        else if(rpy_uav_3.yaw<-pi) rpy_uav_3.yaw = rpy_uav_3.yaw + 2*pi;

        rs_1 << uav_1_mocap.pose.position.x, uav_1_mocap.pose.position.y, uav_1_mocap.pose.position.z;
        rs_2 << uav_2_mocap.pose.position.x, uav_2_mocap.pose.position.y, uav_2_mocap.pose.position.z;
        rs_3 << uav_3_mocap.pose.position.x, uav_3_mocap.pose.position.y, uav_3_mocap.pose.position.z;
        r_target << target_pose.pose.position.x+sin(rpy_target.yaw-pi/2)*target_offset, target_pose.pose.position.y+cos(rpy_target.yaw-pi/2)*target_offset, 0.5*model_height;

        uavs_pose.rx_c.data = rs_1[0];
        uavs_pose.ry_c.data = rs_1[1];
        uavs_pose.rz_c.data = rs_1[2];
        uavs_pose.theta_c.data = rpy_uav_1.yaw;
        uavs_pose.rx_r.data = rs_2[0];
        uavs_pose.ry_r.data = rs_2[1];
        uavs_pose.rz_r.data = rs_2[2];
        uavs_pose.theta_r.data = rpy_uav_2.yaw;
        uavs_pose.rx_b.data = rs_3[0];
        uavs_pose.ry_b.data = rs_3[1];
        uavs_pose.rz_b.data = rs_3[2];
        uavs_pose.theta_b.data = rpy_uav_3.yaw;
        uavs_pose_pub.publish(uavs_pose);

        float rho_xy_1,rho_xy_2,rho_xy_3,beta_12,beta_13,beta_23;
        float rho_xy_12,rho_xy_13,rho_xy_23;
        rho_xy_1 = sqrt(pow(rs_1(0)-r_target(0),2)+pow(rs_1(1)-r_target(1),2));
        rho_xy_2 = sqrt(pow(rs_2(0)-r_target(0),2)+pow(rs_2(1)-r_target(1),2));
        rho_xy_3 = sqrt(pow(rs_3(0)-r_target(0),2)+pow(rs_3(1)-r_target(1),2));
        beta_12 = abs(atan2(rs_1(1)-r_target(1),rs_1(0)-r_target(0))-atan2(rs_2(1)-r_target(1),rs_2(0)-r_target(0)));
        if (beta_12 > pi) beta_12 = 2*pi - beta_12;
        beta_13 = abs(atan2(rs_1(1)-r_target(1),rs_1(0)-r_target(0))-atan2(rs_3(1)-r_target(1),rs_3(0)-r_target(0)));
        if (beta_13 > pi) beta_13 = 2*pi - beta_13;
        beta_23 = abs(atan2(rs_2(1)-r_target(1),rs_2(0)-r_target(0))-atan2(rs_3(1)-r_target(1),rs_3(0)-r_target(0)));
        if (beta_23 > pi) beta_23 = 2*pi - beta_23;
        formation.rho_xy_1.data = rho_xy_1;
        formation.rho_xy_2.data = rho_xy_2;
        formation.rho_xy_3.data = rho_xy_3;
        formation.beta_12.data = beta_12;
        formation.beta_13.data = beta_13;
        formation.beta_23.data = beta_23;
        formation_pub.publish(formation);

        int c = getch();
        //ROS_INFO("C: %d",c);
        if (c != EOF) {
            switch (c) {
            case 115:    // key origin
            {
                vir_1.pz = 0;
                vir_1.yaw = desired_heading_1;
                vir_2.pz = 0;
                vir_2.yaw = desired_heading_2;
                vir_3.pz = 0;
                vir_3.yaw = desired_heading_3;
                break;
            }
            case 49:    // virtual leader mode
            {
                formation_mode = false;
                break;
            }
            case 50:    // formation_mode
            {
                formation_mode = true;
                break;
            }
            case 108:    // close arming
            {
                offb_set_mode.request.custom_mode = "MANUAL";
                set_mode_1_client.call(offb_set_mode);
                set_mode_2_client.call(offb_set_mode);
                set_mode_3_client.call(offb_set_mode);
                arm_cmd.request.value = false;
                arming_1_client.call(arm_cmd);
                arming_2_client.call(arm_cmd);
                arming_3_client.call(arm_cmd);
                break;
            }
            case 63:
                return 0;
                break;
            }
        }


        if(formation_mode == false)
        {
            dis_hover.x = 0;
            dis_hover.y = 0;
            dis_hover.z = 0;
            dis_hover.yaw = 0;
            follow(vir_1,uav_1_mocap,rpy_uav_1,&vs_1,dis_hover);
            local_vel_1_pub.publish(vs_1);
            follow(vir_2,uav_2_mocap,rpy_uav_2,&vs_2,dis_hover);
            local_vel_2_pub.publish(vs_2);
            follow(vir_3,uav_3_mocap,rpy_uav_3,&vs_3,dis_hover);
            local_vel_3_pub.publish(vs_3);
        }
        else
        {   
            if(ukf_flag){
                vir_target.px = estimated_target_state.target_pose.x;
                vir_target.py = estimated_target_state.target_pose.y;
                vir_target.pz = 0.5*model_height;
                vir_target.vx = estimated_target_state.target_vel.x;
                vir_target.vy = estimated_target_state.target_vel.y;
                vir_target.vz = 0;
                vir_target.yaw = 0;
                formation_control(vir_target,uav_1_mocap,rpy_uav_1,&vs_1,dis_1,uav_2_mocap,dis_2,uav_3_mocap,dis_3);
                local_vel_1_pub.publish(vs_1);
                formation_control(vir_target,uav_2_mocap,rpy_uav_2,&vs_2,dis_2,uav_1_mocap,dis_1,uav_3_mocap,dis_3);
                local_vel_2_pub.publish(vs_2);
                formation_control(vir_target,uav_3_mocap,rpy_uav_3,&vs_3,dis_3,uav_1_mocap,dis_1,uav_2_mocap,dis_2);
                local_vel_3_pub.publish(vs_3);
            }
            else{
                vir_target.px = 0.0;
                vir_target.py = 0.0;
                vir_target.pz = 0.5*model_height;
                vir_target.vx = 0.0;
                vir_target.vy = 0.0;
                vir_target.vz = 0.0;
                vir_target.yaw = 0;
                formation_control(vir_target,uav_1_mocap,rpy_uav_1,&vs_1,dis_1,uav_2_mocap,dis_2,uav_3_mocap,dis_3);
                local_vel_1_pub.publish(vs_1);
                formation_control(vir_target,uav_2_mocap,rpy_uav_2,&vs_2,dis_2,uav_1_mocap,dis_1,uav_3_mocap,dis_3);
                local_vel_2_pub.publish(vs_2);
                formation_control(vir_target,uav_3_mocap,rpy_uav_3,&vs_3,dis_3,uav_1_mocap,dis_1,uav_2_mocap,dis_2);
                local_vel_3_pub.publish(vs_3);
            }
            formation_controller.vx_1.data = vs_1.twist.linear.x;
            formation_controller.vy_1.data = vs_1.twist.linear.y;
            formation_controller.vz_1.data = vs_1.twist.linear.z;
            formation_controller.wz_1.data = vs_1.twist.angular.z;
            formation_controller.vx_2.data = vs_2.twist.linear.x;
            formation_controller.vy_2.data = vs_2.twist.linear.y;
            formation_controller.vz_2.data = vs_2.twist.linear.z;
            formation_controller.wz_2.data = vs_2.twist.angular.z;
            formation_controller.vx_3.data = vs_3.twist.linear.x;
            formation_controller.vy_3.data = vs_3.twist.linear.y;
            formation_controller.vz_3.data = vs_3.twist.linear.z;
            formation_controller.wz_3.data = vs_3.twist.angular.z;
            formation_controller_pub.publish(formation_controller);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
