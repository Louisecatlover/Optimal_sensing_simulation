#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <optimal_sensing_sim/output_ukf.h>
#include <optimal_sensing_sim/output_measurement.h>
#include <std_msgs/Float64.h>
#include <string>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
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
Eigen::VectorXd rs_1, rs_2, rs_3, r_target;
Eigen::VectorXd target_gvel;
Eigen::VectorXd r_FLW,r_FRW,r_RLW,r_RRW;
Eigen::MatrixXd rotation_uav_1_x, rotation_uav_1_y, rotation_uav_1_z;
Eigen::MatrixXd rotationB2C_uav_x, rotationB2C_uav_y, rotationB2C_uav_z;
Eigen::MatrixXd rot_g2c; //c : camera  g : global  
float fx = 381.36246688113556, fy = 381.36246688113556; 
float cx = 320.5, cy = 240.5; 							
float image_width = 640, image_height = 480;			
float model_width = 4.6344, model_height = 1.5;
float target_offset = 1.2;
int callback_spin_count = 0;
float asp_ratio = 1.65;
int loop_rate = 30;
bool optimal_sensing_mode = false;
bool measurement_flag = true;
int uav_1_idx = 0;
int uav_2_idx = 0;
int uav_3_idx = 0;
int target_idx = 0;
mavros_msgs::State current_1_state,current_2_state,current_3_state;
geometry_msgs::PoseStamped uav_1_mocap,uav_2_mocap,uav_3_mocap,target_pose;
geometry_msgs::Twist uav_1_mocap_vel,uav_2_mocap_vel,uav_3_mocap_vel, target_vel;
geometry_msgs::PoseStamped FL_wheel_mocap,FR_wheel_mocap,RL_wheel_mocap,RR_wheel_mocap;
std_msgs::Float32MultiArray box;
bool ukf_flag = false;
bool initial_finish_flag = false;
/////////////////////////////////////////////////////
typedef struct
{
    double roll;
    double pitch;
    double yaw;
}rpy;

typedef struct
{
    float yaw;
    float x;
    float y;
    float z;
}vir;

int sign(double num)
{
    if(num >= 0) return 1;
    else if(num < 0) return -1;
    else return 0;
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
rpy rpy_uav_1, rpy_target;


gazebo_msgs::LinkStates link_states;
void link_cb(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
	link_states = *msg;
	if(link_states.name.size()>0) {
		for(unsigned int i=0; i<link_states.name.size(); i++) {
			if(link_states.name[i].compare("prius::front_left_wheel")==0) {
				FL_wheel_mocap.pose = link_states.pose[i];
			}
			if(link_states.name[i].compare("prius::front_right_wheel")==0) {
				FR_wheel_mocap.pose = link_states.pose[i];
			}
            if(link_states.name[i].compare("prius::rear_left_wheel")==0) {
				RL_wheel_mocap.pose = link_states.pose[i];
			}
            if(link_states.name[i].compare("prius::rear_right_wheel")==0) {
				RR_wheel_mocap.pose = link_states.pose[i];
			}
		}
	}
}

void mocap_cb(const gazebo_msgs::ModelStates::ConstPtr &msg){
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
    target_vel = msg->twist[target_idx];
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

////////////////////UKF Global variable//////////////////
double L;
double dt;
//vector size
int x_size;
int y_size;
int x_sigmavector_size;
int y_sigmavector_size;
double innovation_sum;
double innovation_time_avg;
int innovation_times;
int innovation_size = 1000;
int convergence_wait_count = 0;
bool consistency_flag = false;
vector<double> innovation_vector;
//int window_size;

void predict();
void correct(Eigen::VectorXd measure);

Eigen::VectorXd x ; //states
Eigen::VectorXd x_last;
Eigen::VectorXd y ; //measurements

Eigen::VectorXd x_hat; //x mean
Eigen::VectorXd y_hat; //y mean

double alpha ;
double kappa ;
double beta ;
double lambda ;

Eigen::VectorXd w_c ; //weight c
Eigen::VectorXd w_m ;  //weight m
Eigen::MatrixXd Wc;//for covariance calculation

Eigen::MatrixXd x_sigmapoint;
Eigen::MatrixXd x_sigmavector ;
Eigen::MatrixXd y_sigmavector ;
Eigen::MatrixXd H ;    //measurement transform

Eigen::MatrixXd P ; //covariance matrix
std_msgs::Float64MultiArray P_final;
float det_P;
std_msgs::Float64 det_P_msg;

Eigen::MatrixXd P_init;
Eigen::MatrixXd measurement_noise;
Eigen::MatrixXd process_noise;
Eigen::MatrixXd Q ; //process noise matrix
Eigen::MatrixXd R ; //measurement noise matrix
Eigen::VectorXd q ;
Eigen::VectorXd r ;
deque<Eigen::VectorXd> q_window, r_window;
deque<Eigen::MatrixXd> Q_window, R_window;
deque<double> w_window;

Eigen::MatrixXd P_a ; //covariance matrix
Eigen::MatrixXd P_ ; //covariance matrix
Eigen::MatrixXd P_yy ;
Eigen::MatrixXd P_yy_ ;
Eigen::MatrixXd P_xy ;
Eigen::MatrixXd Kalman_gain ;
////////////////////////////////////////////////////////////////


////////////////////define state and measurement////////////////
enum state{
    member_xq=0,
    member_yq,
    member_vqx,
    member_vqy,
    statesize
};

enum measurement{
    member_mu_1 = 0,
    member_mv_1,
    member_mrho_2,
    member_mbeta_3,
    measurementsize
};

////////////////////////////////////////////////////////////////


////////////////////dynamic(predict state)////////////////
Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state){

    x_size = statesize;
    x_sigmavector_size = 2*x_size+1;
    Eigen::MatrixXd predict_sigma_state(x_size,x_sigmavector_size);

    for(int i=0;i<x_sigmavector_size;i++){        
        //initialize the sigma vector
        Eigen::Vector2d q_pose,vq;
        q_pose.setZero();
        vq.setZero();
        q_pose << sigma_state(member_xq,i), sigma_state(member_yq,i);
        vq << sigma_state(member_vqx,i), sigma_state(member_vqy,i);
        //Process model
        Eigen::Vector2d q_pose_,vq_;
        q_pose_.setZero();
        vq_.setZero();
        q_pose_(0) = q_pose(0) + vq(0)*dt;
        q_pose_(1) = q_pose(1) + vq(1)*dt;
        vq_ = vq;         
       //update the predict_sigma_state
        predict_sigma_state(member_xq,i) =  q_pose_(0);
        predict_sigma_state(member_yq,i) =  q_pose_(1);
        predict_sigma_state(member_vqx,i) =  vq_(0);
        predict_sigma_state(member_vqy,i) =  vq_(1);
    }
    return predict_sigma_state;
}
///////////////////////////////////////////////////////


////////////////////predict measurement////////////////
Eigen::MatrixXd state_to_measure(Eigen::MatrixXd sigma_state){
    y_size = measurementsize;
    x_sigmavector_size = 2*x_size+1;
    Eigen::MatrixXd predict_sigma_measure(y_size,x_sigmavector_size);
    for(int i=0;i<x_sigmavector_size;i++){
        predict_sigma_measure( member_mu_1 ,i) = (double)-fx*(cos(rpy_uav_1.yaw)*(rs_1(1)-sigma_state(member_yq,i))-sin(rpy_uav_1.yaw)*(rs_1(0)-sigma_state(member_xq,i)))/(cos(rpy_uav_1.yaw)*(rs_1(0)-sigma_state(member_xq,i))+sin(rpy_uav_1.yaw)*(rs_1(1)-sigma_state(member_yq,i))) + (double)cx;
        predict_sigma_measure( member_mv_1 ,i) = (double)-fy*(rs_1(2)-r_target(2))/(cos(rpy_uav_1.yaw)*(rs_1(0)-sigma_state(member_xq,i))+sin(rpy_uav_1.yaw)*(rs_1(1)-sigma_state(member_yq,i))) + (double)cy;
        predict_sigma_measure( member_mrho_2 ,i) = sqrt(pow(rs_2(0)-sigma_state(member_xq,i),2) + pow(rs_2(1)-sigma_state(member_yq,i),2));
        predict_sigma_measure( member_mbeta_3 ,i) =  atan2(rs_3(1)-sigma_state(member_yq,i),rs_3(0)-sigma_state(member_xq,i));
    }
    return predict_sigma_measure;
}
////////////////////////////////////////////////////



void initialize(){
    ROS_INFO("Initilaize");
    x_size = statesize;
    y_size = measurementsize;
    alpha = 0.001;
    kappa = 0.0;
    beta = 2.0;
    lambda = 0.0;
    innovation_time_avg = 0.0;
    innovation_sum = 0.0;
    innovation_times = 0;

    L=(double)x_size;
    x_sigmavector_size=2*x_size+1;

    lambda= alpha * alpha * (L + kappa) -L;

    x.setZero(x_size);
    x_last.setZero(x_size);
    y.setZero(y_size);

    x_hat.setZero(x_size);
    y_hat.setZero(y_size);

    x_sigmapoint.setZero(x_size,x_sigmavector_size);
    x_sigmavector.setZero(x_size,x_sigmavector_size);
    y_sigmavector.setZero(y_size,x_sigmavector_size);

    H.setZero(y_size,x_size);  // measurement matrix
    y = H*x;

    w_c.setZero(x_sigmavector_size);
    w_m.setZero(x_sigmavector_size);

    w_c(0) = (lambda / (L+lambda))+(1.0-alpha*alpha+beta);
    w_m(0) = (lambda)/(L+lambda);
    for(int i=1 ; i<x_sigmavector_size ; i++){
        w_c(i) = 1/(2*(L+lambda));
        w_m(i) = 1/(2*(L+lambda));
    }

    Wc.setZero(x_sigmavector_size,x_sigmavector_size);
    Wc = w_c.array().matrix().asDiagonal();

    Q.setZero();
    R.setZero();
    q.setZero(x_size);
    r.setZero(y_size);
    P.setZero(x_size,x_size);
    P_.setZero(x_size,x_size);
    P_yy.setZero(y_size,y_size);
    P_xy.setZero(x_size,y_size);
    P_yy_.setZero(y_size,y_size);
    Kalman_gain.setZero(x_size,y_size);
}


void predict(){
    //find sigma point
    P=(lambda+L)*P;
    Eigen::MatrixXd M;
    M.setZero(x_size,x_size);
    M = (P).llt().matrixL();
    x_sigmapoint.col(0) = x;
    for(int i=0;i<x_size;i++)
    {
        Eigen::VectorXd sigma =(M.row(i)).transpose();
        x_sigmapoint.col(i+1) = x + sigma;
        x_sigmapoint.col(i+x_size+1) = x - sigma;
    }
    x_sigmavector = dynamics( x_sigmapoint);
    //x_hat (mean) 
    x_hat.setZero(x_size);//initialize x_hat
    for(int i=0;i<x_sigmavector_size;i++){
        x_hat += w_m(i)* x_sigmavector.col(i);
    }
    x_hat += q;
    //covariance
    P_.setZero(x_size,x_size);
    for(int i=0 ; i<x_sigmavector_size ;i++){
        P_+=   w_c(i) * (x_sigmavector.col(i)-x_hat) * ((x_sigmavector.col(i)-x_hat).transpose());
    }
    //add process noise covariance
    P = P_ + Q;

    y_sigmavector = state_to_measure(x_sigmavector);
    //y_hat (mean)
    y_hat.setZero(y_size);
    for(int i=0;i< x_sigmavector_size;i++){
        y_hat += w_m(i) * y_sigmavector.col(i);
    }
}


void correct(Eigen::VectorXd measure){
    y = measure;

    P_yy_.setZero(y_size,y_size);
    P_yy.setZero(y_size,y_size);
    P_xy.setZero(x_size,y_size);

    for(int i=0;i<x_sigmavector_size;i++){
        Eigen::VectorXd err_y;
        err_y.setZero(y_size);
        err_y = y_sigmavector.col(i) - y_hat;
        P_yy_ += w_c(i) * err_y * err_y.transpose();
    }
    //add measurement noise covarinace
    P_yy = P_yy_ + R;

    for(int i=0;i<x_sigmavector_size;i++){
        Eigen::VectorXd err_y , err_x;
        err_y.setZero(y_size);
        err_x.setZero(x_size);
        err_y = y_sigmavector.col(i) - y_hat;
        err_x = x_sigmavector.col(i) - x_hat;
        P_xy += w_c(i) * err_x * err_y.transpose();
    }
    Kalman_gain = P_xy * (P_yy.inverse());

    if(!measurement_flag) y = y_hat;

    x = x_hat + Kalman_gain *(y-y_hat);
    x_last = x;
    P = P - Kalman_gain * P_yy * (Kalman_gain.transpose());
    det_P = (float)P.block<2,2>(0,0).determinant();
    cout << "det_P:" << det_P << endl;
}


//noise estimate
void noise_estimate(int window_size)
{
    //ROS_INFO("estimate noise");
    Eigen::VectorXd q_window_element;
    Eigen::MatrixXd Q_window_element;
    Eigen::MatrixXd Q_window_element_;
    double delta_S, w_element;
    deque<double> v_weight;
    float S_gain = 10;
    q_window_element.setZero(x_size);
    Q_window_element.setZero(x_size,x_size);
    Q_window_element_.setZero(x_size,x_size);
    v_weight.resize(window_size);
    double accu = 0;

    Eigen::VectorXd q_window_sum;
    Eigen::MatrixXd Q_window_sum;

    Q_window_element_ = P + Kalman_gain*(y - y_hat)*((y - y_hat).transpose())*(Kalman_gain.transpose()) - P_;
    bool Q_PD_flag = true;
    for(int i = 0; i < x_size; i++){
        if(Q_window_element_(i,i) < 0) Q_PD_flag = false;
    }
    if(Q_PD_flag){
        q_window_element = x - x_hat+q;
        Q_window_element = Q_window_element_.diagonal().asDiagonal();
    }
    else{
        Eigen::VectorXd zero_q;
        zero_q.setZero(x_size);
        q_window_element = zero_q;
        Q_window_element = process_noise;
    }
    if (Q_window_element(2,2) < process_noise(2,2)) Q_window_element(2,2) = process_noise(2,2);
    if (Q_window_element(3,3) < process_noise(3,3)) Q_window_element(3,3) = process_noise(3,3);
    delta_S = (S_gain*(P_yy).trace())/(((y - y_hat).transpose())*(y - y_hat));
    w_element = sqrt(((x - x_hat).transpose())*(x - x_hat))*sqrt(((y_hat - y).transpose())*(y_hat - y))*delta_S;

    q_window.push_front(q_window_element);
    Q_window.push_front(Q_window_element);
    w_window.push_front(w_element);

    if(q_window.size()>window_size || Q_window.size()>window_size)
    {
        q_window.resize(window_size);
        Q_window.resize(window_size);
        w_window.resize(window_size);
    }


    if(q_window.size()==window_size || Q_window.size()==window_size)
    {
        //ROS_INFO("estimate noise");
        q_window_sum.setZero(x_size);
        Q_window_sum.setZero(x_size,x_size);

        for(int i=0;i<window_size;i++)
        {
            accu += w_window.at(i);
        }
        for(int i=0;i<window_size;i++)
        {
            v_weight.at(i) = w_window.at(i)/accu;
        }

        for(int i=0;i<window_size;i++)
        {
            q_window_sum += q_window.at(i)*v_weight.at(i);
            Q_window_sum += Q_window.at(i)*v_weight.at(i);
        }
        if(callback_spin_count>4*loop_rate)
        {
            q = q_window_sum;
            Q = Q_window_sum;
        }
    }
    callback_spin_count++;
}




int main(int argc, char **argv)
{
    string topic_box_uav1;
    ros::init(argc, argv, "target_estimate");
    ros::NodeHandle nh;
    ros::Subscriber host_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",1,mocap_cb);
    ros::Subscriber link_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",1,link_cb);
    ros::Subscriber state_1_sub = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10, state_1_cb);
    ros::Subscriber state_2_sub = nh.subscribe<mavros_msgs::State>("/uav2/mavros/state", 10, state_2_cb);
    ros::Subscriber state_3_sub = nh.subscribe<mavros_msgs::State>("/uav3/mavros/state", 10, state_3_cb);
    ros::Publisher true_pub = nh.advertise<optimal_sensing_sim::output_ukf>("/true_data", 1);
    ros::Publisher estimate_pub = nh.advertise<optimal_sensing_sim::output_ukf>("/estimated_data", 1);
    ros::Publisher error_pub = nh.advertise<optimal_sensing_sim::output_ukf>("/error_data", 1);
    ros::Publisher measurement_pub = nh.advertise<optimal_sensing_sim::output_measurement>("/measurement_data", 1);
    ros::Publisher det_P_pub = nh.advertise<std_msgs::Float64>("/det_P", 1);
    ros::Publisher covariance_pub = nh.advertise<std_msgs::Float64MultiArray>("/covariance_matrix", 1);
    ros::Rate rate(loop_rate);

    while (ros::ok() && !current_1_state.connected && !current_2_state.connected && !current_3_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    initialize();
    //set initial value of state
    ros::param::get("~x_init_0", x(0));
    ros::param::get("~x_init_1", x(1));
    ros::param::get("~x_init_2", x(2));
    ros::param::get("~x_init_3", x(3));
    rs_1.setZero(3);
    rs_2.setZero(3);
    rs_3.setZero(3);
    r_target.setZero(3);
    target_gvel.setZero(3);
    r_FLW.setZero(2);
    r_FRW.setZero(2);
    r_RLW.setZero(2);
    r_RRW.setZero(2);

    ros::Time current_time = ros::Time::now();
    ros::Time previous_time = ros::Time::now();
    int measurement_false_count = 0;
    int measurement_true_count = 0;

    //increase the initial value of P can increase the speed of convergence
    P_init.setZero(statesize,statesize);
    ros::param::get("~P_init_0", P_init(0,0));
    ros::param::get("~P_init_1", P_init(1,1));
    ros::param::get("~P_init_2", P_init(2,2));
    ros::param::get("~P_init_3", P_init(3,3));
    P = P_init;             //set initial P matrix

    measurement_noise.setZero(measurementsize,measurementsize);
    measurement_noise = 1* Eigen::MatrixXd::Identity(measurementsize,measurementsize);
    ros::param::get("~measurement_noise_0", measurement_noise(0,0));
    ros::param::get("~measurement_noise_1", measurement_noise(1,1));
    ros::param::get("~measurement_noise_2", measurement_noise(2,2));
    ros::param::get("~measurement_noise_3", measurement_noise(3,3));
    R = measurement_noise;             //set measurement noise


    process_noise.setZero(statesize,statesize);
    process_noise = 1* Eigen::MatrixXd::Identity(statesize,statesize);
    ros::param::get("~process_noise_0", process_noise(0,0));
    ros::param::get("~process_noise_1", process_noise(1,1));
    ros::param::get("~process_noise_2", process_noise(2,2));
    ros::param::get("~process_noise_3", process_noise(3,3));
    Q = process_noise;              //set process noise

    while(ros::ok()){
        nh.getParam("/ukf_flag",ukf_flag);
        optimal_sensing_sim::output_ukf true_value, estimate_value, error_value;
        optimal_sensing_sim::output_measurement measurement_value;

        rpy_uav_1 = quaternionToRPY(uav_1_mocap.pose.orientation.x,uav_1_mocap.pose.orientation.y,uav_1_mocap.pose.orientation.z,uav_1_mocap.pose.orientation.w);
        rpy_target = quaternionToRPY(target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w);

        rs_1 << uav_1_mocap.pose.position.x, uav_1_mocap.pose.position.y, uav_1_mocap.pose.position.z;
        rs_2 << uav_2_mocap.pose.position.x, uav_2_mocap.pose.position.y, uav_2_mocap.pose.position.z;
        rs_3 << uav_3_mocap.pose.position.x, uav_3_mocap.pose.position.y, uav_3_mocap.pose.position.z;
        r_target << target_pose.pose.position.x+sin(rpy_target.yaw-pi/2)*target_offset, target_pose.pose.position.y+cos(rpy_target.yaw-pi/2)*target_offset, 0.5*model_height;
        target_gvel << (double)target_vel.linear.x, (double)target_vel.linear.y, (double)target_vel.linear.z;
        r_FLW << FL_wheel_mocap.pose.position.x, FL_wheel_mocap.pose.position.y;
        r_FRW << FR_wheel_mocap.pose.position.x, FR_wheel_mocap.pose.position.y;
        r_RLW << RL_wheel_mocap.pose.position.x, RL_wheel_mocap.pose.position.y;
        r_RRW << RR_wheel_mocap.pose.position.x, RR_wheel_mocap.pose.position.y;
        
        rotation_uav_1_x.setZero(3,3);
        rotation_uav_1_y.setZero(3,3);
        rotation_uav_1_z.setZero(3,3);

        rotationB2C_uav_x.setZero(3,3);
        rotationB2C_uav_y.setZero(3,3);
        rotationB2C_uav_z.setZero(3,3);

        rotation_uav_1_x <<  1,                0,                 0,
                       0, cos(rpy_uav_1.roll), sin(rpy_uav_1.roll),
                       0, -sin(rpy_uav_1.roll), cos(rpy_uav_1.roll);

        rotation_uav_1_y << cos(rpy_uav_1.pitch), 0, -sin(rpy_uav_1.pitch),
                               0,         1,       0,
                    sin(rpy_uav_1.pitch), 0, cos(rpy_uav_1.pitch);

        if(rpy_uav_1.yaw>2*pi)
            rpy_uav_1.yaw = rpy_uav_1.yaw - 2*pi;
        else if(rpy_uav_1.yaw<0)
            rpy_uav_1.yaw = rpy_uav_1.yaw + 2*pi;

        rotation_uav_1_z << cos(rpy_uav_1.yaw), sin(rpy_uav_1.yaw),    0,
                     -sin(rpy_uav_1.yaw), cos(rpy_uav_1.yaw),    0,
                            0,                0,              1;

        rotationB2C_uav_x <<  1,      0,       0,
                        0, cos(0),  sin(0),
                        0, -sin(0), cos(0);

        rotationB2C_uav_y <<  cos(pi/2), 0, -sin(pi/2),
                            0,       1,        0,
                          sin(pi/2), 0, cos(pi/2);

        rotationB2C_uav_z <<  cos(-pi/2), sin(-pi/2),  0,
                         -sin(-pi/2), cos(-pi/2),  0,
                                0,       0,      1;

        rot_g2c.setZero(3,3);
        rot_g2c = (rotationB2C_uav_z*rotationB2C_uav_y*rotation_uav_1_x*rotation_uav_1_y*rotation_uav_1_z);


        current_time = ros::Time::now();
        dt = current_time.toSec() - previous_time.toSec();
        previous_time = current_time;

        std::random_device rd_x1;
        std::default_random_engine disturbance_generator_x1= std::default_random_engine(rd_x1());
        std::normal_distribution<float> disturbance_distribution_x1(0.0,sqrt(Q(0,0)));
        float rd_noise_x1 = disturbance_distribution_x1(disturbance_generator_x1);
        float true_state_x1 = (float)(r_target(0));// + rd_noise_x1;
        std::random_device rd_x2;
        std::default_random_engine disturbance_generator_x2= std::default_random_engine(rd_x2());
        std::normal_distribution<float> disturbance_distribution_x2(0.0,sqrt(Q(1,1)));
        float rd_noise_x2 = disturbance_distribution_x2(disturbance_generator_x2);
        float true_state_x2 = (float)(r_target(1));// + rd_noise_x2;
        std::random_device rd_x3;
        std::default_random_engine disturbance_generator_x3= std::default_random_engine(rd_x3());
        std::normal_distribution<float> disturbance_distribution_x3(0.0,sqrt(Q(2,2)));
        float rd_noise_x3 = disturbance_distribution_x3(disturbance_generator_x3);
        float true_state_x3 = target_gvel(0);// + rd_noise_x3;
        std::random_device rd_x4;
        std::default_random_engine disturbance_generator_x4= std::default_random_engine(rd_x4());
        std::normal_distribution<float> disturbance_distribution_x4(0.0,sqrt(Q(3,3)));
        float rd_noise_x4 = disturbance_distribution_x4(disturbance_generator_x4);
        float true_state_x4 = target_gvel(1);//+ rd_noise_x4;

        true_value.cmode.data = optimal_sensing_mode;
        true_value.target_pose.x = true_state_x1;
        true_value.target_pose.y = true_state_x2;
        true_value.target_vel.x = true_state_x3;
        true_value.target_vel.y = true_state_x4;
        true_pub.publish(true_value);

        Eigen::Vector3d rqs_1_g,rqs_1_c;
        rqs_1_g.setZero();
        rqs_1_c.setZero();
        rqs_1_g = r_target - rs_1;
        rqs_1_c = rot_g2c*rqs_1_g;
        float depth_truth =  sqrt(pow(rqs_1_g(0),2) + pow(rqs_1_g(1),2));
        float psi_truth = (atan2(rqs_1_g(1),rqs_1_g(0))) - rpy_target.yaw;
        if(psi_truth>pi)
            psi_truth = psi_truth - 2*pi;
        else if(psi_truth<-pi)
            psi_truth = psi_truth + 2*pi;
        if(ukf_flag){
            optimal_sensing_mode = true;
            float center_u_1, center_v_1, rho_2, beta_3;
            float center_u_1_model, center_v_1_model;
            if(initial_finish_flag){
                predict();
                //save measurement data to matrix for correct()
                Eigen::VectorXd measure_vector;
                measure_vector.setZero(measurementsize);
                std::random_device rd_y1;
                std::default_random_engine disturbance_generator_y1= std::default_random_engine(rd_y1());
                std::normal_distribution<float> disturbance_distribution_y1(0.0,sqrt(R(0,0)));
                float rd_noise_y1 = disturbance_distribution_y1(disturbance_generator_y1);
                std::random_device rd_y2;
                std::default_random_engine disturbance_generator_y2= std::default_random_engine(rd_y2());
                std::normal_distribution<float> disturbance_distribution_y2(0.0,sqrt(R(1,1)));
                float rd_noise_y2 = disturbance_distribution_y2(disturbance_generator_y2);
                std::random_device rd_y3;
                std::default_random_engine disturbance_generator_y3= std::default_random_engine(rd_y3());
                std::normal_distribution<float> disturbance_distribution_y3(0.0,sqrt(R(2,2)));
                float rd_noise_y3 = disturbance_distribution_y3(disturbance_generator_y3);
                std::random_device rd_y4;
                std::default_random_engine disturbance_generator_y4= std::default_random_engine(rd_y4());
                std::normal_distribution<float> disturbance_distribution_y4(0.0,sqrt(R(3,3)));
                float rd_noise_y4 = disturbance_distribution_y4(disturbance_generator_y4);
                rho_2 = sqrt(pow(rs_2(0)-r_target(0),2) + pow(rs_2(1)-r_target(1),2))+rd_noise_y3;
                beta_3 = atan2(rs_3(1)-r_target(1),rs_3(0)-r_target(0))+rd_noise_y4;
                center_u_1_model = (double)-fx*(cos(rpy_uav_1.yaw)*(rs_1(1)-r_target(1))-sin(rpy_uav_1.yaw)*(rs_1(0)-r_target(0)))/(cos(rpy_uav_1.yaw)*(rs_1(0)-r_target(0))+sin(rpy_uav_1.yaw)*(rs_1(1)-r_target(1))) + (double)cx +rd_noise_y1;
                center_v_1_model = (double)-fy*(rs_1(2)-r_target(2))/(cos(rpy_uav_1.yaw)*(rs_1(0)-r_target(0))+sin(rpy_uav_1.yaw)*(rs_1(1)-r_target(1))) + (double)cy +rd_noise_y2;
                cout << "center_u_1_model:" << center_u_1_model << endl;
                cout << "center_v_1_model:" << center_v_1_model << endl;
                measure_vector<<(double)center_u_1_model, (double)center_v_1_model, (double)rho_2, (double)beta_3;
                correct(measure_vector);
            }
            estimate_value.cmode.data = optimal_sensing_mode;
            estimate_value.target_pose.x = x(0);
            estimate_value.target_pose.y = x(1);
            estimate_value.target_vel.x = x(2);
            estimate_value.target_vel.y = x(3);

            error_value.cmode.data = optimal_sensing_mode;
            error_value.target_pose.x = true_value.target_pose.x - estimate_value.target_pose.x;
            error_value.target_pose.y = true_value.target_pose.y - estimate_value.target_pose.y;
            error_value.target_vel.x = true_value.target_vel.x - estimate_value.target_vel.x;
            error_value.target_vel.y = true_value.target_vel.y - estimate_value.target_vel.y;

            measurement_value.center_u_1.data = center_u_1_model;
            measurement_value.center_v_1.data = center_v_1_model;
            measurement_value.rho_2.data = rho_2;
            measurement_value.beta_3.data = beta_3;

            estimate_pub.publish(estimate_value);
            error_pub.publish(error_value);
            measurement_pub.publish(measurement_value);
            det_P_msg.data = det_P;
            det_P_pub.publish(det_P_msg);
            tf::matrixEigenToMsg(P,P_final);
            covariance_pub.publish(P_final);
            initial_finish_flag = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
