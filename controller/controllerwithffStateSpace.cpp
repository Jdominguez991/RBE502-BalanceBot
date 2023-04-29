#include <ros/ros.h>            
#include <std_msgs/Float64.h>       // Header file for teeterbot topics
#include <std_msgs/Bool.h> 
#include <std_msgs/Int16.h>          // Header file for fallen over topic
#include <geometry_msgs/Vector3.h>  // Header file for IMU topic
#include <geometry_msgs/Twist.h>    // Header file for teleop topic
#include <boost/algorithm/clamp.hpp>
#include <ros/console.h>
#include <Dynamical_Model.h>
#include "Matrix.h"

int int_var;
double double_var;
std::string string_var;
int KpVar = -4, KiVar = -5, KdVar = -10;     // Gain parameters
float FF=15;
class Controller {
private:
    // ----- ROS VARIABLES ----- //
    ros::Publisher leftPub;		        // Publish left wheel speed/torque
    ros::Publisher rightPub;		    // Publish right wheel speed/torque
    ros::Subscriber rpySub;		        // Subscribe to teeterbot pitch angle 
    ros::Subscriber drpySub;            // Subscribe to teeterbot pitch angle rate
    ros::Subscriber speedInput;         // Subscribe to keyboard inputs
    ros::Subscriber fallen;             // Subscribe to fallen over state
    ros::Subscriber leftSub;            // Subscribe to left wheel speed
    ros::Subscriber rightSub;           // Subscribe to right wheel speed
    ros::Timer tbtimer;     // used to compute q dot

    // ----- PID VARIABLES ----- //
    Matrix<3,3> Kp, Kd;
    Matrix<3,1> q, dq, q_des;
    Matrix<3,1> dq_des, ddq_des;
    Matrix<3,1> q_error, dq_error, first2Values, tau_cmd, desiredTou;  
    Dynamical_Model tbDM;

    // ----- KEYBOARD VARIABLES ----- //
    std_msgs::Float64 leftSpeed;
    std_msgs::Float64 rightSpeed;

    // ----- FALLEN OVER VARIABLE ----- //
    bool grounded;

    // ----- TIME VARIABLES ----- //
    double delta_t = 1/100;
    ros::Time ttick; 
    ros::Duration ttock;
    void resetTimer() { delta_t = 0; ttick = ros::Time::now(); }  // this should be called at the end of every "control iteration"

public:
    // Constructor:
    Controller(ros::NodeHandle *n)
    {
        // Establish communications:
        speedInput = n->subscribe("/cmd_vel",100,&Controller::desiredSpeedSetter,this);
        rpySub = n->subscribe("/teeterbot/rpy",100,&Controller::computedTorque,this);
        drpySub = n->subscribe("/teeterbot/rpy",100,&Controller::currentPitchRateSetter,this);
        fallen = n->subscribe("/teeterbot/fallen_over",100,&Controller::fallenSetter,this);
        leftSub = n->subscribe("/teeterbot/left_wheel_speed",100,&Controller::currentSpeedSetterLW,this);
        rightSub = n->subscribe("/teeterbot/right_wheel_speed",100,&Controller::currentSpeedSetterRW,this);
        leftPub = n->advertise<std_msgs::Float64>("/teeterbot/left_speed_cmd",100);
        rightPub = n->advertise<std_msgs::Float64>("/teeterbot/right_speed_cmd",100);
        // Initialize timer:
        //resetTimer();
        Kp.diag(KpVar);
        Kd.diag(KdVar);
    }

    // Function for Control Logic:
    void computedTorque(const geometry_msgs::Vector3::ConstPtr IMUdata)
    {

        ros::param::get("/Kp", KpVar);
        ros::param::get("/Kd", KdVar);

        Kp.diag(10.0);
        Kd.diag(10.0);
        // Establish current position:
        //timeCheck();
        q(0,0) += dq(0,0) * delta_t;
        q(1,0) += dq(1,0) * delta_t;
        q(2,0) = double(IMUdata->y);    // x = roll, y = pitch, z = yaw

        // Compute positional error:
        q_error = q_des + (-1.0*q);     // Didn't feel like defining "operator-" for Matrix.h

        // (Re-)Set Desired State:
        ddq_des *= 0.0; //= (-1.0*dq_des);     // set this to act against dq_des?
        if ((grounded) || (std::abs(q_error(2,0)) > 0.1))
        {
            dq_des *= -1.0;    // reset to zero even if it contradicts keyboard input
            ddq_des = dq_des;
            q_des *= 0.0;
        }

        // Compute velocity error:
        dq_error = dq_des + (-1.0*dq);  // Didn't feel like defining "operator-" for Matrix.h

        // Calculate the first two varibles in the equation:
        first2Values = (Kd*dq_error) + (Kp*q_error);

        // Apply Dynamics:
        tbDM.compute_M(q_des);
        tbDM.compute_C(q_des, dq_des);
        tbDM.compute_G(q_des);
        tbDM.compute_Tau(dq_des, q_des);
        desiredTou = tbDM.get_Tau();
        tau_cmd=first2Values+desiredTou
        // Publish commands:
        rightSpeed.data = float(tau_cmd.getElem(0,0)); 
        leftSpeed.data = float(tau_cmd.getElem(1,0)); 
        rightPub.publish(rightSpeed);
        leftPub.publish(leftSpeed);

        // Reset Timer:
        //resetTimer();
    }

    // Function to set current speed of right wheel based on subscription:
    void currentSpeedSetterRW(const std_msgs::Float64& input)
    {
        dq(0,0) = double(input.data);
    }

    // Function to set current speed of left wheel based on subscription:
    void currentSpeedSetterLW(const std_msgs::Float64& input)
    {
        dq(1,0) = double(input.data);
    }

    // Function to set current pitch rate based on subscription:
    void currentPitchRateSetter(geometry_msgs::Vector3::ConstPtr IMUdata)
    {
        dq(2,0) = (IMUdata->y)*delta_t; // remember: delta_t will be reset in main Control Logic
    }

    // Function to set speed of teeterbot based on keyboard commands:
    void desiredSpeedSetter(const geometry_msgs::Twist& input){
        dq_des(0,0) = input.linear.x;
        dq_des(1,0) = input.linear.x;
        dq_des(2,0) = input.angular.z;
        //timeCheck();
        q_des(0,0) += dq_des(0,0) * delta_t; // propagate desired velocity to desired position
        q_des(1,0) += dq_des(1,0) * delta_t; // propagate desired velocity to desired position
        q_des(2,0) = 0.0;   // we always want teeterbot pointing straigt up
    }

    // Function to check if teeterbot has fallen over:
    void fallenSetter(const std_msgs::Bool& state){
        grounded = state.data;
    }

    
    void timeCheck() 
    { 
        ttock = ros::Time::now() - ttick;
        delta_t += double(ttock.toSec());
    }

};

int main(int argc, char** argv)
{
    ros::param::set("/Kp", KpVar);
    ros::param::set("/Kd", KdVar);
    ros::init(argc, argv, "controllerCompTorq");
    ROS_INFO("Computed Torque Controller Initialized");
    ros::NodeHandle n;
    Controller con = Controller(&n);
    ros::spin();
}