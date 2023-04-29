#include <ros/ros.h>            
#include <std_msgs/Float64.h>       // Header file for teeterbot topics
#include <std_msgs/Bool.h> 
#include <std_msgs/Int16.h>          // Header file for fallen over topic
#include <geometry_msgs/Vector3.h>  // Header file for IMU topic
#include <geometry_msgs/Twist.h>    // Header file for teleop topic
#include <boost/algorithm/clamp.hpp>
#include <ros/console.h>
#include "../include/dynamical_model/Matrix.h"                 // For defining matrices and vectors
#include "../include/dynamical_model/Dynamical_Model.h"        // For teeterbot dynamical model
#include <cmath>
#include <fenv.h>

class Controller {
    private:
        // ----- ROS VARIABLES ----- //
        ros::Publisher leftPub;		        // Publish left wheel speed/torque
        ros::Publisher rightPub;		    // Publish right wheel speed/torque
        ros::Subscriber rpySub;		        // Subscribe to teeterbot pitch angle 
        ros::Subscriber speedInput;         // Subscribe to keyboard inputs
        ros::Subscriber fallen;             // Subscribe to fallen over state
        ros::Subscriber leftSub;            // Subscribe to left wheel speed
        ros::Subscriber rightSub;           // Subscribe to right wheel speed
        ros::Timer tbtimer;     // used to compute q dot

        // ----- PID PARAMETERS ----- //
        Matrix<3,3> Kp_mat, Ki_mat, Kd_mat;
        Matrix<3,1> q, dq;
        Matrix<3,1> q_des, dq_des, ddq_des;
        Matrix<3,1> iq_error, q_error, dq_error; 
        Matrix<3,1> K_term, tau_cmd, Ki_term;  
        Dynamical_Model tbDM;
        double lin_vel_clip = 0.05;     // desired wheel speed input clip
        double ang_vel_clip = 0.025;    // desired tilt rate input clip
        double cmd_lim = 5000;          // torque output clip
        double cf = 1.2;                 // tilt correction factor
        double stable_tilt = 0.025;        // magnitude of stable tilt boundary (radians)
        double corr_tilt;                // tilt to correct

        // ----- CONTROL COMMANDS ----- //
        std_msgs::Float64 leftControlCmd;
        std_msgs::Float64 rightControlCmd;

        // ----- FALLEN OVER VARIABLE ----- //
        bool grounded;

        // ----- TIME VARIABLES ----- //
        double delta_t;
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
            fallen = n->subscribe("/teeterbot/fallen_over",100,&Controller::fallenSetter,this);
            leftSub = n->subscribe("/teeterbot/left_wheel_speed",100,&Controller::currentSpeedSetterLW,this);
            rightSub = n->subscribe("/teeterbot/right_wheel_speed",100,&Controller::currentSpeedSetterRW,this);
            leftPub = n->advertise<std_msgs::Float64>("/teeterbot/left_torque_cmd",100);
            rightPub = n->advertise<std_msgs::Float64>("/teeterbot/right_torque_cmd",100);
            // Initialize variables:
            resetTimer();
            Kp_mat.diag(35.5); //Kp_mat.diag(5.0);
            Ki_mat.diag(0.25);
            Kd_mat.diag(55.0); //Kd_mat.diag(75.0);
            q_des *= 0.0;   dq_des *= 0.0;  ddq_des *= 0.0;
            iq_error *= 0.0; q_error *= 0.0; dq_error *= 0.0;
        }

        // Function for Control Logic:
        void computedTorque(const geometry_msgs::Vector3::ConstPtr IMUdata)
        {
            // Establish current state:
            timeCheck();
            while (delta_t < (1/(pow(10,4))))   // ensure reasonable time-step
            {
                ros::Duration((1/(pow(10,5)))).sleep();
                timeCheck();
            }
            q(0,0) += dq(0,0) * delta_t;
            q(1,0) += dq(1,0) * delta_t;
            q(2,0) = double(IMUdata->y);    // x = roll, y = pitch, z = yaw
            dq(2,0) = (double(IMUdata->y) - q.getElem(2,0))/delta_t;
            //ROS_INFO_STREAM("\nq_des = \n" << q_des.toString() << std::endl);

            // Compute positional error:
            q_error = q_des + (-1.0*q);     // Didn't feel like defining "operator-" for Matrix.h
            //ROS_INFO_STREAM("\nq_error = \n" << q_error.toString() << std::endl);

            // Compute error "integral" (accumulated error):
            //iq_error += q_error;
            //ROS_INFO_STREAM("\niq_error = \n" << iq_error.toString() << std::endl);

            // (Re-)Set Desired State:
            ddq_des *= 0.0; dq_des *= 0.0; q_des *= 0.0; Ki_term *= 0.0;
            
            if ( (std::abs(q(2,0) + (dq(2,0)*delta_t)) > stable_tilt) )     
            {
                // Scale "correction" to oppose anticipated tilt:
                //ROS_INFO("\nAttempting motion correction...\n");
                corr_tilt = (std::abs(q(2,0) + (dq(2,0)*delta_t))) - stable_tilt;  
                dq_des(0,0) = copySign(-dq(0,0))*(corr_tilt*cf)*dq(0,0);    // ensure opposite sign of current joint vel
                dq_des(1,0) = copySign(-dq(1,0))*(corr_tilt*cf)*dq(1,0);    // ensure opposite sign of current joint vel
                dq_des(2,0) = copySign(-dq(2,0))*(corr_tilt*cf)*dq(2,0);
                q_des(0,0) += (dq_des(0,0)) * delta_t; // "walk back" desired position 
                q_des(1,0) += (dq_des(1,0)) * delta_t; // "walk back" desired position
                ddq_des = (1.0/delta_t) * dq_des;

                // Only compute accumulated error when out of range:
                iq_error += q_error;
                //ROS_INFO_STREAM("\niq_error = \n" << iq_error.toString() << std::endl);
                Ki_term = Ki_mat*iq_error;
                //ROS_INFO_STREAM("\nKi_term = \n" << Ki_term.toString() << std::endl);
            }

            // Compute velocity error:
            dq_error = dq_des + (-1.0*dq);  // Didn't feel like defining "operator-" for Matrix.h

            // Calculate "K_term" which will be multiplied by M matrix:
            K_term = ddq_des + (Kd_mat*dq_error) + (Kp_mat*q_error);// + (Ki*iq_error);
            //K_term = Ki_term + K_term;

            // Apply Dynamics:
            tbDM.compute_M(q);
            tbDM.compute_C(q, dq);
            tbDM.compute_G(q);
            tbDM.compute_Tau(dq, K_term);
            tau_cmd = tbDM.get_Tau();

            // Reset parameters if the teeterbot's fallen over:
            if ((grounded) || ((std::abs(q(2,0))) > 1.5))
            {
                q *= 0.0;       dq *= 0.0;
                q_des *= 0.0;   dq_des *= 0.0;  ddq_des *= 0.0;
                tau_cmd *= 0.0;
                iq_error *= 0.0;    // clear accumulated error for "clean slate"
                //ROS_INFO("\nStates reset due to fallen state.\n");
            }

            // Publish commands:
            rightControlCmd.data = (float)clipInput(tau_cmd.getElem(0,0), cmd_lim); 
            leftControlCmd.data = (float)clipInput(tau_cmd.getElem(1,0), cmd_lim); 
            rightPub.publish(rightControlCmd);
            leftPub.publish(leftControlCmd);

            // Reset Timer:
            resetTimer();
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

        // Function to set speed of teeterbot based on keyboard commands:
        void desiredSpeedSetter(const geometry_msgs::Twist& input)
        {
            dq_des(0,0) = clipInput(input.linear.x, lin_vel_clip);
            dq_des(1,0) = clipInput(input.linear.x, lin_vel_clip);
            dq_des(2,0) = clipInput(input.angular.z, ang_vel_clip);
            q_des(0,0) += dq_des(0,0) * delta_t; // propagate desired velocity to desired position
            q_des(1,0) += dq_des(1,0) * delta_t; // propagate desired velocity to desired position
            q_des(2,0) = 0.0;   // we always want teeterbot pointing straigt up
        }

        // Function to check if teeterbot has fallen over:
        void fallenSetter(const std_msgs::Bool& state)
        {
            grounded = state.data;
            //ROS_INFO("\nRobot has fallen!!\n");
        }
        
        void timeCheck() 
        { 
            ttock = ros::Time::now() - ttick;
            delta_t = delta_t + (double) ttock.toSec();
            //ROS_INFO_STREAM("Delta Time: " << std::to_string(delta_t) << std::endl);
        }

        double clipInput(const double inp, double lim)
        {
            double out = inp;
            if (std::abs(inp) > lim)
                out = (copySign(inp))*lim;    // copies the sign of inp to lim
            return out;
        }

        double copySign(const double inp)
        {
            return 1.0*((inp > 0) - (inp < 0));
        }

};

int main(int argc, char** argv)
{
    //feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
    ros::init(argc, argv, "controllerCompTorq");
    ROS_INFO("Computed Torque Controller Initialized");
    ros::NodeHandle n;
    Controller con = Controller(&n);
    ros::spin();
}