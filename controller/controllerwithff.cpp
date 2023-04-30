#include <ros/ros.h>            
#include <std_msgs/Float64.h>       // Header file for teeterbot topics
#include <std_msgs/Bool.h> 
#include <std_msgs/Int16.h>          // Header file for fallen over topic
#include <geometry_msgs/Vector3.h>  // Header file for IMU topic
#include <geometry_msgs/Twist.h>    // Header file for teleop topic
#include <boost/algorithm/clamp.hpp>
#include <ros/console.h>
int int_var;
double double_var;
std::string string_var;
int Kp = -4, Ki = -5, Kd = -10;     // Gain parameters
float FF=15;
class Controller {
    private:
        // ---- ROS VARIABLES ---- //
        ros::Publisher leftPub;		        // Publish left wheel speed
        ros::Publisher rightPub;		    // Publish right wheel speed
        ros::Subscriber rpySub;		        // Subscribe to teeterbot pitch angle 
        ros::Subscriber speedInput;         // Subscribe to keyboard inputs
        ros::Subscriber fallen;             // Subscribed to fallen over state


        // ---- PID VARIABLES ---- //
        int Kp = -10, Ki = -8, Kd = -1;     // Gain parameters
        float FF=20, angleSpeed=0;
        float currentPitch, desiredPitch=0, error, errorSum = 0, errorDiff, errorPrev = 0;    

        // ---- KEYBOARD VARIABLES ---- //
        float targetSpeed;
        float desiredSpeed;
        float desiredRotation;
        std_msgs::Float64 leftSpeed;
        std_msgs::Float64 rightSpeed;

        // ---- FALLEN OVER VARIABLE ---- //
        bool currentState;
         
        

    public:
        Controller(ros::NodeHandle *n){        
            speedInput = n->subscribe("/cmd_vel",100,&Controller::speedSetter,this);
            rpySub = n->subscribe("/teeterbot/rpy",100,&Controller::pid,this);
            fallen = n->subscribe("/teeterbot/fallen_over",100,&Controller::fallenSetter,this);
            leftPub = n->advertise<std_msgs::Float64>("/teeterbot/left_speed_cmd",100);
            rightPub = n->advertise<std_msgs::Float64>("/teeterbot/right_speed_cmd",100);

        }

        void pid(const geometry_msgs::Vector3::ConstPtr IMUdata){
            // Obtain current angle via IMU
            currentPitch = IMUdata->y; //x is roll, y is pitch, z is yaw

            ros::param::get("/Kp", Kp);
            ros::param::get("/Ki", Ki);
            ros::param::get("/Kd", Kd);
            ros::param::get("/FF", FF);
            // Compute error
            error = desiredPitch - currentPitch;

            // Compute cumulative error
            errorSum = error + errorSum;

            // Compute error difference
            errorDiff = error - errorPrev;
            errorPrev = error;

            //compute Feedforward
            angleSpeed=sinf(currentPitch);
            //ROS_DEBUG(angleSpeed);
            //std::cout << angleSpeed; 

            // Compute target speed
            targetSpeed = Kp*error + Ki*errorSum + Kd*errorDiff+angleSpeed*FF;

            //ROS_INFO_STREAM("Error is " << error << "ErrorSum is " << errorSum << " | ErrorDiff is " << errorDiff << " | desiredSpeed is " << desiredSpeed << " | targetSpeed is " << targetSpeed);

            // Combining the target speed computed from the PID block with the desired speed from the keyboard
            leftSpeed.data = targetSpeed+desiredSpeed;
            rightSpeed.data = targetSpeed+desiredSpeed;


            if(abs(error) > 0 && abs(error) < 1.5){ 
                /*
                    When the error is greater than 0 and less than 1.5 radian (86 degrees), the controller works as intended.
                    The reason the  error is only '> 0'and not '>= 0' is because, at the start of the simulation, the error reads as 0. 
                    This prevents the controller from immediately starting. Once the user commands the desired direction, then the controller kicks in.
                */
                if(desiredRotation == 0.0){
                    leftSpeed.data = targetSpeed+desiredSpeed;
                    rightSpeed.data = targetSpeed+desiredSpeed;
                }
                else if(desiredRotation > 0){
                    leftSpeed.data = (targetSpeed+desiredSpeed)-abs(desiredRotation);
                    rightSpeed.data = (targetSpeed+desiredSpeed)+abs(desiredRotation);
                }
                else if(desiredRotation < 0){
                    leftSpeed.data = (targetSpeed+desiredSpeed)+abs(desiredRotation);
                    rightSpeed.data = (targetSpeed+desiredSpeed)-abs(desiredRotation);
                }
                //ROS_INFO_STREAM("Left Speed is: " << leftSpeed.data << " | right speed is: " << rightSpeed.data);
                leftPub.publish(leftSpeed);
                rightPub.publish(rightSpeed);
            }
            else{
                /*
                    Two situations are passed to this else clause: (1) when error = 0 or (2) when error > 1.5 rad/86 degrees.
                    In both cases, another set of if-else statement occurs below: (1) when the robot has fallen and (2) when the robot hasnt fallen
                        If the robot has fallen over (true) (error > 1.5 rad/86 degs), then the PID values and speed is set to 0; therefore, stopping and resetting the robot
                        If the robot not fallen over (false) (error is = 0 (i.e at the start of the simulation)), then there is a passthrough allowing the desired speeds to be published
                            This is what allows the controller from starting off immediately.                      
                */
                if(currentState == true){
                    leftSpeed.data = 0;
                    rightSpeed.data = 0;
                    errorSum = 0;
                    errorDiff = 0;
                    errorPrev = 0;
                    leftPub.publish(leftSpeed);
                    rightPub.publish(rightSpeed);
                }
                else{
                    leftPub.publish(leftSpeed);
                    rightPub.publish(rightSpeed);
                }
            }

        }

        void speedSetter(const geometry_msgs::Twist& input){
            desiredSpeed = 3.8*input.linear.x;
            desiredRotation = 1.2*input.angular.z;
        }

        void fallenSetter(const std_msgs::Bool& state){
            currentState = state.data;
        }

};

int main(int argc, char** argv){
    ros::init(argc,argv, "controller_with_FF");
    ros::NodeHandle n;
    ros::param::set("/Kp", Kp);
    ros::param::set("/Ki", Ki);
    ros::param::set("/Kd", Kd);
    ros::param::set("/FF", FF);
    Controller con = Controller(&n);
    ros::spin();
}