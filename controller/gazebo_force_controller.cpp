#include <ros/ros.h>            
#include <std_msgs/Float64.h>       // Header file for teeterbot topics
#include <geometry_msgs/Vector3.h>  // Header file for IMU topics

class Controller {
    private:
        // ---- ROS VARIABLES ---- //
        ros::Publisher leftPub;		        // Publish left wheel speed
        ros::Publisher rightPub;		    // Publish right wheel speed
        ros::Subscriber rpySub;		        // Subscribe to teeterbot pitch angle 
        std_msgs::Float64 targetSpeed;      // Variable to store speed that is to be published 

        // ---- PID VARIABLES ---- //
        int Kp = -20, Ki = -5, Kd = -1;     // Gain parameters
        float currentPitch, desiredPitch=0, error, errorSum = 0, errorDiff, errorPrev = 0;    
         

    public:
        Controller(ros::NodeHandle *n){        
            rpySub = n->subscribe("/teeterbot/rpy",100,&Controller::pid,this);
            leftPub = n->advertise<std_msgs::Float64>("/teeterbot/left_speed_cmd",100);
            rightPub = n->advertise<std_msgs::Float64>("/teeterbot/right_speed_cmd",100);
        }

        void pid(const geometry_msgs::Vector3::ConstPtr IMUdata){
            // Obtain current angle via IMU
            currentPitch = IMUdata->y; //x is roll, y is pitch, z is yaw
            
            // Compute error
            error = desiredPitch - currentPitch;

            // Compute cumulative error
            errorSum = error + errorSum;

            // Compute error difference
            errorDiff = error - errorPrev;
            errorPrev = error;

            // Compute target speed
            targetSpeed.data = Kp*error + Ki*errorSum + Kd*errorDiff;

            //ROS_INFO_STREAM("error is " << error << " errorsum is " << errorSum);
            //ROS_INFO_STREAM("speed is " << targetSpeed.data);

            if(abs(error) > 0 && abs(error) < 1.50788){ 
                //ROS_INFO_STREAM("Correcting");
                leftPub.publish(targetSpeed);
                rightPub.publish(targetSpeed);
            }
            else{
                //ROS_INFO_STREAM("Set to 0");
                targetSpeed.data = 0;
                leftPub.publish(targetSpeed);
                rightPub.publish(targetSpeed);
            }
        }
};

int main(int argc, char** argv){
    ros::init(argc,argv, "controller");
    ros::NodeHandle n;
    Controller con = Controller(&n);
    ros::spin();
}