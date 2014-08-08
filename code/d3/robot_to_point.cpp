#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>

const double PI(3.14159265359);

class RobotDriver{
    private:
    // The node handle we'll be using
    ros::NodeHandle nh_;
    // We will be publishing to the "cmd_vel" topic to issue commands
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    // We will be listening to TF transforms as well
    tf::TransformListener listener_;
    float lx, ly;  // point to the robot (local reference)

    // frequency of the position topic 'rostopic hz topic'
    static const float freq = 90;
    float dt;

    float k1, k2;  // gains
    float xx, xy;  // position of the mobile robot
    float v1, v2;  // velocities of the mobile robot (v1: linear, v2: angular)

    float xxp, xyp;  // position of the mobile robot's point

    float xrx, xry;  // position of the reference
    float vrx, vry;  // velocities of the reference
    // Eigen::VectorXd v(2);

    tf::Pose pr2_pose;

    // vectors and matrices
    Eigen::VectorXd l;
    Eigen::VectorXd x;
    Eigen::VectorXd xtil;  // error
    Eigen::VectorXd xtil_last;  // last error
    Eigen::VectorXd xtils;  // sum of errors
    Eigen::VectorXd dx;
    Eigen::VectorXd dxr;
    Eigen::VectorXd dxp;
    Eigen::VectorXd xp;  // point of the robot
    Eigen::VectorXd xd;  // desired point
    Eigen::VectorXd u;  // linear input for transformation
    Eigen::VectorXd v;  // real input for the robot
    Eigen::MatrixXd k;  // proportional gain
    Eigen::MatrixXd k_dev;  // derivative gain
    Eigen::MatrixXd k_int;  // integral gain
    Eigen::MatrixXd S;


    // geometry_msgs::PoseWithCovarianceStampednav_msgs/Odometry pose;
    geometry_msgs::Twist base_cmd;

    public:
    // ROS node initialization
    RobotDriver():l(2), dx(2), dxr(2), dxp(2), xp(2), u(2), v(2),
                  k(2, 2), S(2, 2), xd(2), x(2), xtil(2), xtils(2),
                  xtil_last(2), k_dev(2, 2), k_int(2, 2) {
        // set up the publisher for the cmd_vel topic
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        // robot odometry
        odom_sub_ = nh_.subscribe("/odom", 2, &RobotDriver::getOdom, this);
        // point of the mobile robot
        ly = 0;
        lx = .2;
        l << lx, ly;
        S<<1, -ly,
            0, lx;
        // gains
        k1 = .5;
        k2 = .5;
        k << k1, 0,
            0, k2;
        k_dev << .5, 0,
            0, .5;
        k_int << 1000, 0,
                 0, 1000;
        // velocities initialization
        // but actually we're waiting for the odometry to replace the values
        v1 = 0;
        v2 = 0;
        v << v1, v2;
        xtil << 1, 1;
        xtils << 0, 0;
        xtil_last << 0, 0;
        // positions of the reference
        xrx = 0;
        xry = 0;
        // velocities of the reference
        vrx = 0;
        vry = 0;
        // dt of position measurements
        dt = 1/freq;
    }

    // drive forward a specified distance based on odometry information
    bool driveToPoint(double xxd, double xyd) {
        xd << xxd, xyd;
        float minval = 0.04;  // 4cm from the target
        ros::Rate rate(10.0);
        bool done = false;
        while (!done && nh_.ok()) {
            ros::spinOnce();
            /*with boundaries*/
            // linear
            base_cmd.linear.x = v(0)>.8?.8:v(0);
            // angular
            base_cmd.angular.z = v(1)>.8?.8:v(1);

            // /*without boundaries*/
            // // linear
            // base_cmd.linear.x=v(0);
            // // angular
            // base_cmd.angular.z=v(1);

            cmd_vel_pub_.publish(base_cmd);
            rate.sleep();
            if (xtil(0)<minval && xtil(0)>-minval
               && xtil(1)<minval && xtil(1)>-minval)
                done = true;
        }
        if (done) return true;
        return false;
    }
    // void getOdom(const geometry_msgs::PoseWithCovarianceStamped msg){
    void getOdom(const nav_msgs::Odometry msg) {
        // position of the mobile robot
        xx = msg.pose.pose.position.x;
        xy = msg.pose.pose.position.y;
        x << xx, xy;
        // getting angle
        tf::poseMsgToTF(msg.pose.pose, pr2_pose);
        float theta = tf::getYaw(pr2_pose.getRotation());
        // getting velocities from odometry
        v1 = msg.twist.twist.linear.x;  // linear velocity
        v2 = msg.twist.twist.angular.z;  // angular velocity
        float tol = 0.1;
        v1 = v1>v(0)+tol?v(0)+tol:v1 < v(0)-tol?v(0)-tol:v1;
        v2 = v2>v(1)+tol?v(1)+tol:v2 < v(1)-tol?v(1)-tol:v2;
        v << v1, v2;
        // getting position of the point p (xp)
        xp = x+Rot(theta)*l;
        // getting the error
        xtil = xp-xd;  // desired position minus position of the point
        // getting the control law to stabilize the point
        // (the commented part is the derivative and integral)
        // integral part in next comment is not working well
        u=-k*xtil-k_dev*(xtil-xtil_last)/dt/*-k_int*xtils*dt*/;
        // transforming the control law to the parameters of the robot
        v=(Rot(theta)*S).inverse()*u;
        // printing parameters
        // std::cout << "theta: "<<theta << std::endl;
        // std::cout << "x: "<<x << std::endl;
        // std::cout << "xp: "<<xp << std::endl;
        // std::cout << "xtil: "<<xtil << std::endl;
        std::cout << "u: "<< u << std::endl;
        std::cout << "v: "<< v << std::endl;
        // error for integral and derivative control
        xtil_last = xtil;  // put the error as last error
        xtils += xtil;  // summing the errors
        // setting limit to integration error
        xtils(0) = xtils(0)>1.5?1.5:xtils(0) < -1.5?-1.5:xtils(0);
        xtils(1) = xtils(1)>1.5?1.5:xtils(1) < -1.5?-1.5:xtils(1);
        // std::cout <<  "xtils: "<< xtils << std::endl;
    }
    Eigen::MatrixXd Rot(double theta) {
        Eigen::MatrixXd Rot(2, 2);
        Rot<< cos(theta), -sin(theta),
            sin(theta), cos(theta);
        return Rot;
    }
};

int main(int argc, char** argv) {
    // init the ROS node
    ros::init(argc, argv, "robot_driver");

    double xx = atof(argv[1]);
    double yy = atof(argv[2]);
    // double xx=1;
    // double yy=1;

    RobotDriver driver;
    driver.driveToPoint(xx, yy);
    // Eigen::VectorXd x(2);
    // x<<xx,1;
    // Eigen::MatrixXd A(2,2);
    // A<<3,2,
    //  2,1;
    // std::cout << A*x  << std::endl;
}
