#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Quaternion.h>
#include<tf/tf.h>
#include<geometry_msgs/Twist.h>
#include<math.h>
#include<geometry_msgs/Pose2D.h>

//Variáveis globais
ros::Publisher twist_pub;
geometry_msgs::Twist robotspeed;
geometry_msgs::Pose2D wi;
geometry_msgs::Pose2D wi1;

double normalizeAngle(double angle){
    if(angle > M_PI){
        return(angle - 2*M_PI);
    }else if(angle < -M_PI){
        return (angle + 2*M_PI);
    }else{
        return angle;
    }
}

double inicialize_dots()
{
    wi.x = 4;
    wi.y = 1.5;
    
    wi1.x = -3;
    wi1.y = 1.5;
}

double delta = 2;


//Função de calculo de distancia
double calculateDistance(double x, double y, double xf, double yf)
{
    double dist = sqrt(pow((xf-x),2)+pow((yf-y),2));
    return dist;
}

//Retorno subscribe
void odomCallBack(const nav_msgs::OdometryConstPtr &msg)
{
    //Determinar posição inicial
    geometry_msgs::Pose2D p;
    p.x = msg->pose.pose.position.x;
    
    p.y = msg->pose.pose.position.y;
    
    p.theta = tf::getYaw(msg->pose.pose.orientation);

    //determinar angulo entre as distancias e controlar velocidade por yaw
    double teta1 = atan2(p.y - wi.y, p.x - wi.x);
   
    double teta = atan2 (wi1.y - wi.y, wi1.x - wi.x);
    double beta = teta - teta1;
     //ROS_INFO("%f", beta); ok
    double Ru = sqrt(pow((wi.y -p.y),2) + pow((wi.x - p.x),2));
    double R = sqrt(pow(Ru,2) - pow((Ru*sin(beta)),2));
    double xc = ((R+delta)*cos(teta) + wi.x);
    double yc = ((R+delta)*sin(teta) + wi.y);
    //ROS_INFO("%f %f", yc, xc); nan
    double w;
    //determinar angulo entre as distancias e controlar velocidade por yaw
    //double d = calculateDistance (p.x,p.y,xc,yc);
    double angle = atan2(yc - p.y,xc - p.x);
    double deltaAngle = angle - p.theta;
    deltaAngle = normalizeAngle(deltaAngle);
    //ROS_INFO("%f %f ", angle);
    //Normalizar angulo
    
    
    w = 0.75*deltaAngle;
    if (std::abs(w) > 0.75)
    {
        w = 0.75*deltaAngle/std::abs(deltaAngle);
    }
    robotspeed.angular.z = w;
    ROS_INFO("%f ", p.theta);
    //if (d > 0.1){
    robotspeed.linear.x = 0.5;
    //}
    //else
   // {
       // v = 0;
       // robotspeed.linear.x = v;
    //}
    //retornar twist de velocidade do robo pro kine_controller
    twist_pub.publish(robotspeed);
     
}


int main (int argc, char **argv)
{
    //Iniciar ROS
    ros::init(argc, argv, "quat2yaw");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("/vrep/vehicle/odometry", 1, odomCallBack);
    twist_pub = nh.advertise<geometry_msgs::Twist>("/robotSpeeds", 1);
    
    inicialize_dots();
    ros::spin();
    
    
}