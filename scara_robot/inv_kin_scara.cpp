#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <math.h>


bool control_ws(double x, double y, double z)
{
    double a1 = 0.3, a2 = 0.3, rad1 = 2.5, rad2 = 2;

    if (z < -0.45 || z > 0)
        return 0;
    
    //Raggio corto, dato dal Carnot tra a1 e a2
    double min_radius_pow = pow(a1,2) + pow(a2,2) - 2*a1*a2*cos(M_PI - rad2);
    //Raggio lungo dato dalla somma tra a1 e a2
    double max_radius = a1+a2;

    //Formule trigonometriche per la circonferenza
    //Se è maggiore esce dalla circonferenza grande
    if ((pow(x,2) + pow(y,2)) > pow(max_radius,2))
        return 0;
    if ((pow(x,2) + pow(y,2)) < min_radius_pow)
        return 0;
    else if (x > max_radius * cos(rad1))
    {
        return 1;
    }

    double cx = a1*cos(rad1);
    double cy = a2*sin(rad1);
    //Controllo della forma a cardioide
    if (((pow((x-cx),2) + pow((y-cy),2)) > pow(a2,2)) || ((pow((x-cx),2) + pow((y+cy),2)) > pow(a2,2)))
        return 0;

    return 1;
}

sensor_msgs::JointState inv_joints;

int main(int argc, char** argv)
{
ros::init(argc, argv, "inv_kin");

ros::NodeHandle invkin;

ros::Publisher inv_pub = invkin.advertise<sensor_msgs::JointState>("/JointState", 1000);

//sensor_msgs::JointState inv_joints;

ros::Rate loop_rate(10);

while (ros::ok)
    {
        double x, y, z, angle;
        double a1 = 0.3, a2 = 0.3;

        double cos_theta2, sen_theta2,sen_theta2_n, theta2, theta2_n, alpha, cos_beta, sen_beta, sen_beta_n, beta, beta_n, theta1, theta_1_2;

        std::cout << "Inserire la posizione dell'EE nel formato xyz e l'angolo finale rispetto all'asse z" << std::endl;
        std::cin >> x >> y >> z >> angle;

        if (control_ws(x,y,z))
            {
                cos_theta2 = (pow(x,2) + pow(y,2) - pow(a1,2) - pow(a2,2))/(2*a1*a2);
                sen_theta2 = sqrt(1-pow(cos_theta2,2));
                sen_theta2_n = -(sqrt(1-pow(cos_theta2,2)));

                theta2 = atan2(sen_theta2, cos_theta2);
                theta2_n = atan2(sen_theta2_n, cos_theta2);

                alpha = atan2(y,x);
                
                cos_beta = (pow(a1,2) + pow(x,2) + pow(y,2) - pow(a2,2))/(2*a1*(sqrt(pow(x,2)+pow(y,2))));
                sen_beta = sqrt(1-pow(cos_beta,2));
                sen_beta_n = -(sqrt(1-pow(cos_beta,2)));

                beta = atan2(sen_beta, cos_beta);
                beta_n = atan2(sen_beta_n, cos_beta);

                theta1 = alpha + beta_n;            
                theta_1_2 = alpha - beta_n;
                
                
            /*
                inv_joints.position[0] = theta1;
                inv_joints.position[1] = theta2;
                inv_joints.position[2] = -z;
                inv_joints.position[3] = theta1 + theta2 - angle;
            */
            }

       // inv_pub.publish(inv_joints);
       ROS_INFO("Valori dei giunti:\n1° - [%.3lf, %.3lf, %.3lf, %.3lf]\n2° - [%.3lf, %.3lf, %.3lf %.3lf]\n", theta1, theta2, -z, theta1 + theta2 - angle
        , theta_1_2, theta2_n, -z, theta_1_2 + theta2_n - angle);

        ros::spinOnce();

        loop_rate.sleep();

    }
    return 0;
}
