/* 
 * Date: 2021-11-29
 * Description: move robots to one line
*/

#include <algorithm>
#include <asm-generic/errno.h>
#include <swarm_robot_control.h>
#include <vector>

namespace config {
    /* Convergence threshold */
    double conv_th = 0.05;  // Threshold of angle, in rad

    /* Velocity scale and threshold */
    double MAX_W = 1;       // Maximum angle velocity (rad/s)
    double MIN_W = 0.05;    // Minimum angle velocity(rad/s)
    double MAX_V = 0.2;     // Maximum linear velocity(m/s)
    double MIN_V = 0.01;    // Minimum linear velocity(m/s)
    double k_w = 0.5;       // Scale of angle velocity
    double k_v = 0.5;        // Scale of linear velocity

    /**
     * @brief several params for avoiding collision
     * Avoiding strategy: imitate gravity but using rejection not attraction
     */
    double MIN_dist = 1.5;  // minimun distance between robots to avoid moving far awary
    double MIN_dist_coef = 0.1;
    double force_coef = 0.08;  // coefficient to replace G etc. in gravity formula
    double w_coef = 0.03;   // scale the impact of rejection on w
    
}

// Util functions
void copy_to_matrix(std::vector<std::vector<double> >& vec, Eigen::MatrixXd& mat);
bool all_converge(Eigen::VectorXd& vec);


/* Main function */
int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_robot_control_formation");
    ros::NodeHandle nh;
    
    /* First: Set ids of swarm robot based on Aruco marker */
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};
    int num_robots = swarm_robot_id.size();

    /* Initialize swarm robot */
    SwarmRobot swarm_robot(&nh, swarm_robot_id);

    /* Set L Matrix and the converge line */
    Eigen::VectorXd line(2); // line function
    line << 1, 1;

    Eigen::MatrixXd lap(num_robots, num_robots);
    lap <<  4, -1, -1, -1, -1, 
            -1, 4, -1, -1, -1,
            -1, -1, 4, -1, -1,
            -1, -1, -1, 4, -1,
            -1, -1, -1, -1, 4;
    // Eigen::MatrixXd lap1(num_robots, num_robots);
    // lap <<  1, -1, 0, 0, 0,
    //         -1, 3, -1, 0, -1,
    //         0, -1, 3, -1, -1,
    //         0, 0, -1, 2, -1,
    //         0, -1, -1, -1, 3;


    /* Mobile robot poses and for next poses */
    Eigen::MatrixXd cur_pos(num_robots, 3);
    Eigen::MatrixXd del_pos(num_robots, 3);
    Eigen::VectorXd del_consist_item(num_robots);
    Eigen::VectorXd del_theta(num_robots);
    Eigen::VectorXd dist_factor(num_robots);
    Eigen::VectorXd force_x(num_robots);
    Eigen::VectorXd force_y(num_robots);

    /* store position from camera */
    std::vector<std::vector<double> > current_robot_pose(num_robots);

    /* Convergence sign */
    bool is_conv = false;   

    /* While loop */
    while(! is_conv) {
        /* Get current swarm robot poses */
        swarm_robot.getRobotPose(current_robot_pose);
        
        // copy robot positions to matrix for matrix operation
        copy_to_matrix(current_robot_pose, cur_pos);

        /* Judge whether reached */
        del_pos = -lap * cur_pos;
        del_consist_item = del_pos.block(0, 0, del_pos.rows(), 2) * line;
        del_theta = del_pos.col(2);
        
        bool angle_conv = all_converge(del_theta);
        bool pos_conv = all_converge(del_consist_item);
        is_conv = pos_conv && angle_conv;

        if (is_conv) {
            break;
        }

        // compute force on each robot & distance coefficient
        for (int i = 0; i < num_robots; ++i) {
            force_x(i) = 0;
            force_y(i) = 0;
            dist_factor(i) = 0;
            for (int j = 0; j < num_robots; ++j) {
                if (i == j) continue;
                double radius = sqrt(pow(cur_pos(i, 0) - cur_pos(j, 0), 2) + pow(cur_pos(i, 1) - cur_pos(j, 1), 2));
                force_x(i) += config::force_coef 
                                     / radius / radius
                                     * (cur_pos(i, 0) - cur_pos(j, 0))
                                     / radius;
                force_y(i) += config::force_coef 
                                     / radius / radius
                                     * (cur_pos(i, 1) - cur_pos(j, 1))
                                     / radius;
                dist_factor(i) += pow(sqrt(pow(cur_pos(i, 0) - cur_pos(j, 0), 2) + pow(cur_pos(i, 1) - cur_pos(j, 1), 2)) -
                                    config::MIN_dist, 2);
            }
        }

        /* Swarm robot move */
        for (int i = 0; i < num_robots; ++i) {
            double vec = (del_consist_item(i)) * config::k_v;
            double w = del_theta(i) * config::k_w;

            // add rejection effect on vec and w
            double del_vec = force_x(i) * std::cos(cur_pos(i, 2)) + force_y(i) * std::sin(cur_pos(i, 2));
            vec += del_vec;
            double del_w = config::w_coef * (force_x(i) * std::sin(cur_pos(i, 2)) + force_y(i) * std::cos(cur_pos(i, 2)));
            w += del_w;
            
            // print Info for debug
            std::cout << "num: " << i << std::endl << "vec: " << vec << std::endl << "w: " << w << std::endl;
            std::cout << "del_vec: " << del_vec << std::endl << "del_w: " << del_w << std::endl;
            
            // add consistant connection affect
            vec /= dist_factor(i) * config::MIN_dist_coef;
            
            // addjust vec and w to appropriate range
            vec = swarm_robot.checkVel(vec, config::MAX_V, config::MIN_V);
            w = swarm_robot.checkVel(w, config::MAX_W, config::MIN_W);

            if (angle_conv) {
                swarm_robot.moveRobot(i, vec, 0.0);
            } else if (pos_conv) {
                swarm_robot.moveRobot(i, 0.0, w);
            } else {
                swarm_robot.moveRobot(i, vec, w);
            }
        }

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();
    }

    /* Stop all robots */
    swarm_robot.stopRobot();

    ROS_INFO_STREAM("Succeed!");
    return 0;
}

void copy_to_matrix(std::vector<std::vector<double> >& vec, Eigen::MatrixXd& mat) {
    for (int i = 0; i < vec.size(); ++i) {
        for (int j = 0; j < vec[0].size(); ++j) {
            mat(i, j) = vec[i][j];
        }
    }
}

bool all_converge(Eigen::VectorXd& vec) {
    auto conv = [](double elem) { return std::fabs(elem) < config::conv_th; };
    return std::all_of(vec.data(), vec.data()+vec.size(), conv);
}
