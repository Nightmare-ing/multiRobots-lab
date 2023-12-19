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

    double k_w = 1;       // Scale of angle velocity
    double k_v = 0.2;        // Scale of linear velocity
    double force_coef = 0.2;  // coefficient to replace G etc. in gravity formula
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

    /* Target position of the robots */
    Eigen::MatrixXd targ_pos(num_robots, 2);
    double radius = 0.5;
    targ_pos << radius*1, radius*0,
                radius*0.30902, radius*0.95106,
                radius*(-0.80902), radius*0.58779,
                radius*(-0.80902), radius*(-0.58779),
                radius*0.30902, radius*(-0.95106);
    /*targ_pos << 1, 2,
        -1, 2,
        0, -2,
        1.5, 0,
        -1.5, 0; */

    /* Mobile robot poses and for next poses */
    Eigen::MatrixXd cur_pos(num_robots, 3);
    Eigen::MatrixXd del_pos(num_robots, 2);
    Eigen::VectorXd del_consist_item(num_robots);
    Eigen::VectorXd force_x(num_robots);
    Eigen::VectorXd force_y(num_robots);
    Eigen::MatrixXd expt_drct(num_robots, 2);

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
        del_pos = -lap * (cur_pos.leftCols(2) - targ_pos);
        
        bool pos_conv = all_converge(del_consist_item);
        is_conv = false;

        if (is_conv) {
            break;
        }

        // compute force on each robot & distance coefficient
        for (int i = 0; i < num_robots; ++i) {
            force_x(i) = 0;
            force_y(i) = 0;
            for (int j = 0; j < num_robots; ++j) {
                if (i == j) continue;
                double radius = sqrt(pow(cur_pos(i, 0) - cur_pos(j, 0), 2) + pow(cur_pos(i, 1) - cur_pos(j, 1), 2));
                force_x(i) += 1 / radius / radius
                                     * (cur_pos(i, 0) - cur_pos(j, 0))
                                     / radius;
                force_y(i) += 1 / radius / radius
                                     * (cur_pos(i, 1) - cur_pos(j, 1))
                                     / radius;
            }
            
            expt_drct(i, 0) = force_x(i) * config::force_coef + del_pos(i, 0);
            expt_drct(i, 1) = force_y(i) * config::force_coef + del_pos(i, 1);
        }

        /* Swarm robot move */
        for (int i = 0; i < num_robots; ++i) {
            double del_angle = (std::atan2(expt_drct(i, 1), expt_drct(i, 0)) - cur_pos(i, 2));
            // modify w to rotate in a more efficient direction
            del_angle = (M_PI - std::abs(del_angle)) / std::abs(std::abs(del_angle) - M_PI) * del_angle;
            double w = del_angle * config::k_w;

            double vec = 0;
            double bar_angle = 15;
            if (del_angle < bar_angle / 180 * M_PI && del_angle > -bar_angle / 180 * M_PI) {
                vec = sqrt(expt_drct(i, 0) * expt_drct(i, 0) + expt_drct(i, 1) * expt_drct(i, 1)) * std::cos(del_angle) * config::k_v;
            }
            
            // addjust vec and w to appropriate range
            vec = swarm_robot.checkVel(vec, config::MAX_V, config::MIN_V);
            w = swarm_robot.checkVel(w, config::MAX_W, config::MIN_W);

            /*
            if (angle_conv) {
                swarm_robot.moveRobot(i, vec, 0.0);
            } else if (pos_conv) {
                swarm_robot.moveRobot(i, 0.0, w);
            } else {
                swarm_robot.moveRobot(i, vec, w);
            }
            */
            swarm_robot.moveRobot(i, vec, w);
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