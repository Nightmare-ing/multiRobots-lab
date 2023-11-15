/* 
 * Date: 2021-11-29
 * Description: move robots to one line
*/

#include <algorithm>
#include <asm-generic/errno.h>
#include <swarm_robot_control.h>
#include <vector>

using std::string;

// Util functions
void copy_to_matrix(std::vector<std::vector<double> >& vec, Eigen::MatrixXd& mat);
bool all_converge(Eigen::VectorXd& vec);
bool all_converge(Eigen::MatrixXd& mat);
void generate_pos(const string& expected_queue, Eigen::MatrixXd& expected_pos);
void generate_gd(Eigen::VectorXd& gd, const Eigen::MatrixXd& pos);
void generate_Rg(Eigen::MatrixXd& Rg, const Eigen::VectorXd& pos);

namespace config {
    /* Convergence threshold */
    double conv_th = 0.05;  // Threshold of angle, in rad

    /* Velocity scale and threshold */
    double MAX_W = 1;       // Maximum angle velocity (rad/s)
    double MIN_W = 0.05;    // Minimum angle velocity(rad/s)
    double MAX_V = 0.2;     // Maximum linear velocity(m/s)
    double MIN_V = 0.01;    // Minimum linear velocity(m/s)
    double k_w = 0.8;       // Scale of angle velocity
    double k_v = 1;        // Scale of linear velocity

    /**
     * @brief several params for avoiding collision
     * Avoiding strategy: imitate gravity but using rejection not attraction
     */
    double MIN_dist = 3;  // minimun distance between robots to avoid moving far awary
    double MIN_dist_coef = 0.1;
    double force_coef = 0.08;  // coefficient to replace G etc. in gravity formula
    double w_coef = 0.03;   // scale the impact of rejection on w
    
    std::string expected_queue = "wedge";
}



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
    
    Eigen::MatrixXd expected_pos(num_robots, 3);
    generate_pos(config::expected_queue, expected_pos);
    Eigen::VectorXd expected_gd(2 * num_robots - 3);
    generate_gd(expected_gd, expected_pos);
    std::cout << expected_gd << std::endl;


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
    Eigen::MatrixXd cur_vector(num_robots, 3);
    Eigen::MatrixXd diff_vector(num_robots, 3);
    Eigen::VectorXd del_pos_x(num_robots);
    Eigen::VectorXd del_pos_y(num_robots);
    Eigen::VectorXd del_theta(num_robots);
    Eigen::VectorXd dist_factor(num_robots);
    Eigen::VectorXd force_x(num_robots);
    Eigen::VectorXd force_y(num_robots);
    Eigen::VectorXd cur_gd(2 * num_robots - 3);
    Eigen::MatrixXd R_g_x(2 * num_robots - 3, num_robots);
    Eigen::MatrixXd R_g_y(2 * num_robots - 3, num_robots);


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
        generate_gd(cur_gd, cur_pos);
        
        generate_Rg(R_g_x, cur_pos.col(0));
        generate_Rg(R_g_y, cur_pos.col(1));
        std::cout << R_g_x << std::endl << R_g_y << std::endl;

        /* Judge whether reached */
        del_pos_x = R_g_x.transpose() * (expected_gd - cur_gd);
        del_pos_y = R_g_y.transpose() * (expected_gd - cur_gd);
        std::cout << del_pos_x << std::endl << del_pos_y << std::endl;
        
        bool x_conv = all_converge(del_pos_x);
        bool y_conv = all_converge(del_pos_y);
        is_conv = x_conv && y_conv;

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
            double vec = (del_pos_x(i) * std::cos(cur_pos(i, 2)) + del_pos_y(i) * std::sin(cur_pos(i, 2))) * config::k_v;
            double w =  (del_pos_x(i) * std::sin(cur_pos(i, 2)) + del_pos_y(i) * std::cos(cur_pos(i, 2)))* config::k_w;

            std::cout << "vec: " << vec << std::endl << "w: " << w << std::endl;

            // add rejection effect on vec and w
            double del_vec = force_x(i) * std::cos(cur_pos(i, 2)) + force_y(i) * std::sin(cur_pos(i, 2));
            vec += del_vec;
            double del_w = config::w_coef * (force_x(i) * std::sin(cur_pos(i, 2)) + force_y(i) * std::cos(cur_pos(i, 2)));
            w += del_w;
            
            // print Info for debug
            // std::cout << "num: " << i << std::endl << "vec: " << vec << std::endl << "w: " << w << std::endl;
            // std::cout << "del_vec: " << del_vec << std::endl << "del_w: " << del_w << std::endl;
            
            // add consistant connection affect
            vec /= dist_factor(i) * config::MIN_dist_coef;
            // std::cout << "dist_factor(i): " << dist_factor(i) << std::endl << "vec: " << vec << std::endl;
            
            // addjust vec and w to appropriate range
            vec = swarm_robot.checkVel(vec, config::MAX_V, config::MIN_V);
            w = swarm_robot.checkVel(w, config::MAX_W, config::MIN_W);
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

bool all_converge(Eigen::MatrixXd& mat) {
    bool flag = true;
    auto conv = [](double elem) { return std::fabs(elem) < config::conv_th; };
    for (int i = 0; i < mat.rows(); ++i) {
        flag &= std::all_of(mat.row(i).data(), mat.row(i).data()+mat.cols(), conv);
    }
    return flag;
}

void generate_pos(const string& expected_queue, Eigen::MatrixXd& expected_pos) {
    int num_robots = expected_pos.rows();
    if (expected_queue == "circle") {
        double angle_rad = 2.0 * M_PI / num_robots;
        for (int i = 0; i < num_robots; ++i) {
            expected_pos.row(i) << std::cos(M_PI/2.0 - i * angle_rad), std::sin(M_PI/2.0 - i * angle_rad), 0;
        }
        std::cout << expected_pos<< endl;
        return;
    }
    if (expected_queue == "star") {
        expected_pos << 0, 0, 0,
                        1.5, 0, 0,
                        0, 1.5, 0,
                        -1.5, 0, 0,
                        0, -1.5, 0;
        return;
    }
    if (expected_queue == "wedge") {
        expected_pos << 1.0 / 2, sqrt(3) / 2.0, 0,
                        0, sqrt(3), 0,
                        -1.0 / 2, sqrt(3) / 2.0, 0, 
                        -1.0, 0, 0, 
                        1, 0, 0;
        return;
    }
}

void generate_gd(Eigen::VectorXd& gd, const Eigen::MatrixXd& pos) {
    gd << pow(pos(0, 0) - pos(1, 0), 2) + pow(pos(0, 1) - pos(1, 1), 2),
          pow(pos(0, 0) - pos(2, 0), 2) + pow(pos(0, 1) - pos(2, 1), 2),
          pow(pos(0, 0) - pos(3, 0), 2) + pow(pos(0, 1) - pos(3, 1), 2),
          pow(pos(0, 0) - pos(4, 0), 2) + pow(pos(0, 1) - pos(4, 1), 2),
          pow(pos(1, 0) - pos(2, 0), 2) + pow(pos(1, 1) - pos(2, 1), 2),
          pow(pos(2, 0) - pos(3, 0), 2) + pow(pos(2, 1) - pos(3, 1), 2),
          pow(pos(3, 0) - pos(4, 0), 2) + pow(pos(3, 1) - pos(4, 1), 2);
}
void generate_Rg(Eigen::MatrixXd& Rg, const Eigen::VectorXd& pos) {
    Rg << pos(0) - pos(1), pos(1) - pos(0), 0, 0, 0,
          pos(0) - pos(2), 0, pos(2) - pos(0), 0, 0,
          pos(0) - pos(3), 0, 0, pos(3) - pos(0), 0,
          pos(0) - pos(4), 0, 0, 0, pos(4) - pos(0),
          0, pos(1) - pos(2), pos(2) - pos(1), 0, 0, 
          0, 0, pos(2) - pos(3), pos(3) - pos(2), 0,
          0, 0, 0, pos(3) - pos(4), pos(4) - pos(3);
}