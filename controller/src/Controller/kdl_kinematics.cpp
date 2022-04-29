#include "ros/ros.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <iostream> 
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <controller/Kinematics_msg.h>
#include <mutex>
#include <abb_egm_msgs/EGMState.h>

#include <ros/package.h>

// mutex 
std::mutex mtx_reciving;


 
void jacobian_data(int dof, KDL::Jacobian jacobian_right, KDL::Jacobian jacobian_left, std_msgs::Float64MultiArray* jac){
    // function for packaging jacobinas into std_msgs::Float64MultiArray message

    std::vector<double> data_msg(6*dof*2); // number of elements in jacobian = 6*7*2 for Dof = 7, i.e. 7 joints on each arm  

    for (int i = 0; i < 6; i++){
        for (int j = 0; j < dof; j++){
            for(int k = 0; k <2; k++){
                data_msg[ 2*dof*i + 2*j + k] = (k == 0) ? jacobian_right(i,j):jacobian_left(i,j);
            }
        }
    }
    
    std_msgs::MultiArrayLayout msg_layout;
    std::vector<std_msgs::MultiArrayDimension> msg_dim(3);
    msg_dim[0].label = "height";
    msg_dim[0].size = 6;
    msg_dim[0].stride = 6*dof*2;
    msg_dim[1].label = "width";
    msg_dim[1].size = dof;
    msg_dim[1].stride = 2*dof;
    msg_dim[2].label = "channel";
    msg_dim[2].size = 2;
    msg_dim[2].stride = 2;

    msg_layout.dim = msg_dim;
    msg_layout.data_offset = 0;

    jac->layout = msg_layout;
    jac->data = data_msg;
}

void pose_data(KDL::Frame frame, geometry_msgs::Pose* pose){
    // function geting geometry_msgs::Pose message from KDL::Frame
    double Qx;
    double Qy;
    double Qz;
    double Qw;

    frame.M.GetQuaternion(Qx,Qy,Qz,Qw);

    pose->orientation.x = Qx;
    pose->orientation.y = Qy;
    pose->orientation.z = Qz;
    pose->orientation.w = Qw;

    pose->position.x = frame.p.data[0];
    pose->position.y = frame.p.data[1];
    pose->position.z = frame.p.data[2];
}


class Calc_jacobian{
    // class for most of the kinamtics calcualtions 
    
    private:  // private variable declarations 
    // define ros subscribers and publishers 
    ros::Subscriber joint_state_sub;
    ros::Subscriber egm_state_sub;

    ros::Publisher velocity_pub;
    ros::Publisher jacobian_pub;
    ros::Publisher joint_states_pub;

    // define kdl tree and chain for each arm
    KDL::Tree yumi_tree;
    KDL::Chain yumi_right_arm;
    KDL::Chain yumi_left_arm;
    
    KDL::Chain yumi_right_elbow;
    KDL::Chain yumi_left_elbow;
    KDL::Chain yumi_left_right;

    // define jacobian variables
    KDL::Jacobian jacobian_right_arm = KDL::Jacobian(7);
    KDL::Jacobian jacobian_left_arm = KDL::Jacobian(7);

    KDL::Jacobian jacobian_right_elbow = KDL::Jacobian(4);
    KDL::Jacobian jacobian_left_elbow = KDL::Jacobian(4);

    // define frame variables
    KDL::Frame frame_right_arm; 
    KDL::Frame frame_left_arm; 

    KDL::Frame frame_right_elbow; 
    KDL::Frame frame_left_elbow; 
    KDL::Frame frame_left_right; 

    // joint values -- assuming same as defined in urdf
    KDL::JntArray q_right_arm = KDL::JntArray(7);
    KDL::JntArray q_left_arm = KDL::JntArray(7);

    KDL::JntArray q_right_elbow = KDL::JntArray(4);
    KDL::JntArray q_left_elbow = KDL::JntArray(4);
 
    KDL::JntArray q_left_right = KDL::JntArray(14);

    // jacobian solvers
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_right_arm;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_left_arm;

    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_right_elbow;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_left_elbow;

    // forward kinematics solvers

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_right_arm;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_left_arm;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_left_to_right;

    // order input data

    std::vector<double> joint_state;
    std::vector<double> joint_velocity;

    // naming for Rviz visualization
    std::string joint_name_list[18] = {"yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r",
         "yumi_joint_3_r", "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r", 
         "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l", 
         "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l", "gripper_r_joint",
         "gripper_r_joint_m", "gripper_l_joint", "gripper_l_joint_m"};

    // naming for ABB driver, ABB in inconsistent with naming joint names!
    std::string name_list[18] = {"yumi_robr_joint_1", "yumi_robr_joint_2", "yumi_robr_joint_3", "yumi_robr_joint_4",
         "yumi_robr_joint_5", "yumi_robr_joint_6", "yumi_robr_joint_7", "yumi_robl_joint_1", "yumi_robl_joint_2", "yumi_robl_joint_3",
         "yumi_robl_joint_4", "yumi_robl_joint_5", "yumi_robl_joint_6", "yumi_robl_joint_7"};

    // joint offset for calibration/tuning [rad], program needs to be recompiled after change!
    // follows joint_name_list, i.e. on the real robot from shoulder to wrist. 
    std::vector<double> joint_offset = {-0.0155479, -0.0059908, -0.006814363, -0.02071828, -0.000571185, 0.01696876, 0.0007526933,
                                         -0.00090386355, -0.0052244536, -0.000945309, 0.006259396, 0.003126351383, 0.0016362062, -0.0004466520};

    // egm active for both arms, safety system for if arms not started properly, though doesnt cover everything
    bool egm_active = false;

    // public attributes to class
    public:
    int state_recived = 0;

    // constructor
    Calc_jacobian(ros::NodeHandle *nh );
    // just init pose for initial visualization 
    std::vector<double> init_joint_pos = {1.0, -2.0, -1.2, 0.6, -2.0, 1.0, 0.0, -1.0, -2.0, 1.2, 0.6, 2.0, 1.0, 0.0, 0.0,0.0,0.0,0.0};
    // functions 

    // saves input data from driver
    void callback(const sensor_msgs::JointState::ConstPtr& joint_state_data);
    // function detemines if egm session is ative or not. 
    void callback_egm_state(const abb_egm_msgs::EGMState::ConstPtr& egm_state_data);

    // does all calulations and sends to controller 
    void update();
};

// Member functions definitions

// constructor, gets called when class instance is created
Calc_jacobian::Calc_jacobian(ros::NodeHandle *nh ){
    // setup the supscribers and publishers
    joint_state_sub = nh->subscribe("/yumi/egm/joint_states", 2, &Calc_jacobian::callback, this);
    egm_state_sub = nh->subscribe("/yumi/egm/egm_states", 2, &Calc_jacobian::callback_egm_state, this);

    jacobian_pub = nh->advertise<controller::Kinematics_msg>("/Jacobian_R_L", 1);
    joint_states_pub = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);
    velocity_pub = nh->advertise<std_msgs::Float64MultiArray>("/yumi/egm/joint_group_velocity_controller/command", 1);

    // get tree from urdf file for entire yumi
    std::string const PATH = ros::package::getPath("yumi_description"); // find path 
    
    if (!kdl_parser::treeFromFile(PATH + "/urdf/yumi.urdf", yumi_tree)){ // create tree
        ROS_ERROR("Failed to construct kdl tree");
    }
    
    // get chain for each arm from tree
    yumi_tree.getChain("yumi_base_link","yumi_link_7_r", yumi_right_arm);
    yumi_tree.getChain("yumi_base_link","yumi_link_7_l", yumi_left_arm);    

    yumi_tree.getChain("yumi_base_link","yumi_link_4_r", yumi_right_elbow);
    yumi_tree.getChain("yumi_base_link","yumi_link_4_l", yumi_left_elbow);  
  
    // Jacobians
    jac_solver_right_arm = std::make_unique<KDL::ChainJntToJacSolver>(yumi_right_arm);
    jac_solver_left_arm = std::make_unique<KDL::ChainJntToJacSolver>(yumi_left_arm);

    jac_solver_right_elbow = std::make_unique<KDL::ChainJntToJacSolver>(yumi_right_elbow);
    jac_solver_left_elbow = std::make_unique<KDL::ChainJntToJacSolver>(yumi_left_elbow);

    // Forward kinematics solver
    fk_solver_right_arm = std::make_unique<KDL::ChainFkSolverPos_recursive>(yumi_right_arm);
    fk_solver_left_arm = std::make_unique<KDL::ChainFkSolverPos_recursive>(yumi_left_arm);
    joint_state.resize(18);
    joint_velocity.resize(14);
}


void Calc_jacobian::callback(const sensor_msgs::JointState::ConstPtr& joint_state_data){
    // sort input data and add joint offset
    mtx_reciving.lock();
    for (int i = 0; i < 14; i++){
        for (int j = 0; j < 14; j++){
            if (name_list[i].compare(joint_state_data->name[j]) == 0 ){
                joint_state[i] = joint_state_data->position[j];// + joint_offset[i];
                joint_velocity[i] = joint_state_data->velocity[j];
                break;
            }
        }
    }

    int num_elements = joint_state_data->position.size();
    if (num_elements > 14){ // for simulation
        joint_state[14] = joint_state_data->position[14];
        joint_state[15] = joint_state_data->position[15];
        joint_state[16] = joint_state_data->position[16];
        joint_state[17] = joint_state_data->position[17];      
    }
    else{ // real robot do not give values for gripper position 
        joint_state[14] = 0.0;
        joint_state[15] = 0.0;
        joint_state[16] = 0.0;
        joint_state[17] = 0.0;
    }
    state_recived = 1; // makes sure that nothing is sent until any data has been received 
    mtx_reciving.unlock();

    // publish joint states, for visualization in Rviz
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    for (int i = 0; i < 18; i++){
        msg.name.push_back(joint_name_list[i]);
        msg.position.push_back(joint_state[i]);
    }
   
    joint_states_pub.publish(msg);

}

// Check if both arms have active egm sessions
void Calc_jacobian::callback_egm_state(const abb_egm_msgs::EGMState::ConstPtr& egm_state_data){
    if (egm_state_data->egm_channels[0].active == true && egm_state_data->egm_channels[1].active == true){
        egm_active = true;
    }
    else{
        egm_active = false;
    }
}

// makes most calculations and sends to the controller
void Calc_jacobian::update(){
    // if both egm session not available, publishes 0 velocity command incase one of them is active 
    if (egm_active == false){
        std_msgs::Float64MultiArray msg;
        std::vector<double> msg_data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        msg.data = msg_data;
        velocity_pub.publish(msg);
        // set initial joint pose for visualization
        sensor_msgs::JointState msg_;
        msg_.header.stamp = ros::Time::now();
        for (int i = 0; i < 18; i++){
            msg_.name.push_back(joint_name_list[i]);
            msg_.position.push_back(init_joint_pos[i]);
        }
        std::cout << "inside loop" << std::endl;
        joint_states_pub.publish(msg_);
        return;
    }

    mtx_reciving.lock();

   // joints state to kdl variables 
    for (int i= 0 ; i < 7; i++ ){
        q_right_arm(i) = joint_state[i];
        q_left_arm(i) = joint_state[i+7];
        q_left_right(i) = joint_state[i+7];
        q_left_right(i+7) = joint_state[i];
        if (i < 4){
            q_right_elbow(i) = joint_state[i];
            q_left_elbow(i) = joint_state[i+7];
        }
    }

    // --------------------- Jacobians --------------------------------------------------
    controller::Kinematics_msg kinematics_msg;
    // send joint position 
    kinematics_msg.jointPosition = joint_state;
    kinematics_msg.jointVelocity = joint_velocity;

    mtx_reciving.unlock();

    // arm 
    jac_solver_right_arm->JntToJac(q_right_arm, jacobian_right_arm);
    jac_solver_left_arm->JntToJac(q_left_arm, jacobian_left_arm);
    
    std_msgs::Float64MultiArray jac;

    jacobian_data(7, jacobian_right_arm, jacobian_left_arm, &jac);

    kinematics_msg.jacobian.push_back(jac);

    // elbow 
    jac_solver_right_elbow->JntToJac(q_right_elbow, jacobian_right_elbow);
    jac_solver_left_elbow->JntToJac(q_left_elbow, jacobian_left_elbow);

    jacobian_data(4, jacobian_right_elbow, jacobian_left_elbow, &jac);

    kinematics_msg.jacobian.push_back(jac);

    kinematics_msg.header.stamp = ros::Time::now();

    // --------------------- Forward Kinematics --------------------------------------------------
    // last frame has number 8!
    fk_solver_right_arm->JntToCart(q_right_arm, frame_right_arm, 8);
    fk_solver_left_arm->JntToCart(q_left_arm, frame_left_arm, 8);

    fk_solver_right_arm->JntToCart(q_right_arm, frame_right_elbow, 5);
    fk_solver_left_arm->JntToCart(q_left_arm, frame_left_elbow, 5);

    geometry_msgs::Pose pose;

    pose_data(frame_right_arm, &pose);
    kinematics_msg.forwardKinematics.push_back(pose);
 
    pose_data(frame_left_arm, &pose);
    kinematics_msg.forwardKinematics.push_back(pose);

    pose_data(frame_right_elbow, &pose);
    kinematics_msg.forwardKinematics.push_back(pose);

    pose_data(frame_left_elbow, &pose);
    kinematics_msg.forwardKinematics.push_back(pose);

    // publish msg
    jacobian_pub.publish(kinematics_msg);

    ros::spinOnce(); // spin ros and send msg direcly 
}


int main(int argc, char** argv){
    // ROS
    ros::init(argc, argv, "kdl_kinematics");
    ros::NodeHandle nh;
    // multithreaded spinner 
    ros::AsyncSpinner spinner(0);
    spinner.start();
    // main class 
    Calc_jacobian calc_jacobian(&nh);
    // Hz update fequenzy, this detemines the update frequenzy in the contollerMaster.py, 
    // needs to be the same in both places
    ros::Rate loop_rate(50);


    while (ros::ok()){
        //if (calc_jacobian.state_recived >= 1){ // Do not sen anython if nothing has been recived 
        calc_jacobian.update();
        std::cout << "update loop" << std::endl;
        //}
        loop_rate.sleep();
    }


    ros::waitForShutdown();

    return 0;
}