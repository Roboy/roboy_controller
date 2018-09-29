#include "roboy_controller/roboy.hpp"


Roboy::Roboy() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy", ros::init_options::NoRosout);
    }

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));
    spinner->start();

    string sdf;
    nh.getParam("robot_description_sdf", sdf);

    ROS_DEBUG("found robot_description_sdf: \n\n%s\n\n", sdf.c_str());

    vector<string> endeffectors;
    nh.getParam("end_effectors", endeffectors);

    vector<MuscInfo> muscInfo;
    if (!CASPR::parseSDFusion(sdf, muscInfo))
        ROS_FATAL("error parsing sdf");

    stringstream str;
    str << "initialized CASPR controllers for endeffectors:" << endl;
    for (auto endeffector:endeffectors) {
        vector<string> chain;
        nh.getParam(endeffector + "/kinematic_chain", chain);
        caspr.push_back(boost::shared_ptr<CASPR>(new CASPR(chain.front(), chain.back(), muscInfo)));
        casprByName[endeffector] = caspr.back();
        str << endeffector << endl;
    }
    ROS_INFO_STREAM(str.str());

    str.clear();
    for (auto casp:caspr) {
        for (int j = 0; j < casp->number_of_dofs; j++) {
            if (((casp->joint_angle_mask >> j) & 0x1) == 0) {
                if (find(joint_names.begin(), joint_names.end(), casp->joint_names[j]) != joint_names.end()) {
                    ROS_FATAL("mmultiple endeffectors are controlling joint %s, check your joint angle masks",
                              casp->joint_names[j].c_str());
                } else {
                    joint_names.push_back(casp->joint_names[j]);
                    q[casp->joint_names[j]] = &casp->q[j];
                    qd[casp->joint_names[j]] = &casp->qd[j];
                    str << casp->joint_names[j] << "\t";
                }
            }
        }
    }
    ROS_INFO_STREAM(str.str());

    fmt = Eigen::IOFormat(4, 0, " ", ";\n", "", "", "[", "]");

    reset_srv = nh.advertiseService("/CASPR/reset", &Roboy::ResetService, this);
    motor_config_srv = nh.serviceClient<roboy_communication_middleware::MotorConfigService>(
            "/roboy/shoulder_left/middleware/MotorConfig");
    clearAll();

    lookAt_as.reset(new actionlib::SimpleActionServer<roboy_communication_control::LookAtAction>(nh, "Roboy/LookAt",
                                                                                                 boost::bind(
                                                                                                         &Roboy::lookAt,
                                                                                                         this, _1),
                                                                                                 false));
    lookAt_as->start();
    moveEndEffector_as.reset(new actionlib::SimpleActionServer<roboy_communication_control::MoveEndEffectorAction>(nh,
                                                                                                                   "Roboy/MoveEndEffector",
                                                                                                                   boost::bind(
                                                                                                                           &Roboy::moveEndEffector,
                                                                                                                           this,
                                                                                                                           _1),
                                                                                                                   false));
    moveEndEffector_as->start();
}

Roboy::~Roboy() {
}

void Roboy::sendToRealHardware(CASPRptr casp) {
    int number_of_motors = 0;
    nh.getParam(casp->end_effektor_name + "/number_of_motors", number_of_motors);
    roboy_communication_middleware::MotorCommand msg;
    msg.id = casp->id;
    vector<int> motors = active_motors[casp->id];
    switch (casp->id) {
        case HEAD: {
            for (int i = 0; i < number_of_motors; i++) {
                msg.motors.push_back(motors[i]);
                switch (motor_type[HEAD][i]) {
                    case MYOBRICK100N:
                        msg.setPoints.push_back(-myoBrick100NEncoderTicksPerMeter(casp->motor_pos[i]));
                        break;
                    case MYOBRICK300N:
                        msg.setPoints.push_back(-myoBrick300NEncoderTicksPerMeter(casp->motor_pos[i]));
                        break;

                }
            }
            casp->motorcommand_pub.publish(msg);
            std_msgs::Float32 msg2;
            break;
        }
        case SHOULDER_LEFT: {
            roboy_communication_middleware::MotorCommand msg;
            msg.id = SHOULDER_LEFT;
            for (int i = 0; i < number_of_motors; i++) {
                msg.motors.push_back(motors[i]);
                switch (motor_type[SHOULDER_LEFT][i]) {
                    case MYOMUSCLE500N:
                        msg.setPoints.push_back(-myoMuscleEncoderTicksPerMeter(
                                (casp->motor_pos[i] - casp->displacement_real[i] * 2.0))); //
                        break;
                    case MYOBRICK100N:
                        msg.setPoints.push_back(-myoBrick100NEncoderTicksPerMeter(casp->motor_pos[i]));
                                                                                  //- casp->displacement_real[i] * 2.0));
                        break;
                    case MYOBRICK300N:
                        msg.setPoints.push_back(-myoBrick300NEncoderTicksPerMeter(casp->motor_pos[i]));
                                                                                  //- casp->displacement_real[i] * 2.0));
                        break;

                }
            }
            casp->motorcommand_pub.publish(msg);
            std_msgs::Float32 msg2;
            msg2.data = casp->q[4] * 180.0 / M_PI;
            casp->elbow_joint_pub.publish(msg2);
            msg2.data = casp->q[5] * 180.0 / M_PI;
            casp->wrist_joint_pub.publish(msg2);
            break;
        }
        case 5:{

            break;
        }
    }
}

void Roboy::main_loop() {
    info_time_prev = ros::Time::now();
    while (ros::ok()) {
        if ((ros::Time::now() - info_time_prev).toSec() > 3) {
            ROS_INFO_STREAM(caspr[info_counter]->q_target.transpose().format(fmt)
                                    << "\ne="
                                    << caspr[info_counter]->e.transpose().format(fmt)
                                    << "\nde="
                                    << caspr[info_counter]->de.transpose().format(fmt)
                                    << "\ntorques="
                                    << caspr[info_counter]->torques.transpose().format(fmt)
                                    << "\nl_dot="
                                    << caspr[info_counter]->l_dot.transpose().format(fmt));
            info_counter++;
            if (info_counter > (caspr.size() - 1))
                info_counter = 0;
            info_time_prev = ros::Time::now();
            if (!moveEndEffector_as->isActive() && !lookAt_as->isActive()) {
                for (auto casp:caspr) {
                    casp->update(0.00001);
                    casp->updateController();
                }
            }
        }
    }
}

void Roboy::lookAt(const roboy_communication_control::LookAtGoalConstPtr &goal) {
    roboy_communication_control::LookAtFeedback feedback;
    roboy_communication_control::LookAtResult result;

    CASPRptr casp = casprByName[goal->endEffector];
    if (casp == nullptr) {
        ROS_WARN_STREAM("MoveEndEffector: FAILED endeffector " << goal->endEffector << " does not exist");
        lookAt_as->setAborted(result, "endeffector " + goal->endEffector + " does not exist");
        return;
    }

    if (goal->sendToRealHardware) {
        setControlMode(casp);
    }

    bool success = true;
    tf::StampedTransform root_transform, target_transform;

    nh.getParam("controller", casp->controller);
    double timestep = 0.001;
    switch (casp->controller) {
        case 0:
            nh.getParam(casp->end_effektor_name + "/ForceControl/Kp", casp->Kp);
            nh.getParam(casp->end_effektor_name + "/ForceControl/Kd", casp->Kd);
            timestep = 0.001;
            break;
        case 1:
            nh.getParam(casp->end_effektor_name + "/TorqueControl/Kp", casp->Kp);
            nh.getParam(casp->end_effektor_name + "/TorqueControl/Kd", casp->Kd);
            break;
        case 2:
            nh.getParam(casp->end_effektor_name + "/PositionControl/Kp", casp->Kp);
            nh.getParam(casp->end_effektor_name + "/PositionControl/Kd", casp->Kd);
            timestep = 0.00001;
            break;
    }

    Eigen::Affine3d root, target;
    if (goal->type == 0) {
        Vector3d t(goal->point.x, goal->point.y, goal->point.z);
        target.setIdentity();
        target.translate(t);
    } else if (goal->type == 2) {
        if (listener.waitForTransform("world", goal->target_frame.c_str(), ros::Time(0), ros::Duration(0.01))) {
            listener.lookupTransform("world", goal->target_frame.c_str(), ros::Time(0), target_transform);
            tf::transformTFToEigen(target_transform, target);
        }
    }

    double error = 10000;
    ros::Time last_feedback_time = ros::Time::now(), start_time = ros::Time::now();
    while (error > goal->tolerance && success) {
        try {
            if (listener.waitForTransform("world", goal->root_frame.c_str(), ros::Time(0), ros::Duration(0.01))) {
                listener.lookupTransform("world", goal->root_frame.c_str(), ros::Time(0), root_transform);
            }
            if (goal->type == 1) {
                if (listener.waitForTransform("world", goal->target_frame.c_str(), ros::Time(0), ros::Duration(0.01))) {
                    listener.lookupTransform("world", goal->target_frame.c_str(), ros::Time(0), target_transform);
                    tf::transformTFToEigen(target_transform, target);
                }
            }
        }
        catch (tf::LookupException ex) {
            ROS_WARN_THROTTLE(1, "%s", ex.what());
            success = false;
        }
        tf::transformTFToEigen(root_transform, root);
        Vector3d eye_ray(0, -1, 0), target_ray = (target.matrix().block(0, 3, 3, 1) - root.matrix().block(0, 3, 3, 1));
        target_ray.normalize();
        eye_ray = root.matrix().block(0, 0, 3, 3) * eye_ray;

        Vector3d pos = root.matrix().block(0, 3, 3, 1);
        publishRay(pos, eye_ray, "world", "eye_ray", 1111111, COLOR(1, 0, 0, 0.2), 0.1);
        publishRay(pos, target_ray, "world", "target_ray", 1111112, COLOR(0, 1, 0, 0.2), 0.1);

        casp->q_target[goal->yaw_joint_index] = casp->q[goal->yaw_joint_index] - (target_ray[2] - eye_ray[2]);
        casp->q_target[goal->pitch_joint_index] = casp->q[goal->pitch_joint_index] + (target_ray[0] - eye_ray[0]);

        for (int i = 0; i < casp->number_of_dofs; i++) {
            // if joint angle is masked and someone is controlling that joint angle, update it
            if (((casp->joint_angle_mask >> i) & 0x1) == 1 && q[casp->joint_names[i]] != nullptr) {
                casp->q[i] = *q[casp->joint_names[i]];
                casp->qd[i] = *qd[casp->joint_names[i]];
            }
        }

        casp->update(timestep);
        casp->updateController();

        if (goal->sendToRealHardware) {
            sendToRealHardware(casp);
        }

        error = (target_ray - eye_ray).norm();

        feedback.error = error;
        if ((ros::Time::now() - last_feedback_time).toSec() > 1) {
            last_feedback_time = ros::Time::now();
            // publish the feedback
            lookAt_as->publishFeedback(feedback);
        }

        if (lookAt_as->isPreemptRequested() || !ros::ok()) {
            ROS_INFO("LookAt: Preempted");
            // set the action state to preempted
            lookAt_as->setPreempted();
            success = false;
            break;
        }

        if ((ros::Time::now() - start_time).toSec() > goal->timeout) {
            ROS_ERROR("Lookat timeout %d", goal->timeout);
            success = false;
        }
    }
    // publish the feedback
    lookAt_as->publishFeedback(feedback);
    if (error < goal->tolerance && success) {
        ROS_INFO("MoveEndEffector: Succeeded");
        lookAt_as->setSucceeded(result, "done");
    } else {
        ROS_INFO("MoveEndEffector: FAILED");
        lookAt_as->setAborted(result, "failed");
    }


//
//    ROS_INFO_STREAM_THROTTLE(1, "euler " << target_ray[2]-eye_ray[2] << " " << target_ray[0]-eye_ray[0]);
}

void Roboy::moveEndEffector(const roboy_communication_control::MoveEndEffectorGoalConstPtr &goal) {
    roboy_communication_control::MoveEndEffectorFeedback feedback;
    roboy_communication_control::MoveEndEffectorResult result;
    bool success = true, timeout = false;

    double error = 10000;
    ros::Time last_feedback_time = ros::Time::now(), start_time = ros::Time::now();
    bool ik_solution_available = false;

    CASPRptr casp = casprByName[goal->endEffector];
    if (casp == nullptr) {
        ROS_WARN_STREAM("MoveEndEffector: FAILED endeffector " << goal->endEffector << " does not exist");
        moveEndEffector_as->setAborted(result, "endeffector " + goal->endEffector + " does not exist");
        return;
    }

    if (goal->sendToRealHardware) {
        setControlMode(casp);
    }

    Vector3d target_position;
    switch (goal->type) {
        case 0: {
            target_position[0] = goal->pose.position.x;
            target_position[1] = goal->pose.position.y;
            target_position[2] = goal->pose.position.z;
            break;
        }
        case 1: {
            tf::StampedTransform trans;
            try {
                if (listener.waitForTransform("world", goal->target_frame.c_str(),
                                              ros::Time(0), ros::Duration(0.0001))) {
                    listener.lookupTransform("world", goal->target_frame.c_str(),
                                             ros::Time(0), trans);
                    target_position[0] = trans.getOrigin().x() + goal->pose.position.x;
                    target_position[1] = trans.getOrigin().y() + goal->pose.position.y;
                    target_position[2] = trans.getOrigin().z() + goal->pose.position.z;
                } else {
                    ROS_ERROR("target frame %s is not available", goal->target_frame.c_str());
                    success = false;
                }
            }
            catch (tf::LookupException ex) {
                ROS_WARN("%s", ex.what());
                success = false;
            }
            break;
        }
        case 2: {
            if (goal->q_target.size() != casp->number_of_dofs) {
                ROS_ERROR("number of requested dofs does not match endeffector dofs");
                success = false;
            } else {
                for (int i = 0; i < casp->number_of_dofs; i++) {
                    casp->q_target[i] = goal->q_target[i];
                }
            }
            break;
        }
        default:
            success = false;
    }

    nh.getParam("controller", casp->controller);
    double timestep = 0.00001;
    switch (casp->controller) {
        case 0:
            nh.getParam(casp->end_effektor_name + "/ForceControl/Kp", casp->Kp);
            nh.getParam(casp->end_effektor_name + "/ForceControl/Kd", casp->Kd);
            timestep = 0.001;
            break;
        case 1:
            nh.getParam(casp->end_effektor_name + "/TorqueControl/Kp", casp->Kp);
            nh.getParam(casp->end_effektor_name + "/TorqueControl/Kd", casp->Kd);
            timestep = 0.001;
            break;
        case 2:
            nh.getParam(casp->end_effektor_name + "/PositionControl/Kp", casp->Kp);
            nh.getParam(casp->end_effektor_name + "/PositionControl/Kd", casp->Kd);
            timestep = 0.00001;
            break;
    }

    roboy_communication_middleware::InverseKinematics srv;
    srv.request.pose.position.x = target_position[0];
    srv.request.pose.position.y = target_position[1];
    srv.request.pose.position.z = target_position[2];
    srv.request.pose.orientation.x = goal->pose.orientation.x;
    srv.request.pose.orientation.y = goal->pose.orientation.y;
    srv.request.pose.orientation.z = goal->pose.orientation.z;
    srv.request.pose.orientation.w = goal->pose.orientation.w;

    publishSphere(target_position,"world","ik_target",696969,COLOR(0,1,0,1),0.05,goal->timeout);

    while (error > goal->tolerance && success && !timeout) {
        if (moveEndEffector_as->isPreemptRequested() || !ros::ok()) {
            ROS_INFO("LookAt: Preempted");
            // set the action state to preempted
            moveEndEffector_as->setPreempted();
            timeout = true;
        }

        if (!ik_solution_available && (goal->type == 0 || goal->type == 1)) {
            if (casp->InverseKinematicsService(srv.request, srv.response)) {
                ik_solution_available = true;
                for (int i = 0; i < casp->number_of_dofs; i++) {
                    casp->q_target[i] = srv.response.angles[i];
                }
            } else {
                ik_solution_available = false;
            }
        }

        if (ik_solution_available || goal->type == 2) {
            for (int i = 0; i < casp->number_of_dofs; i++) {
                // if joint angle is masked and someone is controlling that joint angle, update it
                if (((casp->joint_angle_mask >> i) & 0x1) == 1 && q[casp->joint_names[i]] != nullptr) {
                    casp->q[i] = *q[casp->joint_names[i]];
                    casp->qd[i] = *qd[casp->joint_names[i]];
                }
            }

            casp->update(timestep);
            casp->updateController();

            error = (casp->q_target - casp->q).norm();

        }

        if (goal->sendToRealHardware) {
            sendToRealHardware(casp);
        }

        if ((ros::Time::now() - last_feedback_time).toSec() > 1) {
            last_feedback_time = ros::Time::now();
            // publish the feedback
            feedback.error = error;
            moveEndEffector_as->publishFeedback(feedback);
        }
        if ((ros::Time::now() - start_time).toSec() > goal->timeout) {
            ROS_ERROR("move endeffector timeout %d", goal->timeout);
            success = false;
        }
    }
    // publish the feedback
    moveEndEffector_as->publishFeedback(feedback);
    if (error < goal->tolerance && success && !timeout) {
        ROS_INFO("MoveEndEffector: Succeeded");
        moveEndEffector_as->setSucceeded(result, "done");
    } else {
        ROS_WARN("MoveEndEffector: FAILED");
        moveEndEffector_as->setAborted(result, "failed");
    }


}

bool Roboy::ResetService(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res) {
    for (auto casp:caspr) {
        casp->init();
    }
    return true;
}

void Roboy::setControlMode(CASPRptr casp) {
    int number_of_motors = 0;
    nh.getParam(casp->end_effektor_name + "/number_of_motors", number_of_motors);
    roboy_communication_middleware::MotorConfigService msg;
    msg.request.config.id = casp->id;
    for (int motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        msg.request.config.motors.push_back(motor);
        if (motor < number_of_motors) { // position control
            msg.request.config.control_mode.push_back(POSITION);
            msg.request.config.outputPosMax.push_back(1000);
            msg.request.config.outputNegMax.push_back(-1000);
            msg.request.config.spPosMax.push_back(1000000);
            msg.request.config.spNegMax.push_back(-1000000);
            msg.request.config.Kp.push_back(1);
            msg.request.config.Ki.push_back(0);
            msg.request.config.Kd.push_back(0);
            msg.request.config.forwardGain.push_back(0);
            msg.request.config.deadBand.push_back(0);
            msg.request.config.IntegralPosMax.push_back(0);
            msg.request.config.IntegralNegMax.push_back(0);
            msg.request.config.outputDivider.push_back(1);
            switch (casp->id) {
                case HEAD: {
                        switch (motor_type[HEAD][motor]) {
                            case MYOBRICK100N:
                                msg.request.config.setpoint.push_back(-myoBrick100NEncoderTicksPerMeter(casp->motor_pos[motor]));
                                break;
                            case MYOBRICK300N:
                                msg.request.config.setpoint.push_back(-myoBrick300NEncoderTicksPerMeter(casp->motor_pos[motor]));
                                break;
                        }
                    break;
                }
                case SHOULDER_LEFT: {
                    switch (motor_type[SHOULDER_LEFT][motor]) {
                        case MYOMUSCLE500N:
                            msg.request.config.setpoint.push_back(-myoMuscleEncoderTicksPerMeter(
                                    (casp->motor_pos[motor] - casp->displacement_real[motor] * 2.0))); //
                            break;
                        case MYOBRICK100N:
                            msg.request.config.setpoint.push_back(-myoBrick100NEncoderTicksPerMeter(casp->motor_pos[motor]));
                            //- casp->displacement_real[i] * 2.0));
                            break;
                        case MYOBRICK300N:
                            msg.request.config.setpoint.push_back(-myoBrick300NEncoderTicksPerMeter(casp->motor_pos[motor]));
                            //- casp->displacement_real[i] * 2.0));
                            break;

                    }
                    break;
                }
            }
        }else {
            msg.request.config.control_mode.push_back(DISPLACEMENT);
            msg.request.config.outputPosMax.push_back(1000);
            msg.request.config.outputNegMax.push_back(-1000);
            msg.request.config.spPosMax.push_back(500);
            msg.request.config.spNegMax.push_back(0);
            msg.request.config.Kp.push_back(30);
            msg.request.config.Ki.push_back(0);
            msg.request.config.Kd.push_back(0);
            msg.request.config.forwardGain.push_back(0);
            msg.request.config.deadBand.push_back(0);
            msg.request.config.IntegralPosMax.push_back(0);
            msg.request.config.IntegralNegMax.push_back(0);
            msg.request.config.outputDivider.push_back(100);
            msg.request.config.setpoint.push_back(0);
        }
    }
    if (motor_config_srv.call(msg)) {
        ROS_WARN("could not change motor config");
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "roboy", ros::init_options::NoRosout);

    Roboy robot;

    ROS_INFO("STARTING ROBOY MAIN LOOP...");

    robot.main_loop();

    ROS_INFO("TERMINATING...");

    gazebo::shutdown();

    return 0;
}
