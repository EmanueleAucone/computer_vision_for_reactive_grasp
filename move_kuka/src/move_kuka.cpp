#include <move_kuka.h>

//------------------------------------------------------------------------------------------
//                                                                                  callBack
//------------------------------------------------------------------------------------------
//Aucone-Bagheri
void move_kuka::contactDetection(const std_msgs::String &msg)
{
	if (msg.data == "frontal_thumb")				
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "frontal_middle")		
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "frontal_ring")			
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "frontal_index")		 
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "frontal_little")		
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "side_index")			
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "side_thumb")		
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "side_little")	
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_thumb")		
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_middle")   
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_ring")		
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_index")		
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_little")		
	{
		flag_contact_   = true;
		finger_name_ = msg.data;
	}
	else
	{
		flag_contact_ = false;
		//NOTHING TO DO
	}
}

void move_kuka::callImuId(std_msgs::Int64 imu_id)
{
	if(imu_id.data >= 0)
	{
		std::cout << imu_id.data << std::endl;
		flag_contact_ = true;
	}
}

//------------------------------------------------------------------------------------------
//                                                                               constructor
//------------------------------------------------------------------------------------------
//Aucone-Bagheri
move_kuka::move_kuka()
{
	//Aucone-Bagheri
	sub_contact_detection_ = n_.subscribe("/contact_detection", 0, &move_kuka::contactDetection, this);
	sub_imu_id_ = n_.subscribe("/imu_id", 0, &move_kuka::callImuId, this);
	visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("vito_anchor", "/rviz_visual_markers"));
	visual_tools_->deleteAllMarkers();
	//check which controller you want to use; now it is teleoperation_controller
	pub_command_ = n_.advertise<lwr_controllers::CartesianImpedancePoint>("/right_arm/cartesian_impedance_controller/command", 0);
	pub_command_t = n_.advertise<geometry_msgs::Pose>("/right_arm/teleoperation_controller/command", 0);
	pub_home_ = n_.advertise<std_msgs::Float64MultiArray>("/right_arm/teleoperation_controller/home", 0);
	//Aucone-Bagheri
	pub_traj_ = n_.advertise<geometry_msgs::Pose>("/right_arm/one_task_inverse_kinematics/command", 0);
	pub_flag_ = n_.advertise<std_msgs::String>("/move_kuka/flag", 0);
	hand_publisher_ = n_.advertise<std_msgs::Float64>("/right_hand/hand_position", 0);
	// flag_which_finger_ = flag_grasp_ = false;

	trajectory_type = 1;
	n_.param<int>("/trajectory_type", trajectory_type, 0);
	n_.param<double>("/traj_time", traj_time, 2.0);
	n_.param<double>("/spin_rate", spin_rate, 300.0);
	n_.param<double>("/box_size", box_size, 0.15);

	n_.param<float>("/A_",A_,0.10);

	n_.param<float>("/rest_x_",rest_x_, 0.0);
	n_.param<float>("/rest_y_",rest_y_, 0.0);
	n_.param<float>("/rest_z_",rest_z_, 0.0);
	n_.param<float>("/rest_q_w_",rest_q_w_, 0.702);
	n_.param<float>("/rest_q_x_",rest_q_x_, 0.000);
	n_.param<float>("/rest_q_y_",rest_q_y_, 0.712);
	n_.param<float>("/rest_q_z_",rest_q_z_, 0.000);

	n_.param<float>("/offset_x_",offset_x_,-0.80);
	n_.param<float>("/offset_y_",offset_y_,0.40);
	n_.param<float>("/offset_z_",offset_z_,0.10);
	n_.param<float>("/q_w_",q_w_,0.702);
	n_.param<float>("/q_x_",q_x_,0.000);
	n_.param<float>("/q_y_",q_y_,0.712);
	n_.param<float>("/q_z_",q_z_,0.000);
	n_.param<bool>("/ground_",ground_,false);
	n_.param<double>("/off_high_",off_high_,0.0);
	n_.param<bool>("/bowl_",bowl_,false);
	n_.param<bool>("/sliding_",sliding_,false);

	joint_home.data.resize(7);
	n_.param<double>("/a1_home",joint_home.data[0],0.0);
	n_.param<double>("/a2_home",joint_home.data[1],0.0);
	n_.param<double>("/e1_home",joint_home.data[2],0.0);
	n_.param<double>("/a3_home",joint_home.data[3],0.0);
	n_.param<double>("/a4_home",joint_home.data[4],0.0);
	n_.param<double>("/a5_home",joint_home.data[5],0.0);
	n_.param<double>("/a6_home",joint_home.data[6],0.0);

	std::cout<<"JOINT HOME: [";
	for(int i =0; i < 7; i++)
		std::cout<<joint_home.data[i]<<"  ";
	    std::cout<<"]"<<std::endl;

	n_.param<float>("/delta_degree_",delta_degree_,0.30);

	n_.param<float>("/stiffness_t",stiffness_t,2000);
	n_.param<float>("/stiffness_r",stiffness_r,150);
	n_.param<float>("/damping",damping,0.7);
	n_.param<float>("/wrench",wrench,0.0);


	pkg_path_ = ros::package::getPath("move_kuka");

	// rest msg for publisher
	zero_stiffness_.x = stiffness_t;
	zero_stiffness_.y = stiffness_t;
	zero_stiffness_.z = stiffness_t;
	zero_stiffness_.rx = stiffness_r;
	zero_stiffness_.ry = stiffness_r;
	zero_stiffness_.rz = stiffness_r;
	msg_.k_FRI = zero_stiffness_;

	zero_stiffness_.x = damping;
	zero_stiffness_.y = damping;
	zero_stiffness_.z = damping;
	zero_stiffness_.rx = damping;
	zero_stiffness_.ry = damping;
	zero_stiffness_.rz = damping;
	msg_.d_FRI = zero_stiffness_;

	zero_wrench_.force.x = wrench;
	zero_wrench_.force.y = wrench;
	zero_wrench_.force.z = wrench;
	zero_wrench_.torque.x = wrench;
	zero_wrench_.torque.y = wrench;
	zero_wrench_.torque.z = wrench;
	msg_.f_FRI = zero_wrench_;

	visual_tools_->deleteAllMarkers();

	degree_ = 0;
	handClosure(0.1);
}

move_kuka::~move_kuka()
{

}


void move_kuka::homePosition()
{
	// get current transformation between "vito_anchor" and "right_palm_link" (equal to "right_arm_7_link")
	std::string link_from = "/vito_anchor";
	std::string link_to   = "/right_arm_7_link";

	tf::TransformListener listener;
	tf::StampedTransform t;

	ros::spinOnce();

	bool tf_ok=true;
	int tf_i=0;

	while(ros::ok())
	{
		try
		{
			tf_ok=true;
			listener.waitForTransform(link_from, link_to, ros::Time(0), ros::Duration(20.0)); //ros::Duration(2.5)
			// listener.lookupTransform(link_to, link_from,  ros::Time(0), t);
			listener.lookupTransform(link_from, link_to,  ros::Time(0), t);
		}
		catch (tf::TransformException ex)
		{
			tf_ok=false;
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
		if(tf_ok)
		{
			break;
		}
		else
		{
			std::cout<<"trial tf: #"<<tf_i++<<std::endl;
		}
	}

	Eigen::Quaterniond q_init, q_rest;
	Eigen::Vector3d v_init;
	tf::quaternionTFToEigen(t.getRotation(), q_init);
	tf::vectorTFToEigen(t.getOrigin(), v_init);

	Eigen::Affine3d pose_rest;

	pose_init = Eigen::Affine3d(q_init);
	pose_init.translation() = Eigen::Vector3d(v_init(0), v_init(1), v_init(2)); // translate x,y,z

	// quaternion from end-effector to camera
	q_middle = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())	//+180°	
		* Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY())	//+90°
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());	

	// define the rest pose and go from any initial position to the rest position
	q_rest = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())		
		* Eigen::AngleAxisd(1.48, Eigen::Vector3d::UnitY())	//75° < angle < 90°
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());	

	pose_rest = Eigen::Affine3d(q_rest*q_middle);
	pose_rest.translation() = Eigen::Vector3d(rest_x_ , rest_y_ , rest_z_); // translate x,y,z

	interpolation(pose_init, pose_rest, traj_time);

	std::cout << "\r\n\n\n\033[32m\033[1mPress to start.. \033[0m" << std::endl;
	getchar();

}

//------------------------------------------------------------------------------------------
//                                                                        getCurrentPosition
//------------------------------------------------------------------------------------------
//Aucone-Bagheri
Eigen::Affine3d move_kuka::getCurrentPosition()
{
	Eigen::Affine3d pose;

	// get current transformation between "vito_anchor" and "right_palm_link"
	std::string link_from = "/vito_anchor";
	std::string link_to   = "/right_arm_7_link";

	tf::TransformListener listener;
	tf::StampedTransform t;

	ros::spinOnce();

	bool tf_ok=true;
	int tf_i=0;

	while(ros::ok())
	{
		try
		{
			tf_ok=true;
			listener.waitForTransform(link_from, link_to, ros::Time(0), ros::Duration(20.0)); //ros::Duration(2.5)
			// listener.lookupTransform(link_to, link_from,  ros::Time(0), t);
			listener.lookupTransform(link_from, link_to,  ros::Time(0), t);
		}
		catch (tf::TransformException ex)
		{
			tf_ok=false;
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
		if(tf_ok)
		{
			break;
		}
		else
		{
			std::cout<<"trial tf: #"<<tf_i++<<std::endl;
		}
	}

	Eigen::Quaterniond q_init;
	Eigen::Vector3d v_init;
	tf::quaternionTFToEigen(t.getRotation(), q_init);
	tf::vectorTFToEigen(t.getOrigin(), v_init);

	pose = Eigen::Affine3d(q_init);
	pose.translation() = Eigen::Vector3d(v_init(0), v_init(1), v_init(2)); // translate x,y,z
	return pose;
}


//------------------------------------------------------------------------------------------
//                                                                                   manager
//------------------------------------------------------------------------------------------
//Aucone-Bagheri
void move_kuka::manager()
{
	ros::Rate r(spin_rate);

	std::ifstream input_file;
	input_file.open ("/home/emanuele/catkin_ws/src/move_kuka/src/my_input_file2.txt");

	if(input_file.is_open()){
		std::cout << "file correctly open" << std::endl;
	}
		
	while (input_file >> mypose1 >> mypose2 >> mypose3 >> mypose4 >> mypose5 >> mypose6 >> myangle >> mydimx >> mydimy >> mydimz >> myindex) 		{
		obj_posX     = mypose1;
		obj_posY     = mypose2;
		obj_posZ     = mypose3;
		obj_roll     = mypose4;
		obj_pitch    = mypose5;
		obj_yaw      = mypose6;
		angle_z	     = myangle;
		dimX 	     = mydimx;
		dimY 	     = mydimy;
		dimZ 	     = mydimz;	
		height_index = myindex;
	}

	switch(height_index)
	{
		case 0:
			height = dimX;
			break;
		case 1:
			height = dimY;
			break;
		case 2:
			height = dimZ;
			break;
		default:
			break;
	}

	handClosure(0.2);
	std::cout << "\r\n\n\n\033[32m\033[1mChoose if recognize object or start with a primitive! \033[0m" << std::endl;
	char c = getchar();
	sleep(1);
	switch(c)
	{
		case '0':
			std::cout << "\r\n\n\n\033[32m\033[1mObject recognition! \033[0m" << std::endl;
			g = obj_recognition;
			objectRecognition();
			break;
		case '1':
			std::cout << "\r\n\n\n\033[32m\033[1mLateral grasp! \033[0m" << std::endl;
			g = lateral;
			lateralGrasp();
			break;
		case '2':
			std::cout << "\r\n\n\n\033[32m\033[1mTop grasp! \033[0m" << std::endl;
			g = top;
			topGrasp();
			break;
		case '3':
			std::cout << "\r\n\n\n\033[32m\033[1mTop right/left grasp! \033[0m" << std::endl;
			g = topRL;
			topRightLeftGrasp();
			break;
		case '4':
			std::cout << "\r\n\n\n\033[32m\033[1mBottom grasp! \033[0m" << std::endl;
			g = bottom;
			bottomGrasp();
			break;
		case '5':
			std::cout << "\r\n\n\n\033[32m\033[1mPinch grasp! \033[0m" << std::endl;
			g = pinch;
			pinchGrasp();
			break;		
		case '6':
			std::cout << "\r\n\n\n\033[32m\033[1mPinch right/left grasp! \033[0m" << std::endl;
			g = pinchRL;
			pinchRightLeftGrasp();
			break;
		default:
			break;
				
	}
}

//------------------------------------------------------------------------------------------
//                                                                                 detection
//------------------------------------------------------------------------------------------
//Aucone-Bagheri
void move_kuka::detection()		//stop the robot arm and close the hand
{
	//std::cout<<flag_contact_<<std::endl;
	if(flag_contact_ && !flag_grasp_)
	{
		std::cout << "DETECTED!" << std::endl;

		std_msgs::String msg;
		std::stringstream ss;
		ss << "1";
		msg.data = ss.str();
		pub_flag_.publish(msg);		//stop the controller

		switch(g)
		{
			case lateral:
				flag_grasp_ = true;
				handClosure(0.8);		//close hand
				sleep(7);
				finishPosition(0.1);
				ss << "0";
				msg.data = ss.str();
				pub_flag_.publish(msg);		//restart the controller
			case top:
				flag_grasp_ = true;
				finishPosition(0.1);
				ss << "0";
				msg.data = ss.str();
				pub_flag_.publish(msg);			
			case topRL:
				flag_grasp_ = true;
				finishPosition(0.1);
				ss << "0";
				msg.data = ss.str();
				pub_flag_.publish(msg);
			case bottom:
				flag_grasp_ = true;
			case pinch:
				flag_grasp_ = true;
				finishPosition(0.1);
				ss << "0";
				msg.data = ss.str();
				pub_flag_.publish(msg);	
			case pinchRL:
				flag_grasp_ = true;
				finishPosition(0.1);
				ss << "0";
				msg.data = ss.str();
				pub_flag_.publish(msg);	
			case slide:
				flag_grasp_ = true;
			case flip:
				flag_grasp_ = true;
			default: 
				break;	
		}
		ros::shutdown();
	}
}

//------------------------------------------------------------------------------------------
//                                                                        object recognition
//------------------------------------------------------------------------------------------
//Aucone-Bagheri
void move_kuka::objectRecognition()		//object recognition from 3 different POV (the first acquisition is done yet)
{	
	std::cout << "\r\n\n\n\033[32m\033[1mAcquire the first cluster with segmentation2 and the click to move! \033[0m" << std::endl;
	while ((getchar()) != '\n');
	getchar();

	// movement for second acquition
	float curr_x_ = std::max((float)(obj_posX-dimZ-0.1), rest_x_);
	float curr_y_ = rest_y_ - dimZ/2 - 0.25;	
	
	q_middle = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())	//+180°	
		* Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY())	//+90°
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());	
	quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())		
		* Eigen::AngleAxisd(1.48, Eigen::Vector3d::UnitY())	// just a little bit less than 90°	
		* Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d::UnitZ());	//+45°

	pose_target = Eigen::Affine3d(quat*q_middle);
	pose_target.translation() = Eigen::Vector3d(curr_x_ , curr_y_, rest_z_); 

	interpolation(pose_, pose_target, traj_time);

	std::cout << "\r\n\n\n\033[32m\033[1mAcquire the second cluster with segmentation2 and the click to move! \033[0m" << std::endl;
	while ((getchar()) != '\n');
	getchar();

	// movement for third acquisition
	curr_y_ = rest_y_ + dimZ/2 + 0.25;

	quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())		
		* Eigen::AngleAxisd(1.48, Eigen::Vector3d::UnitY())	// just a little bit less than 90°	
		* Eigen::AngleAxisd(-M_PI/6, Eigen::Vector3d::UnitZ());	//-45°

	pose_target = Eigen::Affine3d(quat*q_middle);
	pose_target.translation() = Eigen::Vector3d(curr_x_ , curr_y_, rest_z_); 

	interpolation(pose_, pose_target, traj_time);

	std::cout << "\r\n\n\n\033[32m\033[1mAcquire the third cluster with segmentation2! \033[0m" << std::endl;
	while ((getchar()) != '\n');
	getchar();

	std::cout << "\r\n\n\n\033[32m\033[1mComing back in home position! \033[0m" << std::endl;
	homePosition();	
	// save the complete cluster
	// exit or return to manager for the primitive

}

//------------------------------------------------------------------------------------------
//                                                                                primitives
//------------------------------------------------------------------------------------------
//Aucone-Bagheri
void move_kuka::lateralGrasp()		//lateral grasp from right
{
	float offset = 0.15 + dimZ/2;
		
	//local rotations	
	quat = Eigen::AngleAxisd(3*M_PI/2, Eigen::Vector3d::UnitX())		//-90°
		* Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY())		//+90°
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());	
	
	pose_target = Eigen::Affine3d(quat);
	pose_target.translation() = Eigen::Vector3d(obj_posX, obj_posY-offset, obj_posZ); 

	//std::cout << "quaternione da dare al controllo " << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() << std::endl;	
	interpolation(pose_, pose_target, traj_time);

	std::cout << "\r\n\n\n\033[32m\033[1mClick to start grasping! \033[0m" << std::endl;
	while ((getchar()) != '\n');
	getchar();

	pose_target = Eigen::Affine3d(quat);
	pose_target.translation() = Eigen::Vector3d(obj_posX, obj_posY, obj_posZ); 
	
	interpolation(pose_, pose_target, traj_time);
}

//Aucone-Bagheri
void move_kuka::topGrasp()		//top grasp 
{
	float offset = 0.15 + height/2;
	
	quat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())		//180°
		* Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY())		//90°
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());	
	
	pose_target = Eigen::Affine3d(quat);
	pose_target.translation() = Eigen::Vector3d(obj_posX, obj_posY, obj_posZ+offset);
	
	interpolation(pose_, pose_target, traj_time);

	std::cout << "\r\n\n\n\033[32m\033[1mClick to start grasping! \033[0m" << std::endl;
	while ((getchar()) != '\n');
	getchar();

	pose_target = Eigen::Affine3d(quat);
	pose_target.translation() = Eigen::Vector3d(obj_posX, obj_posY, obj_posZ); 

	interpolation(pose_, pose_target, traj_time);
}

//Aucone-Bagheri
void move_kuka::topRightLeftGrasp()		//top right/left grasp depending on the object orientation
{
	float offset = 0.15 + height/2;
	float yaw_angle;
	std::cout << "angolo letto da file " << angle_z << std::endl;
	if(angle_z <= M_PI/2 || angle_z >= 3*M_PI/2)
		yaw_angle = angle_z + M_PI/2;	//object yaw angle + 90°	
	else 
		yaw_angle = angle_z - M_PI/2;	//object yaw angle - 90°
	
	quat = Eigen::AngleAxisd(3*M_PI/2, Eigen::Vector3d::UnitX())		//-90°
		* Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitY())		
		* Eigen::AngleAxisd(3*M_PI/2, Eigen::Vector3d::UnitZ());	//-90°

	pose_target = Eigen::Affine3d(quat);
	pose_target.translation() = Eigen::Vector3d(obj_posX, obj_posY, obj_posZ+offset); 
	
	interpolation(pose_, pose_target, traj_time);

	std::cout << "\r\n\n\n\033[32m\033[1mClick to start grasping! \033[0m" << std::endl;
	while ((getchar()) != '\n');
	getchar();

	pose_target = Eigen::Affine3d(quat);
	pose_target.translation() = Eigen::Vector3d(obj_posX, obj_posY, obj_posZ); 
	
	interpolation(pose_, pose_target, traj_time);
}

//Aucone-Bagheri
void move_kuka::bottomGrasp()
{
	float offset = 0.05 + dimZ/2;
	
	quat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())		//180°
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())		
		* Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());		//90°
	
	pose_target = Eigen::Affine3d(quat);
	pose_target.translation() = Eigen::Vector3d(obj_posX, obj_posY-offset, obj_posZ); 
	
	interpolation(pose_, pose_target, traj_time);
	std::cout << "\r\n\n\n\033[32m\033[1mClick to start grasping! \033[0m" << std::endl;
	while ((getchar()) != '\n');
	getchar();

	pose_target = Eigen::Affine3d(quat);
	pose_target.translation() = Eigen::Vector3d(obj_posX, obj_posY, obj_posZ); 

	interpolation(pose_, pose_target, traj_time);
}

//Aucone-Bagheri
void move_kuka::pinchGrasp()
{
	float offset = 0.15 + height/2;
	float offset2 = offset - 0.1;
	
	quat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())		//180°
		* Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY())		//90°
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());	
	
	pose_target = Eigen::Affine3d(quat);
	pose_target.translation() = Eigen::Vector3d(obj_posX, obj_posY, obj_posZ+offset);
	
	interpolation(pose_, pose_target, traj_time);

	std::cout << "\r\n\n\n\033[32m\033[1mClick to start grasping! \033[0m" << std::endl;
	while ((getchar()) != '\n');	
	getchar();

	pose_target = Eigen::Affine3d(quat);
	pose_target.translation() = Eigen::Vector3d(obj_posX, obj_posY, obj_posZ+offset2); 

	interpolation(pose_, pose_target, traj_time);
}

//Aucone-Bagheri
void move_kuka::pinchRightLeftGrasp()
{
	float offset = 0.15 + height/2;
	float offset2 = offset - 0.1;
	float yaw_angle;
	if(obj_yaw <= M_PI)
		yaw_angle = obj_yaw + 3*M_PI/2;	//object yaw angle - 90°	
	else 
		yaw_angle = obj_yaw + M_PI/2;	//object yaw angle + 90°
	
	quat = Eigen::AngleAxisd(3*M_PI/2, Eigen::Vector3d::UnitX())		//-90°
		* Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitY())		
		* Eigen::AngleAxisd(3*M_PI/2, Eigen::Vector3d::UnitZ());	//-90°

	pose_target = Eigen::Affine3d(quat);
	pose_target.translation() = Eigen::Vector3d(obj_posX, obj_posY, obj_posZ+offset); 
	
	interpolation(pose_, pose_target, traj_time);

	std::cout << "\r\n\n\n\033[32m\033[1mClick to start grasping! \033[0m" << std::endl;
	while ((getchar()) != '\n');
	getchar();

	pose_target = Eigen::Affine3d(quat);
	pose_target.translation() = Eigen::Vector3d(obj_posX, obj_posY, obj_posZ+offset2); 

	interpolation(pose_, pose_target, traj_time);
}

//Aucone-Bagheri
void move_kuka::slideGrasp()		//slide grasp
{
	
}

//Aucone-Bagheri
void move_kuka::flipGrasp()		//flip grasp
{
	
}

//------------------------------------------------------------------------------------------
//                                                                             interpolation
//------------------------------------------------------------------------------------------
int move_kuka::interpolation(Eigen::Affine3d x_start, Eigen::Affine3d x_finish, double traj_time_local)
{
	float th = 0.001;	//trade error
	float alpha = 0.015;
	float translation_error = 1;
	float c = 0; //for sleep

	Eigen::Affine3d x_next, x_now, x_prev;

	x_now  = x_start;
	x_prev = x_start;

	geometry_msgs::Pose x_finish_frame, x_now_frame, x_next_frame;

	// read quaternion from GeometryPoseMsg and convert them to Eigen::Quaterniond
	//passing from affine3d to geometry_msg
	tf::poseEigenToMsg(x_now, x_now_frame);
	tf::poseEigenToMsg(x_finish, x_finish_frame);

	Eigen::Quaterniond q_start(x_now_frame.orientation.w, x_now_frame.orientation.x, x_now_frame.orientation.y, x_now_frame.orientation.z);
	Eigen::Quaterniond q_finish(x_finish_frame.orientation.w, x_finish_frame.orientation.x, x_finish_frame.orientation.y, x_finish_frame.orientation.z);
	Eigen::Quaterniond q_err;
	Eigen::Vector3d euler;
	
	ros::Rate r(spin_rate);
	while (c <= 1 && ros::ok())
	{
		// update orientation
		double ctanh(std::tanh(4*c));
		if (c <= 1)
			q_err = q_start.slerp(ctanh, q_finish);

		x_next = Eigen::AngleAxisd(q_err);

		// update translation
		if (translation_error > th)
			x_next.translation() = x_now.translation() + ctanh * (x_finish.translation() - x_now.translation() ) ;

		// visual tool
		visual_tools_->publishAxis(x_next, 0.1, 0.01, "axis");
		visual_tools_->trigger();

		// set control
		tf::poseEigenToMsg(x_next, x_next_frame);
		
		//Aucone-Bagheri
		pose_ee.position.x    = x_next_frame.position.x;
		pose_ee.position.y    = x_next_frame.position.y;
		pose_ee.position.z    = x_next_frame.position.z;
		pose_ee.orientation.w = x_next_frame.orientation.w;
		pose_ee.orientation.x = x_next_frame.orientation.x;
		pose_ee.orientation.y = x_next_frame.orientation.y;
		pose_ee.orientation.z = x_next_frame.orientation.z;

		pub_traj_.publish(pose_ee);

		ros::spinOnce();

		c += (1.0 / spin_rate) / traj_time_local;
		//std::cout << flag_contact_ << std::endl;
		
		if(flag_stop_)
		{
			//std::cout<< "TOUCHED" <<std::endl;
			pose_ = x_next;
			return 0;
		}
		r.sleep();
	}
	
	// update global pose_ for next steps
	pose_ = x_next;
	return 1;
}

//------------------------------------------------------------------------------------------
//                                                                            finishPosition
//------------------------------------------------------------------------------------------
void move_kuka::finishPosition(float z)
{
	Eigen::Affine3d pose, pose_finish;
	
	pose_finish = pose_;
	pose_finish.translation() = pose_finish.translation() + Eigen::Vector3d(0, 0, z); // translate x,y,z

	interpolation(pose_, pose_finish, traj_time);

	std::cout << "\r\n\n\n\033[32m\033[1mGrasped Object\033[0m" << std::endl;
}

//------------------------------------------------------------------------------------------
//                                                                               handClosure
//------------------------------------------------------------------------------------------
void move_kuka::handClosure(float v)
{
	std_msgs::Float64 msg;
	msg.data = v;
	hand_publisher_.publish(msg);
	ros::spinOnce();
}
