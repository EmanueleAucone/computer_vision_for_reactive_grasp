/*  Copyright (C) 2015 Alessandro Tondo
 *  email: tondo.codes+ros <at> gmail.com
 *
 *  This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public 
 *  License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any 
 *  later version.
 *  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more 
 *  details.
 *  You should have received a copy of the GNU General Public License along with this program.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#include <XmlRpcValue.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>
#include <iostream>
#include <fstream>

std::ofstream temp_file_;

#include "reactive_grasping_detection_standalone.h"

ReactiveGraspingDetection::ReactiveGraspingDetection()
{
	// handles server private parameters (private names are protected from accidental name collisions)
	private_node_handle_ = new ros::NodeHandle("~");

	// new filter by ga-cds
	//         5.684e-05 z^3 + 0.0005333 z^2 + 0.0004544 z + 3.517e-05
	// G(z) = -------------------------------------------------------
	//         z^4 - 3.275 z^3 + 4.022 z^2 - 2.195 z + 0.4493

	std::vector<double> a = {1.000000000000000,  -3.729575279623796,   5.216149412392845,  -3.242336983880757,   0.755783741455728};
	std::vector<double> b = {0,   0.00000094599516119859,   0.00000984028346829946,   0.00000930436517043737,   0.00000079970021913990};

	private_node_handle_->param("verbose_mode", verbose_mode_, DEFAULT_VERBOSE_MODE);
	private_node_handle_->param("very_verbose_mode", very_verbose_mode_, DEFAULT_VERY_VERBOSE_MODE);
	private_node_handle_->param("only_detection", only_detection_, DEFAULT_ONLY_DETECTION);
	private_node_handle_->param("calibration", calibration_, DEFAULT_CALIBRATION);
	private_node_handle_->param("glove_topic_name", glove_topic_name_, std::string(DEFAULT_GLOVE_TOPIC_NAME));
	private_node_handle_->param("accel_map_topic_name", accel_map_topic_name_, std::string(DEFAULT_ACCEL_MAP_TOPIC_NAME));
	private_node_handle_->param("contact_detection_topic_name_", contact_detection_topic_name_, std::string(DEFAULT_CONTACT_DETECTION_TOPIC_NAME));
	private_node_handle_->param("imu_id_topic_name_", imu_id_topic_name_, std::string(DEFAULT_IMU_ID_TOPIC_NAME));
	private_node_handle_->param("topic_queue_length", topic_queue_length_, DEFAULT_TOPIC_QUEUE_LENGTH);
	private_node_handle_->param("filter_coeff_a", filter_coeff_a_, a);
	private_node_handle_->param("filter_coeff_b", filter_coeff_b_, b);
	private_node_handle_->param("num_imus", num_imus_, DEFAULT_NUM_IMUS);
	private_node_handle_->param("gravity_value", gravity_value_, DEFAULT_GRAVITY_VALUE);
	private_node_handle_->param("contact_threshold", contact_threshold_, (double)DEFAULT_CONTACT_THRESHOLD);
	private_node_handle_->param("false_positive_threshold", false_positive_threshold_, (double)DEFAULT_FALSE_POSITIVE_THRESHOLD);
	private_node_handle_->param("window_size", window_size_, DEFAULT_WINDOW_SIZE);
	private_node_handle_->param("tails_scale_factor", tails_scale_factor_, (double)DEFAULT_TAILS_SCALE_FACTOR);
	private_node_handle_->param("delay_threshold", delay_threshold_, (double)DEFAULT_DELAY_THRESHOLD);
	private_node_handle_->param("action_server", action_server_, std::string(DEFAULT_ACTION_SERVER));
	private_node_handle_->param("log_file_base_path", log_file_base_path_, std::string(DEFAULT_LOG_FILE_BASE_PATH));
	private_node_handle_->param("log_file_name_raw", log_file_name_raw_, std::string(DEFAULT_LOG_FILE_NAME_RAW));
	private_node_handle_->param("log_file_name_filt", log_file_name_filt_, std::string(DEFAULT_LOG_FILE_NAME_FILT));
	private_node_handle_->param("log_file_name_map", log_file_name_map_, std::string(DEFAULT_LOG_FILE_NAME_MAP));
	private_node_handle_->param("log_file_name_gyro", log_file_name_gyro_, std::string(DEFAULT_LOG_FILE_NAME_GYRO));
	private_node_handle_->param("demo", demo_, false);

	// retrieves the comparison acceleration evolutions and their relative target pose of the end-effector
	parseAccelerationMap();

	// at the startup the raw and filtered acceleration structures are filled with 0 values
	reactive_grasping::DataHistory init_values;
	init_values.x.resize(window_size_);
	init_values.y.resize(window_size_);
	init_values.z.resize(window_size_);
	init_values.abs_contribution.resize(window_size_);
	for (int i=0; i<num_imus_; i++)
	{
		accel_raw_.push_back(init_values);
		accel_filt_.push_back(init_values);
		gyro_raw_.push_back(init_values);
	}

	// stores in 'date_time_' the current time converted into a handful form (date/time format YYYYMMDD_HHMMSS)
	std::time_t raw_time;
	char buffer[16];
	std::time(&raw_time);
	std::strftime(buffer, 16, "%G%m%d_%H%M%S", std::localtime(&raw_time));
	date_time_ = buffer;

	// creates folder if it doesn't exist
	std::string command = "mkdir -p " + log_file_base_path_;
	system(command.c_str());
	// opens log files
	log_file_accel_raw_.open(log_file_base_path_ + date_time_ + "_" + log_file_name_raw_ + ".dat");
	log_file_accel_filt_.open(log_file_base_path_ + date_time_ + "_" + log_file_name_filt_ + ".dat");
	log_file_accel_map_.open(log_file_base_path_ + date_time_ + "_" + log_file_name_map_ + ".dat");
	log_file_gyro_raw_.open(log_file_base_path_ + date_time_ + "_" + log_file_name_gyro_ + ".dat");

	// statistics variables initialization
	start_time_ = ros::Time::now();
	num_data_processed_ = 0;
	num_contacts_detected_ = 0;

	// simulates a fake contact detection to let the robot reach the home pose
	contact_detected_ = true;
	false_positive_ = false;
	hand_closed_ = false;
	publish_primitive = node_handle_.advertise<geometry_msgs::Pose>("primitive", 1);
	generateAndSendGoal("first_homing","none");
	
	// publishers
	accel_map_publisher_ = node_handle_.advertise<std_msgs::Float64MultiArray>(accel_map_topic_name_, topic_queue_length_);
	contact_detection_ = node_handle_.advertise<std_msgs::String>(contact_detection_topic_name_, topic_queue_length_);
	imu_id_publisher_ = node_handle_.advertise<std_msgs::Int64>(imu_id_topic_name_, topic_queue_length_);

	// subscribes to the sensorized Glove topic
	glove_subscriber_ = node_handle_.subscribe(glove_topic_name_, topic_queue_length_, &ReactiveGraspingDetection::gloveMessageCallback, this);
	ROS_INFO_STREAM("[Detection] Node is retrieving Glove messages... (<ctrl+c> to terminate)");
}

ReactiveGraspingDetection::~ReactiveGraspingDetection()
{
	ros::Time elapsed_time = ros::Time::now();

	// prints statistics (can't use rosconsole macros due to ros::shutdown() callback)
	std::cout << "\n\n";
	std::cout << "[ INFO] [" << elapsed_time << "]: ReactiveGrasping Node statistics..." << '\n';
	std::cout << "       + Total elapsed time: " << elapsed_time - start_time_ << '\n';
	std::cout << "       + Total data processed: " << num_data_processed_ << '\n';
	std::cout << "       + Total data rate: " << (double)num_data_processed_ / (elapsed_time-start_time_).toSec() << '\n';
	std::cout << "       + Total contacts detected: " << num_contacts_detected_ << '\n';

	// closes log files if previously opened
	if (log_file_accel_raw_.is_open())
	{
		std::cout << "       + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_raw_ + ".dat\n";
		log_file_accel_raw_.close();
	}
	if (log_file_accel_filt_.is_open())
	{
		std::cout << "       + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_filt_ + ".dat\n";
		log_file_accel_filt_.close();
	}
	if (log_file_accel_map_.is_open())
	{
		std::cout << "       + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_map_ + ".dat\n";
		log_file_accel_map_.close();
	}
	if (log_file_gyro_raw_.is_open())
	{
		std::cout << "       + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_gyro_ + ".dat\n";
		log_file_gyro_raw_.close();
	}
	std::cout << std::endl;

	// delete motion_action_client_;
	delete private_node_handle_;
}

void ReactiveGraspingDetection::actionActiveCallback()
{
	if (verbose_mode_)
	{
		ROS_INFO_STREAM("[Detection::actionActiveCallback] Goal just went active.");
	}
	goal_activate_time_ = ros::Time::now();
}

void ReactiveGraspingDetection::appendToLogFile(std::ofstream *log_file, const std::vector<reactive_grasping::DataHistory> &data, const ros::Duration &acquisition_time)
{
	// stores all linear accelerations in the given log file (first term is acquisition time)
	*log_file << std::fixed << acquisition_time;
	for (auto const &imu : data)
	{
		*log_file << ";" << imu.x.back()
				 << ";" << imu.y.back()
				 << ";" << imu.z.back();
	}
	*log_file << std::endl;
}

int ReactiveGraspingDetection::checkOscillation(int imu_id)
{
	std::vector<double> values_x, values_y, values_z;
  	for (int j=0; j<12; j++)
	{
		values_x.push_back(accel_filt_.at(imu_id).x.at(window_size_/3 + j));
		values_y.push_back(accel_filt_.at(imu_id).y.at(window_size_/3 + j));
		values_z.push_back(accel_filt_.at(imu_id).z.at(window_size_/3 + j));
	}
	return -1;
}

int ReactiveGraspingDetection::detectContact()
{
	// finds which IMU has the biggest acceleration 'abs_contribution' from all the axes (at 'window_size_/3' samples)
	static bool first_print = true;

	std::vector<double> accel_abs_values;

	for (auto const &imu : accel_filt_)
		accel_abs_values.push_back(imu.abs_contribution.at(window_size_/3));

	std::vector<double>::const_iterator it_accel_max = std::max_element(accel_abs_values.begin(), accel_abs_values.end());

	// evaluates if the max(abs(xyz)) is greater than a specific threshold
	if (*it_accel_max > contact_threshold_)
	{
		//std::cout << "Threshold exceeded!" << std::endl;
		int imu_id = it_accel_max - accel_abs_values.begin();

		// checks for outliers (following 2 samples must be relevant and at least one opposite in sign to the first)
		if (accel_filt_.at(imu_id).abs_contribution.at(window_size_/3 + 1) > contact_threshold_
		&& accel_filt_.at(imu_id).abs_contribution.at(window_size_/3 + 2) > contact_threshold_
		&& accel_filt_.at(imu_id).abs_contribution.at(window_size_/3 + 3) > contact_threshold_
		&& accel_filt_.at(imu_id).abs_contribution.at(window_size_/3 + 4) > contact_threshold_
		&& accel_filt_.at(imu_id).abs_contribution.at(window_size_/3 + 5) > contact_threshold_)
		{
			if(imu_id >= 0)
			{
				if((accel_filt_.at(imu_id).abs_contribution.at(window_size_/3 + 1) < 200 && accel_filt_.at(imu_id).abs_contribution.at(window_size_/3 + 2) < 200))
		    			return -1;
		  		else
				{
		    			std::cout << *it_accel_max << std::endl;
				}
			}
			false_positive_ = false;  
			num_contacts_detected_++;
	
			if (!hand_closed_)
			{
				if(first_print)
				{
		 			 ROS_INFO_STREAM("[Detection::detectContact] IMU: " << imu_id << ", acc contact value:" << *it_accel_max);
		  			first_print = false;
				}
			}
			return imu_id;
		}
	}
	return -1;
}

void ReactiveGraspingDetection::eraseOldestSample(std::vector<reactive_grasping::DataHistory> &data)
{
	for (auto &imu : data)
	{
		imu.x.erase(imu.x.begin());
		imu.y.erase(imu.y.begin());
		imu.z.erase(imu.z.begin());
		imu.abs_contribution.erase(imu.abs_contribution.begin());
	}
}

void ReactiveGraspingDetection::eraseOldestSample()
{
	eraseOldestSample(accel_raw_);
	eraseOldestSample(accel_filt_);
	eraseOldestSample(gyro_raw_);
}

std::string ReactiveGraspingDetection::extractGraspPrimitive(const std::vector<double> &data)
{
	static bool first_print = true;

	std::map<std::string, double> max_xcorr_values_map;
	std::vector<double> xcorr_values;

	for (auto const &pair : accel_map_)
	{
		// evaluates the cross-correlation with the lag limited to 'window_size_/2'
		xcorr_values = xcorr(data, pair.second.at("samples"), window_size_/2);
		// stores the absolute peak of cross-correlation in a map containing also the relative approaching direction    
		std::for_each(xcorr_values.begin(), xcorr_values.end(), [](double &d){ d = std::abs(d); });
		auto it_max = std::max_element(xcorr_values.begin(), xcorr_values.end());
		max_xcorr_values_map.insert(std::make_pair(pair.first, *it_max));
	}
	auto it_max = std::max_element(max_xcorr_values_map.begin(), max_xcorr_values_map.end(),
		                 [](const std::map<std::string, double>::value_type x,
		                    const std::map<std::string, double>::value_type y){ return x.second < y.second; });

	if ((it_max->second) < 0.0 )
	{
		if(first_print)
		{
		ROS_INFO_STREAM("XCorr Value: " << it_max->second);
		first_print = false;
		}
		return "fake_finger";

	}
	else
	{
		if(first_print)
		{
		ROS_INFO_STREAM("XCorr Value: " << it_max->second);
		first_print = false;
		}
	return it_max->first;
	}
}

geometry_msgs::Pose ReactiveGraspingDetection::fillTargetPose(std::string approaching_direction)
{
	geometry_msgs::Pose target_pose;

	target_pose.position.x = accel_map_.at(approaching_direction).at("position").at(0);
	target_pose.position.y = accel_map_.at(approaching_direction).at("position").at(1);
	target_pose.position.z = accel_map_.at(approaching_direction).at("position").at(2);

	double roll = accel_map_.at(approaching_direction).at("orientation").at(0);
	double pitch = accel_map_.at(approaching_direction).at("orientation").at(1);
	double yaw = accel_map_.at(approaching_direction).at("orientation").at(2);
	target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(roll), angles::from_degrees(pitch), angles::from_degrees(yaw));
	return target_pose;
}

std::vector<double> ReactiveGraspingDetection::filter(std::vector<double> b, std::vector<double> a, std::vector<double> x, std::vector<double> y)
{
	if (x.size() < b.size() - 1)
	{
		ROS_WARN_STREAM("[Detection::filter] Measurements are shorter than parameters.");
		return y;
	}

	// builds first filtered values (to compute aj_yi for all aj in the following)
	if (y.size() < a.size() - 1)
	{
		if (y.empty())
			y.push_back((b.front() * x.front()) / a.front());
		
		std::vector<double>::const_iterator it_a = a.begin();
		std::vector<double>::const_iterator it_b = b.begin();
		while (y.size() < a.size() - 1)
		{
			double bj_xi = 0;
			for (std::vector<double>::const_iterator it_x = x.begin(); it_x != x.begin() + y.size(); it_x++)
				bj_xi += *it_x * *(it_b + y.size() - (it_x - x.begin()));
		
			double aj_yi = 0;
			for (std::vector<double>::const_iterator it_y = y.begin(); it_y != y.end(); it_y++)
				aj_yi += *it_y * *(it_a + y.size() - (it_y - y.begin()));
		
			y.push_back((bj_xi - aj_yi) / a.front());
		}
	}

	int num_elements_x_exceed_y = x.size() - y.size();
	if (num_elements_x_exceed_y < 0)
	{
		ROS_WARN_STREAM("[Detection::filter] Previous filtered data exceeds measurements.");
		return y;
	}
	if (num_elements_x_exceed_y == 0 && y.size() != 1)
	{
		ROS_WARN_STREAM("[Detection::filter] There are no new measurements.");
		return y;
	}

	for (std::vector<double>::const_iterator it_x = x.end() - num_elements_x_exceed_y; it_x != x.end(); it_x++)
	{
		double bj_xi = 0;
		for (std::vector<double>::const_iterator it_b = b.begin(); it_b != b.end(); it_b++)
			bj_xi += *it_b * *(it_x - (it_b - b.begin()));
		
		std::vector<double>::const_iterator it_y = y.begin() + (it_x - x.begin());
		double aj_yi = 0;
		if (a.size() > 1)
		{
			for (std::vector<double>::const_iterator it_a = a.begin() + 1; it_a != a.end(); it_a++)
				aj_yi += *it_a * *(it_y - (it_a - a.begin()));
		}
		y.push_back((bj_xi - aj_yi) / a.front());
	}
	return y;
}

void ReactiveGraspingDetection::filterAccelerations()
{
	std::vector<reactive_grasping::DataHistory>::const_iterator it_raw = accel_raw_.begin();
	for (auto &imu : accel_filt_)
	{
		imu.x = filter(filter_coeff_b_, filter_coeff_a_, (*it_raw).x, imu.x);
		imu.y = filter(filter_coeff_b_, filter_coeff_a_, (*it_raw).y, imu.y);
		imu.z = filter(filter_coeff_b_, filter_coeff_a_, (*it_raw).z, imu.z);
		// imu.abs_contribution.push_back(std::abs(imu.x.back()) + std::abs(imu.y.back()) + std::abs(imu.z.back()));
		imu.abs_contribution.push_back(std::sqrt(imu.x.back()*imu.x.back() + imu.y.back()*imu.y.back() + imu.z.back()*imu.z.back()));
		it_raw++;
	}
}

void ReactiveGraspingDetection::generateAndSendGoal(std::string status, std::string approaching_direction)
{
	geometry_msgs::Pose msg_primitive;
	if (status != "first_homing")
		msg_primitive = fillTargetPose(approaching_direction);

	publish_primitive.publish(msg_primitive);
	ros::spinOnce();
}

void ReactiveGraspingDetection::gloveMessageCallback(const reactive_grasping::GloveIMUArray::ConstPtr &msg)
{
	if (very_verbose_mode_)
		printGloveIMUArray(msg->data);

	num_data_processed_++;

	processData(msg->data, msg->acquisition_time);
}

std::vector<double> ReactiveGraspingDetection::normalizeAccelerations(std::vector<double> data, int imu_id)
{
	double max_value = *(std::max_element(data.begin(), data.end()));
	double min_value = *(std::min_element(data.begin(), data.end()));
	double max_abs_value = std::max(std::abs(max_value), std::abs(min_value));

	int sample_id=0;
	for (auto &sample : data)
	{
		sample /= max_abs_value;
		// scales only the samples not relative to the IMU 'imu_id'
		if (sample_id < imu_id*3*window_size_ || sample_id >= (imu_id+1)*3*window_size_)
			sample *= tails_scale_factor_;
		sample_id++;
	}

	return data;
}

void ReactiveGraspingDetection::parseAccelerationMap()
{
	ROS_INFO_STREAM("[Detection::parseAccelerationMap] Parsing object parameters from YAML configuration file...");

	std::string base_name = "approaching_directions";
	std::vector<std::string> approaching_directions;
	XmlRpc::XmlRpcValue list;
	if (!private_node_handle_->getParam("/" + base_name, list))
	{
		ROS_ERROR_STREAM("[Detection::parseAccelerationMap] Can't find '" + base_name + "' in YAML configuration file.");
		return;
	}
	for (auto it = list.begin(); it != list.end(); it++)
		approaching_directions.push_back(it->first);

	for (auto const &direction : approaching_directions)
	{
		std::string param_name;
		std::string field_name;
		std::vector<double> samples;
		std::vector<double> position;
		std::vector<double> orientation;
		std::map<std::string, std::vector<double>> map;

		field_name = "samples";
		param_name = "/"  + base_name + "/"  + direction + "/"  + field_name;
		if (!private_node_handle_->getParam(param_name, samples))
		{
			ROS_WARN_STREAM("[Detection::parseAccelerationMap] Can't find '" + param_name + "' in YAML configuration file.");
			continue;
		}
		map.insert(std::make_pair(field_name, samples));

		field_name = "position";
		param_name = "/"  + base_name + "/" + direction + "/"  + field_name;
		if (!private_node_handle_->getParam(param_name, position))
		{
			ROS_WARN_STREAM("[Detection::parseAccelerationMap] Can't find '" + param_name + "' in YAML configuration file.");
			continue;
		}
		map.insert(std::make_pair(field_name, position));

		field_name = "orientation";
		param_name = "/"  + base_name + "/"  + direction + "/"  + field_name;
		if (!private_node_handle_->getParam(param_name, orientation))
		{
			ROS_WARN_STREAM("[Detection::parseAccelerationMap] Can't find '" + param_name + "' in YAML configuration file.");
			continue;
		}
		map.insert(std::make_pair(field_name, orientation));

		accel_map_.insert(std::make_pair(direction, map));
	}
}

void ReactiveGraspingDetection::printGloveIMUArray(const std::vector<reactive_grasping::GloveIMU> &data)
{
	for (auto const &imu : data)
	{
		ROS_DEBUG_STREAM("[Detection::printGloveIMUArray] accelerometer[" << imu.id << "] = ["
			     << imu.linear_acceleration.x << "  "
			     << imu.linear_acceleration.y << "  "
			     << imu.linear_acceleration.z << "]");
	}
	for (auto const &imu : data)
	{
		ROS_DEBUG_STREAM("[Detection::printGloveIMUArray] gyroscope[" << imu.id << "] = ["
			     << imu.angular_velocity.x << "  "
			     << imu.angular_velocity.y << "  "
			     << imu.angular_velocity.z << "]");
	}
	if (!data.empty())
		ROS_DEBUG_STREAM("");  // adds one extra line to space the output
}

void ReactiveGraspingDetection::processData(std::vector<reactive_grasping::GloveIMU> data, ros::Duration acquisition_time)
{
	static bool first_print = true;

	eraseOldestSample();
	pushBackNewSample(data);
	filterAccelerations();
	updateLogFiles(acquisition_time);

	if (false_positive_ || only_detection_ || calibration_)
	{
		if (skip_samples_ > 0)
			skip_samples_--;
		else
		{
			contact_detected_ = false;
			false_positive_ = false;
			ROS_INFO_STREAM("[Detection::processData] Ready to detect other contacts.");
		}
	}

	int imu_id = detectContact();

	if (imu_id >= 0 && !false_positive_)
	{    
		// save imu id and publish it
		std_msgs::Int64 imu_id_msg;
		imu_id_msg.data = imu_id;
		imu_id_publisher_.publish(imu_id_msg);

		contact_detected_ = true;
		hand_closed_ = !hand_closed_;

		// concatenates the samples in a vector of length equals 3 x 'num_imus_' x 'window_size_'
		std::vector<double> accelerations_concatenated = toVector(accel_filt_);

		// normalizes the vector and scales its 'tails' down (emphasizes only peaks of imu with id = 'imu_id')
		std::vector<double> accelerations_normalized = normalizeAccelerations(accelerations_concatenated, imu_id);

		// evaluates the best grasp primitive throught cross-correlation with the data set
		std::string approaching_direction = extractGraspPrimitive(accelerations_normalized);

		if(first_print)
		{
			ROS_INFO_STREAM("[Detection::processData] Contact Detected on IMU " << imu_id);
			ROS_INFO_STREAM("[Detection::processData] Cross-Correlation Detection on '" << approaching_direction << "'");
			first_print = false;
		}

		// save imu direction and publish it
		std_msgs::String contact_detection_msg;
		contact_detection_msg.data = approaching_direction;
		std::cout << "I'M HERE!" << std::endl;
		contact_detection_.publish(contact_detection_msg);
		std::cout << "PUBLISHED!" << std::endl;

		if (approaching_direction == "fake_finger")
			only_detection_ = 1;
		if (!only_detection_ && !calibration_)
			generateAndSendGoal("sending target pose", approaching_direction);
		only_detection_ = 0;

		// save imu accel map and publish it
		std_msgs::Float64MultiArray accelerations_map;
		accelerations_map.data = accelerations_normalized;
		accel_map_publisher_.publish(accelerations_map);

		if (calibration_)
		{
			std::string answer = "";
			ROS_INFO_STREAM("[Detection::processData] Calibration: insert the current approaching direction ('n' to skip)");
			std::cin >> answer;
			if (answer == "n" || answer == "no" || answer == "N" || answer == "NO")
				ROS_WARN_STREAM("[Detection::processData] Calibration: acceleration map rejected (not marked in the log).");
			else
			{
				// highlight the acceleration maps to be stored in 'comparison_dataset'
				log_file_accel_map_ << "*** To be copied in the current 'comparison_dataset' ***\n"
					      << "*** Real contact on: " << answer << " ***\n";
			}
			std::cin.clear();
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}

		log_file_accel_map_ << std::fixed << "Acquisition time: " << acquisition_time
		                        << "\nContact detected on IMU: " << imu_id
		                        << "\nGrasp primitive extracted: " << approaching_direction
		                        << "\n" << toString(accelerations_normalized) << "\n" << std::endl;

		// avoid following false contact detection
		if (only_detection_ || calibration_)
			skip_samples_ = window_size_;
	}
}

void ReactiveGraspingDetection::pushBackNewSample(const std::vector<reactive_grasping::GloveIMU> &new_sample)
{
	std::vector<reactive_grasping::GloveIMU>::const_iterator it_new = new_sample.begin();
	for (auto &imu : accel_raw_)
	{
		imu.x.push_back((*it_new).linear_acceleration.x);
		imu.y.push_back((*it_new).linear_acceleration.y);
		imu.z.push_back((*it_new).linear_acceleration.z);
		imu.abs_contribution.push_back(std::abs(imu.x.back()) + std::abs(imu.y.back()) + std::abs(imu.z.back()));

		it_new++;
	}

	it_new = new_sample.begin();
	for (auto &imu : gyro_raw_)
	{
		imu.x.push_back((*it_new).angular_velocity.x);
		imu.y.push_back((*it_new).angular_velocity.y);
		imu.z.push_back((*it_new).angular_velocity.z);
		imu.abs_contribution.push_back(std::abs(imu.x.back()) + std::abs(imu.y.back()) + std::abs(imu.z.back()));

		it_new++;
	}
}

std::string ReactiveGraspingDetection::toString(const std::vector<double> &data)
{
	std::stringstream ss;
	std::copy(data.begin(), data.end(), std::ostream_iterator<double>(ss, " "));
	return ss.str();
}

std::vector<double> ReactiveGraspingDetection::toVector(const std::vector<reactive_grasping::DataHistory> &data)
{
	std::vector<double> data_vector;
	for (auto const &imu : data)
	{
		std::copy_n(imu.x.begin(), imu.x.size(), std::back_inserter(data_vector));
		std::copy_n(imu.y.begin(), imu.y.size(), std::back_inserter(data_vector));
		std::copy_n(imu.z.begin(), imu.z.size(), std::back_inserter(data_vector));
	}
	return data_vector;
}

void ReactiveGraspingDetection::updateLogFiles(const ros::Duration &acquisition_time)
{
	appendToLogFile(&log_file_accel_raw_, accel_raw_, acquisition_time);
	appendToLogFile(&log_file_accel_filt_, accel_filt_, acquisition_time);
	appendToLogFile(&log_file_gyro_raw_, gyro_raw_, acquisition_time);
}

std::vector<double> ReactiveGraspingDetection::xcorr(std::vector<double> x, std::vector<double> y, int max_lag)
{
	std::vector<double> xcorr_values;
	int num_samples = std::max(x.size(), y.size());

	if (x.size() > y.size())
		y.resize(x.size());

	else if (x.size() < y.size())
		x.resize(y.size());

	if (max_lag > num_samples)
		max_lag = num_samples;


	double mean_x = std::accumulate(x.begin(), x.end(), 0.0) / num_samples;
	double mean_y = std::accumulate(y.begin(), y.end(), 0.0) / num_samples;

	// calculates the denominator
	double sx = 0;
	double sy = 0;
	for (int i=0; i<num_samples; i++)
	{
		sx += (x.at(i) - mean_x) * (x.at(i) - mean_x);
		sy += (y.at(i) - mean_y) * (y.at(i) - mean_y);
	}
	double denominator = std::sqrt(sx*sy);

	// calculates the Cross-Correlation series
	for (int lag=-max_lag; lag<max_lag; lag++)
	{
		double sxy = 0;
		for (int i=0; i<num_samples; i++)
		{
			int j = i + lag;
			if (j < 0 || j >= num_samples)
	 			sxy += (x.at(i) - mean_x) * (-mean_y);
			else
				sxy += (x.at(i) - mean_x) * (y.at(j) - mean_y);
		}
		// stores the new value computed for the current lag in the vector
		xcorr_values.push_back(sxy / denominator);
	}
	return xcorr_values;
}
