/*
	IMUGL - IMU LIBRARY

    Copyright (C) 2016	Emanuele Luberto (emanuele.luberto@gmail.com)

	Author affiliation:
		Emanuele Luberto - Research Center “E.Piaggio”,School of Engineering,University of Pisa
							from 01-april-2016 to current


    IMUGL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
	any later version.

    IMUGL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FORz A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SHOG. If not, see <http://www.gnu.org/licenses/>.
*/

#include "IMUGL.h"

// ============================================================================ Constructor
IMUGL::IMUGL()
{
	/********************
	*                   *
	*    CODE VERSION   *
	*                   *
	********************/
	/*
		Release date: 20-apr-2016
		Version: vMajorVersion.MinorVersion.Patcsh
		Release Version: v0.3.20


	*/
	time_t rawtime;
	// struct tm * timeinfo;
	int majorVersion;
	int minorVersion;
	int patchVersion;
	std::stringstream ssVersion;

	time(&rawtime);
	// timeinfo = localtime(&rawtime);

	//tm_year (int)	--> years since 1900
	//tm_mon (int)	--> months since January (0-11)
	//tm_mday (int)	--> day of the month (1-31)
	// majorVersion = (1901 + timeinfo->tm_year) - 2016;
	// minorVersion = timeinfo->tm_mon;
	// patchVersion = timeinfo->tm_mday;

	majorVersion = 2;
	minorVersion = 0;
	patchVersion = 0;

	ssVersion.str("");
	ssVersion << "v" << majorVersion << "." << minorVersion << "." << patchVersion << "\n";

	std::cout << "\nWelcome\n\tIMUGL " << ssVersion.str() << "\n";


	//initialize variables
	sub_acc_ = n_.subscribe("/qb_class_imu/acc", 100, &IMUGL::callbackAcc, this);
	sub_gyro_ = n_.subscribe("/qb_class_imu/gyro", 100, &IMUGL::callbackGyro, this);

	acc_flag_ = false;
	gyro_flag_ = false;

	n_.param<int>("nIMU", nIMU_, 6);
	n_.param<double>("sampleFreq", sampleFreq_, 100);
	n_.param<double>("beta", beta_, 1);
	n_.param<double>("thGyro", thGyro_, 15);


	// pkg_path_ = ros::package::getPath("imu_gripper");
	flag_write_ = false;


	std::cout << "Class initialized" << std::endl;
}


// =============================================================================================
//                                                                                   Desctructor
// =============================================================================================
IMUGL::~IMUGL()
{
	//nothing to be done
	std::cout << "\n\n\nSHUTDOWN\n\n\n";
}




/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*                                        FUNCTIONS                                           */
/*                                                                                            */
/*                                                                                            */
/**********************************************************************************************/


// =============================================================================================
//                                                                                      InitPSoC
// =============================================================================================
void IMUGL::start()
{


	// XmlRpc::XmlRpcValue c;
	// n_.getParam("chains", c);


	// Read values from rosparam
	// std::vector<int> v;
	// for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = c.begin(); it != c.end(); ++it)
	// {
	// n_.param<std::vector<int>>("chains/" + (std::string)(it->first), v, std::vector<int>{0,0,0,0,0,0});
	// chains_[it->first] = v;
	// }


	// Map creation
	// std::cout << "Selected Chains\n";
	// for(std::map<std::string, std::vector<int>>::iterator it = chains_.begin(); it != chains_.end(); ++it)
	// {
	// 	// print chains
	//     std::cout << it->first;
	//    	for (int j=0; j< (int)  it->second.size(); j++)
	//    		std::cout << " " <<  it->second[j];
	// 	std::cout << "\n";

	// 	// initilize quaternion map
	// 	Eigen::MatrixXd Mtmp_q = Eigen::MatrixXd::Zero(it->second.size()-1,4);
	// 	Eigen::VectorXd Vtmp = Eigen::VectorXd::Ones(it->second.size()-1);
	// 	Mtmp_q.col(0) = Vtmp;
	// 	q_map_.emplace(it->first, Mtmp_q);
	// 	q_map_joints_.emplace(it->first, Mtmp_q);
	// 	q_map_off_.emplace(it->first, Mtmp_q);

	// 	//initialize angles map
	// 	Eigen::MatrixXd Mtmp_angle = Eigen::MatrixXd::Zero(it->second.size()-1,3);
	// 	angles_map_.emplace(it->first, Mtmp_angle);

	// }

	// // print  first quaterinion for each chain
	// std::cout << "First Quaternions \n";
	// for(std::map<std::string, Eigen::MatrixXd>::iterator it = q_map_.begin(); it != q_map_.end(); ++it)
	//     std::cout << it->first<<"\n"<<  it->second << std::endl;

	QL_vector = std::vector<Eigen::Vector4d>(5, Eigen::Vector4d(1.0, 0., 0., 0.));
	angles = std::vector<Eigen::Vector3d>(5, Eigen::Vector3d(0.0, 0., 0.));


	// getchar();

	// init class variables
	Acc_.resize(nIMU_, 3);
	Acc_Old_.resize(nIMU_, 3);
	Gyro_.resize(nIMU_, 3);
	Gyro_Old_.resize(nIMU_, 3);
	// Gyro_Bias_.resize(nIMU_,3);
	// Gyro_Bias_.setZero();

	Acc_Old_ = Acc_;
	// Gyro_Bias_ = Gyro_;

	waitBoard();

	// while(gyro_flag_)
}






// =============================================================================================
//                                                                                MadgwickFilter
// =============================================================================================
Eigen::Vector4d IMUGL::MadgwickFilter(int P, int N, Eigen::Vector4d qL)
{
	//come N vede P;  N è il mio d (wordl), P è il mio s (sensor)
	// ad esempio N=imu1, P=imu0, quindi come la IMU 1 vede la IMU 0

	// float recipNorm;
	// float qDot1, qDot2, qDot3, qDot4;
	float q1, q2 , q3 , q4;

	float dx, dy, dz;
	float sx, sy, sz;

	Eigen::Vector3d aP, aN, gPpartial, gNpartial;
	Eigen::Vector4d gP, gN, g;

	Eigen::Vector3d fa;
	Eigen::MatrixXd Ja(3, 4);
	Eigen::Vector4d  qdot;

	Eigen::Vector4d Napla;


	aP(0)  = Acc_(P, 0);
	aP(1)  = Acc_(P, 1);
	aP(2)  = Acc_(P, 2);

	aN(0)  = Acc_(N, 0);
	aN(1)  = Acc_(N, 1);
	aN(2)  = Acc_(N, 2);

	// if ((aP.norm() == 0) || (aN.norm() == 0.0))
	// 	return qL;

	aP = aP / aP.norm();
	aN = aN / aN.norm();


	gP(0)  = 0;
	gP(1)  = Gyro_(P, 0);
	gP(2)  = Gyro_(P, 1);
	gP(3)  = Gyro_(P, 2);

	gN(0)  = 0;
	gN(1)  = Gyro_(N, 0);
	gN(2)  = Gyro_(N, 1);
	gN(3)  = Gyro_(N, 2);

	gP = gP * (M_PI / 180.);
	gN = gN * (M_PI / 180.);


	// rotate the angular velocity
	g = QxQ(QxQ(qL, gP), ConjQ(qL)) - gN;


	q1 = qL(0);
	q2 = qL(1);
	q3 = qL(2);
	q4 = qL(3);

	//accelerometer
	dx = aN(0);
	dy = aN(1);
	dz = aN(2);

	sx = aP(0);
	sy = aP(1);
	sz = aP(2);

	fa(0) =  2 * dx * (0.5 - q3 * q3 - q4 * q4) + 2 * dy * (q1 * q4 + q2 * q3) + 2 * dz * (q2 * q4 - q1 * q3) - sx;
	fa(1) =  2 * dx * (q2 * q3 - q1 * q4) + 2 * dy * (0.5 - q2 * q2 - q4 * q4) + 2 * dz * (q1 * q2 + q3 * q4) - sy;
	fa(2) =  2 * dx * (q1 * q3 - q2 * q4) + 2 * dy * (q3 * q4 - q1 * q2) + 2 * dz * (0.5 - q2 * q2 - q3 * q3) - sz;

	// Compute the Jacobian
	Ja << 2 * dy*q4 - 2 * dz*q3,    2 * dy*q3 + 2 * dz*q4 ,        -4 * dx*q3 + 2 * dy*q2 - 2 * dz*q1,  -4 * dx*q4 + 2 * dy*q1 + 2 * dz*q2,
	-2 * dx*q4 + 2 * dz*q2,   2 * dx*q3 - 4 * dy*q2 + 2 * dz*q1, 2 * dx*q2 + 2 * dz*q4,           -2 * dx*q1 - 4 * dy*q4 + 2 * dz*q3,
	2 * dx*q3 - 2 * dy*q2,    2 * dx*q4 - 2 * dy*q1 - 4 * dz*q2, 2 * dx*q1 + 2 * dy*q4 - 4 * dz*q3,   2 * dx*q2 + 2 * dy*q3;


	// Compute the Napla
	Napla = Ja.transpose() * fa;

	qdot = 0.5 * QxQ( qL, g ) - ( beta_ * Napla );
	// std::cout << gP.transpose() << std::endl;

	qL = qL + qdot / 100;

	qL = qL / qL.norm();

	return qL;
}




// =============================================================================================
//                                                                                 InitialOffest
// =============================================================================================
void IMUGL::initialOffset()
{
	std::string s0;
	// j = 2 because of the filter needs two different orientation

	for (int j = 0; j < 2; j++)
	{
		for (int k = 0; k < _OFFSET_STEP_; k++)
		{
			// waitBoard();
			// std::cout<<"step init: "<< k+1 << "\r\n";
			// rate.sleep();
			ros::spinOnce();
			usleep(10000);

			// Filter the gyroscope
			for (int i = 0; i < nIMU_; i++)
			{
				if (std::abs(Gyro_(i, 0)) < thGyro_)  Gyro_(i, 0) = 0;
				if (std::abs(Gyro_(i, 1)) < thGyro_)  Gyro_(i, 1) = 0;
				if (std::abs(Gyro_(i, 2)) < thGyro_)  Gyro_(i, 2) = 0;
			}


			// Compute the MadgwickFilter for the each IMUchains in the map
			// for(std::map<std::string, std::vector<int>>::iterator it = chains_.begin(); it != chains_.end(); ++it)
			// {
			// 	for (uint ii=0; ii<it->second.size()-1; ii++ )
			// 		q_map_.at(it->first).row(ii) = MadgwickFilter(it->second[ii], it->second[ii+1], q_map_.at(it->first).row(ii));
			// }

			for (int j = 0; j < nIMU_ - 1; ++j)
			{
				// All_Q.block<4,1>(0,j) = QL_vector[j];
				QL_vector[j] = MadgwickFilter(j, nIMU_ - 1, QL_vector[j]);
				std::cout << "Q( " << j << "): " << QL_vector[j] << " " << std::endl;
			}

			// usleep();
			// Record old Gyro value
			// Gyro_Old_ = Gyro_;

			//
			// q_map_joints_ = q_map_;

		}

		// Compute angles from quaternions
		quat2anglesTOT();

		// Print IMUs_Angles offset
		printIMUangles();

		// if (j < 2)
		// {
		// 	printf("Change Hand Orientation and push ENTER \r\n");
		// 	getchar();
		// }

		// // Update quaternions offset
		// q_map_off_ = q_map_joints_;
	}
	// Open the file
	// s0 = pkg_path_ + "/measurements/" + "angles_offset.txt";
	// std::cout << s0 << std::endl;
	// fileAngles_Offset_ = fopen(s0.c_str(), "w");
	// // Write Offset Angles
	// Eigen::Vector3d tmp;
	// int j = 0;
	// for (std::map<std::string, std::vector<int>>::iterator it = chains_.begin(); it != chains_.end(); ++it) {
	// 	fprintf(fileAngles_Offset_, "%d\t", j);
	// 	for (uint i = 0; i < it->second.size() - 1; i++ )
	// 	{
	// 		tmp = angles_map_.at(it->first).row(i);
	// 		tmp *= 180 / M_PI;
	// 		fprintf(fileAngles_Offset_, "%.2f\t", tmp(0));
	// 		fprintf(fileAngles_Offset_, "%.2f\t", tmp(1));
	// 		fprintf(fileAngles_Offset_, "%.2f\t", tmp(2));
	// 	}
	// 	j++;
	// }
	// fprintf(fileAngles_Offset_, "\n");
	// // Close the File
	// fclose(fileAngles_Offset_);
}



// =============================================================================================
//                                                                                 computeAngles
// =============================================================================================
// void IMUGL::computeAngles()
// {
// 	waitBoard();

// 	// Filter the gyroscope
// 	for (int i = 0; i < nIMU_; i++)
// 	{
// 		if (std::abs(Gyro_(i, 0)) < thGyro_)  Gyro_(i, 0) = 0;
// 		if (std::abs(Gyro_(i, 1)) < thGyro_)  Gyro_(i, 1) = 0;
// 		if (std::abs(Gyro_(i, 2)) < thGyro_)  Gyro_(i, 2) = 0;
// 	}


// 	// Compute the MadgwickFilter for the each IMUchains in the map
// 	for (std::map<std::string, std::vector<int>>::iterator it = chains_.begin(); it != chains_.end(); ++it)
// 	{
// 		for (uint ii = 0; ii < it->second.size() - 1; ii++ )
// 			q_map_.at(it->first).row(ii) = MadgwickFilter(it->second[ii], it->second[ii + 1], q_map_.at(it->first).row(ii));
// 	}

// 	// record old data
// 	Acc_Old_ = Acc_;
// 	Gyro_Old_ = Gyro_;

// 	offsetCorrector();
// 	quat2anglesTOT();
// }



// =============================================================================================
//                                                                                printIMUangles
// =============================================================================================
void IMUGL::printIMUangles()
{
	// Eigen::Vector3d tmp;

	// for (std::map<std::string, std::vector<int>>::iterator it = chains_.begin(); it != chains_.end(); ++it)
	// {
	// 	std::cout << "chain \033[1;31m" <<  it->first  << "\033[0m" << std::endl;
	// 	for (uint i = 0; i < it->second.size() - 1; i++ )
	// 	{
	// 		tmp = angles_map_.at(it->first).row(i);
	// 		tmp *= 180 / M_PI;
	// 		printf("%.2f\t%.2f\t%.2f\n", tmp(0), tmp(1), tmp(2));
	// 	}

	// 	std::cout << "\n";
	// }


	for (int j = 0; j < nIMU_ ; ++j)
	{
		// All_Q.block<4,1>(0,j) = QL_vector[j];
		std::cout << "imu: " << j << "t" << angles[j].transpose() * 180 / M_PI << std::endl;
		// std::cout << "Q( " << j << "): " << QL_vector[j] << " " << std::endl;
	}
}



// =============================================================================================
//                                                                                   Quat2Angles
// =============================================================================================
Eigen::Vector3d IMUGL::quat2Angles(Eigen::Vector4d Q_in)
{
	Eigen::Vector3d Angles_out;
	// pitch singularity
	// Angles_out(0) = atan2(2*Q_in(1)*Q_in(2) - 2*Q_in(0)*Q_in(3), 2*Q_in(0)*Q_in(0) + 2*Q_in(1)*Q_in(1)-1); //YAW
	// Angles_out(1) = -asin(2*Q_in(1)*Q_in(3) + 2*Q_in(0)*Q_in(2)); //PITCH
	// Angles_out(2) = atan2(2*Q_in(2)*Q_in(3) - 2*Q_in(0)*Q_in(1), 2*Q_in(0)*Q_in(0) + 2*Q_in(3)*Q_in(3)-1); // ROLL

	// yaw singularity
	float w = Q_in(0);
	float x = Q_in(1);
	float y = Q_in(2);
	float z = Q_in(3);
	Angles_out(0) =  asin(2 * x * y + 2 * z * w); //YAW
	Angles_out(2) = - atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z); //PITCH
	Angles_out(1) = - atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z); // ROLL
	return Angles_out;
}



// =============================================================================================
//                                                                                Quat2anglesTOT
// =============================================================================================
void IMUGL::quat2anglesTOT()
{
	Eigen::Vector3d tmp;

	// COMPUTE ANGLES FROM q_map_JOINTS_
	// for (std::map<std::string, std::vector<int>>::iterator it = chains_.begin(); it != chains_.end(); ++it)
	// {
	// 	// compute angles for each chain
	// 	for (uint i = 0; i < it->second.size() - 1; i++ )
	// 	{
	// 		tmp = quat2Angles(q_map_joints_.at(it->first).row(i));
	// 		angles_map_.at(it->first).row(i) = tmp;
	// 	}
	// }

	for (int j = 0; j < nIMU_ - 1; ++j)
	{
		// All_Q.block<4,1>(0,j) = QL_vector[j];
		angles[j] = quat2Angles(QL_vector[j]);
		// std::cout << "Q( " << j << "): " << QL_vector[j] << " " << std::endl;
	}
}



// =============================================================================================
//                                                                               offsetCorrector
// =============================================================================================
// void IMUGL::offsetCorrector()
// {
// 	Eigen::Vector4d qoff, q, qjoint;
// 	for (std::map<std::string, std::vector<int>>::iterator it = chains_.begin(); it != chains_.end(); ++it)
// 	{
// 		for (uint i = 0; i <  it->second.size() - 1; i++ )
// 		{
// 			q = q_map_.at(it->first).row(i);
// 			qoff = q_map_off_.at(it->first).row(i);

// 			qjoint(0) = qoff(0) * q(0) - (-qoff(1) * q(1) - qoff(2) * q(2) - qoff(3) * q(3));
// 			qjoint(1) = qoff(0) * q(1) - qoff(1) * q(0) - qoff(2) * q(3) + qoff(3) * q(2);
// 			qjoint(2) = qoff(0) * q(2) - qoff(2) * q(0) - qoff(3) * q(1) + qoff(1) * q(3);
// 			qjoint(3) = qoff(0) * q(3) - qoff(3) * q(0) - qoff(1) * q(2) + qoff(2) * q(1);

// 			q_map_joints_.at(it->first).row(i) = qjoint;
// 		}
// 	}
// }




// =============================================================================================
//                                                                                     waitBoard
// =============================================================================================
void IMUGL::waitBoard()
{
	// wait acc and gyro flag
	while (!acc_flag_ && !gyro_flag_)
		ros::spinOnce();
	acc_flag_ = gyro_flag_ = false;
}



// =============================================================================================
//                                                                                   callbackAcc
// =============================================================================================
void IMUGL::callbackAcc(qb_interface::inertialSensorArray imu)
{
	for (int i = 0; i < (int) imu.m.size(); i++)
	{
		Acc_(i, 0) = imu.m[i].x;
		Acc_(i, 1) = imu.m[i].y;
		Acc_(i, 2) = imu.m[i].z;
// 
		// std::cout << i  << " acc " << Acc_(i, 0) << "\t" << Acc_(i, 1) << "\t" << Acc_(i, 2) << std::endl;
		// if(i==(int) imu.m.size()-1)
		// std::cout << "\n";
	}

	acc_flag_ = true;
	// std::cout << "getting acc" << std::endl;
}



// =============================================================================================
//                                                                                  callbackGyro
// =============================================================================================
void IMUGL::callbackGyro(qb_interface::inertialSensorArray imu)
{
	for (int i = 0; i < (int) imu.m.size(); i++)
	{
		Gyro_(i, 0) = imu.m[i].x;
		Gyro_(i, 1) = imu.m[i].y;
		Gyro_(i, 2) = imu.m[i].z;

		// std::cout << i  <<" gyro " << Gyro_(i,0) << "\t" << Gyro_(i,1) << "\t" << Gyro_(i,2) << std::endl;
		// if(i==(int) imu.m.size()-1)
		// 	std::cout << "\n";
	}

	gyro_flag_ = true;
	// std::cout << "getting gyro" << std::endl;

}

// =============================================================================================
//                                                                                     openFiles
// =============================================================================================
// void IMUGL::openFiles(std::string s)
// {
// 	std::string s1, s2, s3, s4;

// 	flag_write_ = true;

// 	s1 = pkg_path_ + "/measurements/" + s + "_acc.txt";
// 	s2 = pkg_path_ + "/measurements/" + s + "_gyro.txt";
// 	s3 = pkg_path_ + "/measurements/" + s + "_angles.txt";
// 	fileAcc_    = fopen(s1.c_str(), "w");
// 	fileGyro_   = fopen(s2.c_str(), "w");
// 	fileAngles_ = fopen(s3.c_str(), "w");

// 	// write Parameters on file
// 	s4 = pkg_path_ + "/measurements/" + s + "_param.txt";
// 	fileParam_  = fopen(s4.c_str(), "w");
// 	fprintf(fileParam_, "Filter Parameters\n");
// 	fprintf(fileParam_, "nIMU: %d\n", nIMU_);
// 	fprintf(fileParam_, "beta: %.2f\n", beta_);
// 	fprintf(fileParam_, "sampleFreq: %.2f\n", sampleFreq_);
// 	fprintf(fileParam_, "thGyro: %.2f\n\n", thGyro_);
// 	fprintf(fileParam_, "chains:\n");

// 	for (std::map<std::string, std::vector<int>>::iterator it = chains_.begin(); it != chains_.end(); ++it)
// 	{
// 		fprintf(fileParam_, "%s:\t", it->first.c_str());
// 		for (int j = 0; j < (int)  it->second.size(); j++)
// 			fprintf(fileParam_, "%d ", it->second[j]);
// 		fprintf(fileParam_, "\n");
// 	}

// 	fprintf(fileParam_, "\nAngles Order\n");
// 	int j = 0;
// 	for (std::map<std::string, std::vector<int>>::iterator it = chains_.begin(); it != chains_.end(); ++it)
// 	{
// 		fprintf(fileParam_, "%d\t", j);
// 		fprintf(fileParam_, "%s\n", it->first.c_str());
// 		j++;
// 	}
// 	fprintf(fileParam_, "\n");
// 	fclose(fileParam_);
// }


// =============================================================================================
//                                                                                    writeFiles
// =============================================================================================
// void IMUGL::saveData()
// {
// 	if (flag_write_)
// 	{
// 		// save Acc
// 		for (int i = 0; i < Acc_.rows(); i++)
// 		{
// 			fprintf(fileAcc_, "%d\t", i);
// 			fprintf(fileAcc_, "%f\t", Acc_(i, 0));
// 			fprintf(fileAcc_, "%f\t", Acc_(i, 1));
// 			fprintf(fileAcc_, "%f\t", Acc_(i, 2));
// 		}
// 		fprintf(fileAcc_, "\n");

// 		// save Gyro
// 		for (int i = 0; i < Gyro_.rows(); i++)
// 		{
// 			fprintf(fileGyro_, "%d\t", i);
// 			fprintf(fileGyro_, "%f\t", Gyro_(i, 0));
// 			fprintf(fileGyro_, "%f\t", Gyro_(i, 1));
// 			fprintf(fileGyro_, "%f\t", Gyro_(i, 2));
// 		}
// 		fprintf(fileGyro_, "\n");

// 		// save Angles
// 		Eigen::Vector3d tmp;
// 		int j = 0;
// 		for (std::map<std::string, std::vector<int>>::iterator it = chains_.begin(); it != chains_.end(); ++it)
// 		{
// 			fprintf(fileAngles_, "%d\t", j);
// 			for (uint i = 0; i < it->second.size() - 1; i++ )
// 			{
// 				tmp = angles_map_.at(it->first).row(i);
// 				tmp *= 180 / M_PI;
// 				fprintf(fileAngles_, "%.2f\t", tmp(0));
// 				fprintf(fileAngles_, "%.2f\t", tmp(1));
// 				fprintf(fileAngles_, "%.2f\t", tmp(2));
// 			}
// 			j++;
// 		}
// 		fprintf(fileAngles_, "\n");
// 	}
// }


// =============================================================================================
//                                                                                    closeFiles
// =============================================================================================
// void IMUGL::closeFiles()
// {
// 	flag_write_ = false;

// 	fclose(fileAcc_);
// 	fclose(fileGyro_);
// 	fclose(fileAngles_);
// 	getchar();
// }





// =============================================================================================
//                                                                              Useful Functions
// =============================================================================================
Eigen::Matrix3d IMUGL::rotX(float x)
{
	Eigen::Matrix3d Rx;
	x = x * (M_PI / 180); // From deg to Rad
	Rx << 1 , 0, 0,
	0, cos(x), -sin(x),
	0, sin(x), cos(x);
	return Rx;
}
Eigen::Matrix3d IMUGL::rotY(float y)
{
	Eigen::Matrix3d Ry;
	y = y * (M_PI / 180); // From deg to Rad
	Ry << cos(y) , 0, sin(y),
	0,       1,   0,
	-sin(y), 0, cos(y);
	return Ry;
}

Eigen::Matrix3d IMUGL::rotZ(float z)
{
	Eigen::Matrix3d Rz;
	z = z * (M_PI / 180); // From deg to Rad
	Rz << cos(z), -sin(z), 0,
	sin(z), cos(z),  0,
	0,        0,     1;
	return Rz;
}

Eigen::Matrix3d IMUGL::rot(float x, float y, float z)
{
	Eigen::Matrix3d Rx, Ry, Rz, Rf;

	Rx = rotX(x);
	Ry = rotY(y);
	Rz = rotZ(z);

	Rf = Rz * Ry * Rx;
	return Rf;
}
//Skew Matrix
Eigen::Matrix3d IMUGL::skew(Eigen::Vector3d a)
{
	Eigen::Matrix3d q;
	q(0, 0) =  0;          q(0, 1) = -a(2);       q(0, 2) = a(1);
	q(1, 0) =  a(2);       q(1, 1) =  0;          q(1, 2) = -a(0);
	q(2, 0) = -a(1);       q(2, 1) =  a(0);       q(2, 2) = 0;
	return q;
}
//From Quaternion to Rotation Matrix
Eigen::Matrix3d IMUGL::Quat2Rot(Eigen::Vector4d Q_in)
{
	Eigen::Matrix3d RQ;
	float q0, q1, q2, q3;
	q0 = Q_in(0);
	q1 = Q_in(1);
	q2 = Q_in(2);
	q3 = Q_in(3);

	RQ <<         q0*q0 + q1*q1 - q2*q2 - q3*q3, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2),
	2 * (q1 * q2 + q0 * q3), q0*q0 + q2*q2 - q1*q1 - q3*q3, 2 * (q2 * q3 - q0 * q1),
	2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), q0*q0 + q3*q3 - q1*q1 - q2*q2;
	return RQ;
}

Eigen::Vector4d IMUGL::ConjQ(Eigen::Vector4d Q_in)
{
	Eigen::Vector4d Q_out;
	Q_out(0) =  Q_in(0);
	Q_out(1) = -Q_in(1);
	Q_out(2) = -Q_in(2);
	Q_out(3) = -Q_in(3);
	return Q_out;
}

Eigen::Vector4d IMUGL::QxQ(Eigen::Vector4d Q_1, Eigen::Vector4d Q_2)
{
	Eigen::Vector4d Q_out;
	Q_out(0) = Q_1(0) * Q_2(0) - (Q_1(1) * Q_2(1) + Q_1(2) * Q_2(2) + Q_1(3) * Q_2(3));
	Q_out(1) = Q_1(0) * Q_2(1) + Q_1(1) * Q_2(0) + (Q_1(2) * Q_2(3) - Q_1(3) * Q_2(2));
	Q_out(2) = Q_1(0) * Q_2(2) + Q_1(2) * Q_2(0) + (Q_1(3) * Q_2(1) - Q_1(1) * Q_2(3));
	Q_out(3) = Q_1(0) * Q_2(3) + Q_1(3) * Q_2(0) + (Q_1(1) * Q_2(2) - Q_1(2) * Q_2(1));
	return Q_out;
}

Eigen::Vector4d IMUGL::Rot2Quat(Eigen::Matrix3d R_in)
{
	Eigen::Vector4d q_out;
	q_out (0) = 0.5 * sqrt(R_in(0, 0) + R_in(1, 1) + R_in(2, 2) + 1 );
	q_out (1) = 0.5 * sgn(R_in(2, 1) - R_in(1, 2)) * sqrt(R_in(0, 0) - R_in(1, 1) - R_in(2, 2) + 1);
	q_out (2) = 0.5 * sgn(R_in(0, 2) - R_in(2, 0)) * sqrt(-R_in(0, 0) + R_in(1, 1) - R_in(2, 2) + 1);
	q_out (3) = 0.5 * sgn(R_in(1, 0) - R_in(0, 1)) * sqrt(-R_in(0, 0) - R_in(1, 1) + R_in(2, 2) + 1);

	return q_out;
}

Eigen::Vector3d IMUGL::Rot2Angle(Eigen::Matrix3d R_in)
{
	Eigen::Vector3d Angles_out;
	Angles_out(0) = atan2(R_in(1, 0), R_in(0, 0)); //YAW
	Angles_out(1) = atan2(-R_in(2, 0), sqrt(R_in(2, 1) * R_in(2, 1) + R_in(2, 2) * R_in(2, 2))); //PITCH
	Angles_out(2) = atan2(R_in(2, 1), R_in(2, 2)); // ROLL
	return Angles_out;
}