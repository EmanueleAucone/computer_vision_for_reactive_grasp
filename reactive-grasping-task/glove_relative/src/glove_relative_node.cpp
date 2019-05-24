#include <ros/ros.h>
#include <reactive_grasping/GloveIMU.h>
#include <reactive_grasping/GloveIMUArray.h>
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>
#include <iostream>
#include <fstream>

ros::Publisher pub_glove, pub_glove_local;
ros::Subscriber sub_acc, sub_gyro;
std::vector<double> acc, gyro, acc_relative, gyro_relative;
std::vector<int> imu_ids;
int num_imu;
double sampleFreq_  =  100;
double beta_        =  3.0;

std::ofstream accFile_;
//Aucone-Bagheri
std::ofstream raw_acc_file;
std::ofstream acc_local_file;
std::ofstream acc_temp_file;
std::ofstream imu_linear_acceleration_file;
std::ofstream imu_file;

std::vector<Eigen::Vector4d> QL_vector;


/*This package publish the relative acceleration of num_imu-1 imu's relative to the num_imu_imo*/

Eigen::Vector4d ConjQ(Eigen::Vector4d Q_in)
{
	Eigen::Vector4d Q_out;
	Q_out(0) =  Q_in(0);
	Q_out(1) = -Q_in(1);
	Q_out(2) = -Q_in(2);
	Q_out(3) = -Q_in(3);
	return Q_out;
}

Eigen::Vector4d QxQ(Eigen::Vector4d Q_1, Eigen::Vector4d Q_2)
{
	Eigen::Vector4d Q_out;
	Q_out(0) = Q_1(0) * Q_2(0) - (Q_1(1) * Q_2(1) + Q_1(2) * Q_2(2) + Q_1(3) * Q_2(3));
	Q_out(1) = Q_1(0) * Q_2(1) + Q_1(1) * Q_2(0) + (Q_1(2) * Q_2(3) - Q_1(3) * Q_2(2));
	Q_out(2) = Q_1(0) * Q_2(2) + Q_1(2) * Q_2(0) + (Q_1(3) * Q_2(1) - Q_1(1) * Q_2(3));
	Q_out(3) = Q_1(0) * Q_2(3) + Q_1(3) * Q_2(0) + (Q_1(1) * Q_2(2) - Q_1(2) * Q_2(1));
	return Q_out;
}


Eigen::Matrix3d quat2rot(Eigen::Vector4d Q)
{

	double x = Q(1), y = Q(2), z = Q(3), w = Q(0);
	double x2, y2, z2, w2;
	x2 = x * x;  y2 = y * y; z2 = z * z;  w2 = w * w;

	Eigen::Matrix3d R;
	R << w2 + x2 - y2 - z2, 2 * x*y - 2 * w*z, 2 * x*z + 2 * w*y,
	2 * x*y + 2 * w*z, w2 - x2 + y2 - z2, 2 * y*z - 2 * w*x,
	2 * x*z - 2 * w*y, 2 * y*z + 2 * w*x, w2 - x2 - y2 + z2;

	return R;
}

Eigen::Vector4d MadgwickFilter(int P, int N, Eigen::Vector4d qL)
{
	//come N vede P;  N è il mio d (wordl), P è il mio s (sensor)
	// ad esempio N=imu1, P=imu0, quindi come la IMU 1 vede la IMU 0

	// float recipNorm;
	// float qDot1, qDot2, qDot3, qDot4;
	float q1, q2 , q3 , q4;

	float dx, dy, dz;
	float sx, sy, sz;
	int gyro_threshold = 20;
	Eigen::Vector3d aP, aN, gPpartial, gNpartial;
	Eigen::Vector4d gP, gN, g;

	Eigen::Vector3d fa;
	Eigen::MatrixXd Ja(3, 4);
	Eigen::Vector4d  qdot;

	Eigen::Vector4d Napla;

	aP(0)  = acc[P * 3];
	aP(1)  = acc[P * 3 + 1];
	aP(2)  = acc[P * 3 + 2];

	aN(0)  = acc[N * 3];
	aN(1)  = acc[N * 3 + 1];
	aN(2)  = acc[N * 3 + 2];

	//if ((aP.norm() == 0) || (aN.norm() == 0.0))
	//	return qL;
	

	if ((aP.norm() < 0.9) || (aN.norm() < 0.9))
		return qL;	

	if ((aP.norm() > 1.1) || (aN.norm() > 1.1))
		return qL;	



	aP = aP / aP.norm();
	aN = aN / aN.norm();

	gP(0)  = 0;
	gP(1)  = gyro[P * 3];
	if (fabs(gP(1)) < gyro_threshold)
		gP(1) = 0;
	gP(2)  = gyro[P * 3 + 1];
	if (fabs(gP(2)) < gyro_threshold)
		gP(2) = 0;
	gP(3)  = gyro[P * 3 + 2];
	if (fabs(gP(3)) < gyro_threshold)
		gP(3) = 0;

	gN(0)  = 0;
	gN(1)  = gyro[N * 3];
	if (fabs(gN(1)) < gyro_threshold)
		gN(1) = 0;
	gN(2)  = gyro[N * 3 + 1];
	if (fabs(gN(2)) < gyro_threshold)
		gN(2) = 0;
	gN(3)  = gyro[N * 3 + 2];
	if (fabs(gN(3)) < gyro_threshold)
		gN(3) = 0;

	gP = gP * (M_PI / 180);
	gN = gN * (M_PI / 180);



	// rotate the angular velocity
	g = QxQ(QxQ(qL, gP), ConjQ(qL)) - gN;

	//std::cout << "P " << P << "  ax " << aP(0) << "  ay " << aP(1) << "  az " << aP(2) << std::endl;
	//std::cout << "N " << N << "  ax " << aN(0) << "  ay " << aN(1) << "  az " << aN(2) << std::endl;
	//std::cout << "g" << P << N << " : " << g(0) << " , " << g(1) << " , " << g(2) << " , " << g(3) << std::endl;
	//std::cout << std::endl;

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

	qL = qL + qdot / sampleFreq_;

	qL = qL / qL.norm();

	return qL;
}


void cb_acc(const qb_interface::inertialSensorArray::ConstPtr& msg )
{
	int j = ((num_imu-1)*3 - 1);
	int d = 0;
	for (int i = 0; i < msg->m.size(); ++i)
	{
		if ( msg->m[i].id == 15)// filter imu's // Prima 15
			continue;
		if (msg->m[i].id == 16) { // Prima 16
			imu_ids[d] = msg->m[i].id;
			acc[num_imu*3 - 3] = msg->m[i].x;
			acc[num_imu*3 - 2] = msg->m[i].y;
			acc[num_imu*3 - 1] = msg->m[i].z;
		}
		else
		{
			imu_ids[d] = msg->m[i].id;
			acc[j - 2] = msg->m[i].x;	
			acc[j - 1] = msg->m[i].y;
			acc[j] = msg->m[i].z;
			j -= 3;
		}
		++d;
	}
}

void cb_gyro(const qb_interface::inertialSensorArray::ConstPtr& msg )
{
	int j = ((num_imu-1)*3 - 1);
	int d = 0;
	//for (int i = 0; i < num_imu; ++i)
	for (int i = 0; i < msg->m.size(); ++i)	
	{
		if ( msg->m[i].id == 15)// filter imu's
			continue;
		if (msg->m[i].id == 16) {
			imu_ids[d] = msg->m[i].id; // Aggiunta Dopo
			gyro[num_imu*3 - 3] = msg->m[i].x;
			gyro[num_imu*3 - 2] = msg->m[i].y;
			gyro[num_imu*3 - 1] = msg->m[i].z;
		}
		else
		{
			imu_ids[d] = msg->m[i].id; // Aggiunta Dopo
			gyro[j - 2] = msg->m[i].x;	
			gyro[j - 1] = msg->m[i].y;
			gyro[j] = msg->m[i].z;
			j -= 3;
		}
		d++; // Aggiunta Dopo
	}		
	

	j = 0;
	//for (int i = 0; i < num_imu; i++)
	//{
	//	if (std::abs(gyro[j]) < 15)  gyro[j] = 0;
	//	if (std::abs(gyro[j + 1]) < 15)  gyro[j+1] = 0;
	//	if (std::abs(gyro[j + 2] ) < 15)  gyro[j+2] = 0;
	//	j += 3;	
	//}
}

// // Modified EB 11/22/2017 - remove Magwick filter and use raw acceleration data
// float filter_accel(float new_value, uint8 idx, uint8 dir) {
// // This function implements a first order Butterworth filter to remove data generated by movement
// 	// new_value -> last value measured by the IMU
// 	// idx -> IMU for which the value was measured (position in the variable vect_acc)
// 		// NOTE: this is the position in the variable defined here. A number one to NUM_OF_IMU = 5 is expected
// 	// dir -> axis (0,1,2 corresponding to x,y or z)
// 	// NOTE: this function is not finished yet
// 	float A0,A1,B0,B1;
// 	A0 = 1;
// 	A1 = -0.98441412741609691;
// 	B0 = 0.0077929362919515527;
// 	B1 = 0.0077929362919515527;
	
	
//     float s1, s2, s3, s4;
   	
//    	int NUM_OF_IMU = 5;

//     static float vect_acc[NUM_OF_IMU][3];            // Last Sample (Filter input)
//     static float vect_acc_old[NUM_OF_IMU][3];         // Sample one step back
   
//     static float vect_acc_filt[NUM_OF_IMU][3];        // Filtered last sample (Filter Output)
//     static float vect_acc_old_filt[NUM_OF_IMU][3];    // Filter output one step back
//     vect_acc_old[idx][dir] = vect_acc[idx][dir];
   
//     vect_acc_old_filt[idx][dir] = vect_acc_filt[idx][dir];
   
//     vect_acc[idx][dir] = new_value;
   
//     s1 = ( ( B0 )  * ( vect_acc[idx][dir]));
//     s2 = ( ( B1 ) * ( vect_acc_old[idx][dir]));
//     s3 = ( ( A0 ) * ( vect_acc_filt[idx][dir]));
//     s4 = ( ( A1 ) * ( vect_acc_old_filt[idx][dir]));
 
//  	vect_acc_filt[idx][dir] = ( s1 + s2 - s3 - s4);
   
//     return (vect_acc_filt[idx][dir]);
// }
// // end EB


//SIMONE CIOTTI - START
// Modified EB 11/22/2017 - remove Magwick filter and use raw acceleration data
Eigen::VectorXd Raw_Acc_(15);
Eigen::VectorXd imu_bias_(15);

// This last data has gravity removed and a low pass filter to remove acceleration measurements generated by movement
// This is all though: no coordinate changes!
// end EB
//SIMONE CIOTTI - END

void publish(ros::Time t)
{
	int j = 0;
	reactive_grasping::GloveIMU imu, imu_local;
	reactive_grasping::GloveIMUArray imus;
	reactive_grasping::GloveIMUArray imus_local;
	double acc_base[3];

	//acc_base[0] = acc[(num_imu - 1)*3];
	//acc_base[1] = acc[(num_imu - 1)*3 + 1];
	//acc_base[2] = acc[(num_imu - 1)*3 + 2];
	// acc_base[0] = acc[(num_imu - 1)*3] - imu_bias_[(num_imu - 1)*3];
	// acc_base[1] = acc[(num_imu - 1)*3 + 1] - imu_bias_[(num_imu - 1)*3 + 1];
	// acc_base[2] = acc[(num_imu - 1)*3 + 2] - imu_bias_[(num_imu - 1)*3 + 2];

	for (int i = 0; i < num_imu - 1; i++)
	{

		Eigen::Vector3d acc_relative, acc_local, acc_temp;
		acc_local <<  acc[j],  acc[j + 1], acc[j + 2];
		//**** EB 11/22/2017 
		// Remove bias (i.e. gravity basically)

		//SIMONE CIOTTI - START
		int fattore_gaspare = 1;
		Raw_Acc_[3*i]     = fattore_gaspare*acc[j];
		Raw_Acc_[3*i + 1] = fattore_gaspare*acc[j+1];
		Raw_Acc_[3*i + 2] = fattore_gaspare*acc[j+2];
		//SIMONE CIOTTI - END

		acc_temp(0) = acc_local(0) - imu_bias_[3*i];
		acc_temp(1) = acc_local(1) - imu_bias_[3*i + 1];
		acc_temp(2) = acc_local(2) - imu_bias_[3*i + 2];

		// EB
		//acc_relative = quat2rot(QL_vector[i]) * acc_local;
		// end EB

		//imu_local.linear_acceleration.x = acc_local(0);
		//imu_local.linear_acceleration.y = acc_local(1);
		//imu_local.linear_acceleration.z = acc_local(2);
		
		// imu_local.linear_acceleration.x = acc_relative(0);
		// imu_local.linear_acceleration.y = acc_relative(1);
		// imu_local.linear_acceleration.z = acc_relative(2);

		//std::cout << "Acc Relative " << imu_local.id << " Values: " <<std::endl;
		//std::cout << imu_local.linear_acceleration << std::endl;		
		//std::cout << "*************************************** " <<std::endl;
		// imu.id = imu_ids[i];
		//imu.linear_acceleration.x = (acc_relative(0) - acc[num_imu * 3 - 3]) * 4096;
		//imu.linear_acceleration.y = (acc_relative(1) - acc[num_imu * 3 - 2]) * 4096;
		//imu.linear_acceleration.z = (acc_relative(2) - acc[num_imu * 3 - 1]) * 4096;

		//**** EB 11/22/2017 acc_temp should be now assigned the raw acc data filtered by the butterworth filter
		//acc_temp(0) = (acc_relative(0) - acc_base[0]);
		//acc_temp(1) = (acc_relative(1) - acc_base[1]);
		//acc_temp(2) = (acc_relative(2) - acc_base[2]);

		// acc_temp(0) = filter_accel(acc_local(0),i,0);
		// acc_temp(1) = filter_accel(acc_local(1),i,1);
		// acc_temp(2) = filter_accel(acc_local(2),i,2);
		// acc_temp(0) = acc_local(0);
		// acc_temp(1) = acc_local(1);
		// acc_temp(2) = acc_local(2);
		// **** end EB

		//

		//if (i == 1) {
			//std::cout << "*************************************** " <<std::endl;
			//std::cout << "Acc Local " << imu_local.id << " Values: " <<std::endl;
			//std::cout << acc_temp.transpose() << std::endl;
			//std::cout << "*************************************** " <<std::endl;
		//}




		//std::cout << "Acc Reference: " << acc[num_imu * 3 - 3] << " : " <<  acc[num_imu * 3 - 2] << " : " << acc[num_imu * 3 - 1] << std::endl;
 		//std::cout << "*************************************** " <<std::endl;
		//std::cout << "Acc Relative - Acc Reference " << imu_local.id << " Values: " <<std::endl;
		//std::cout << imu.linear_acceleration << std::endl;		



		Eigen::Vector3d gyro_relative, gyro_local;
		gyro_local <<  gyro[j],  gyro[j + 1], gyro[j + 2];
		gyro_relative = quat2rot(QL_vector[i]) * gyro_local;

		imu.angular_velocity.x = gyro_relative(0) - gyro[num_imu * 3 - 3];
		imu.angular_velocity.y = gyro_relative(1) - gyro[num_imu * 3 - 2];
		imu.angular_velocity.z = gyro_relative(2) - gyro[num_imu * 3 - 1];



//		imu_local.id = imu_ids[i];
		imu.id = imu_ids[i];

		// pi rad rotation around y axis to get tondo's
		imu.linear_acceleration.x = -acc_temp(0) * 4096;
		imu.linear_acceleration.y = acc_temp(1) * 4096;
		imu.linear_acceleration.z = -acc_temp(2) * 4096;



		// MODIFICA 29 AGOSTO 19e54
		// QUI METTO UN MATRICE DI ROTAZIONE PER AVERE ENTRAMBI I GUANTI UGUALI

		//imu.linear_acceleration.x = acc_temp(1); 
		//imu.linear_acceleration.y = acc_temp(0);
		//imu.linear_acceleration.z = -acc_temp(2);

		// imu.linear_acceleration.x = imu.linear_acceleration.x - imu_bias_[3*i];
		// imu.linear_acceleration.y = imu.linear_acceleration.y - imu_bias_[3*i + 1];
		// imu.linear_acceleration.z = imu.linear_acceleration.z - imu_bias_[3*i + 2];

	

		imu_local.angular_velocity.x = gyro_local(0);
		imu_local.angular_velocity.y = gyro_local(1);
		imu_local.angular_velocity.z = gyro_local(2);

		//imus_local.data.push_back(imu_local);
		imus.data.push_back(imu);

		j += 3;

		for (int k=0; k<3; k++) {
			acc_local_file << acc_local(k) << " ";
			acc_temp_file << acc_temp(k) << " ";
			//acc_relative_file << acc_relative(k) << " ";
		}
		imu_linear_acceleration_file << imu.linear_acceleration.x << " " << imu.linear_acceleration.y << " " << imu.linear_acceleration.z << " ";
		//Aucone-Bagheri
		imu_file << imu.linear_acceleration.x << " " << imu.linear_acceleration.y << " " << imu.linear_acceleration.z << " " << imu.angular_velocity.x << " " << imu.angular_velocity.y << " " << imu.angular_velocity.z << " ";

	}

	for (int k=0; k<3; k++) {
		acc_local_file << acc_base[k] << " ";
	}

	acc_local_file << "\n";
	acc_temp_file << "\n";
	//acc_relative_file << "\n";
	imu_linear_acceleration_file << "\n";
	imu_file << "\n";
 	// accFile_ << Row_Acc.transpose() << "\n";
	//std::cout << Row_Acc << "\n";
	imus.header.stamp = t;
	imus.header.frame_id = std::string("frame_ee_glove");
	imus.acquisition_time = ros::Duration();
	pub_glove.publish(imus);
	//pub_glove_local.publish(imus_local);

	
	
}

Eigen::Vector3d quat2Angles(Eigen::Vector4d Q_in)
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

void print_angles(std::vector<Eigen::Vector4d> QL_vector)
{
	Eigen::Vector3d Angles_out;
	float s1, s2, s3, s4, s5;
	Angles_out = quat2Angles(QL_vector[0]);
	s1 = (Angles_out[0] + Angles_out[1] + Angles_out[2]) * (180 / M_PI);
	Angles_out = quat2Angles(QL_vector[1]);
	s2 = (Angles_out[0] + Angles_out[1] + Angles_out[2]) * (180 / M_PI);
	Angles_out = quat2Angles(QL_vector[2]);
	s3 = (Angles_out[0] + Angles_out[1] + Angles_out[2]) * (180 / M_PI);
	Angles_out = quat2Angles(QL_vector[3]);
	s4 = (Angles_out[0] + Angles_out[1] + Angles_out[2]) * (180 / M_PI);
	Angles_out = quat2Angles(QL_vector[4]);
	s5 = (Angles_out[0] + Angles_out[1] + Angles_out[2]) * (180 / M_PI);
	std::cout << "Imu 1: " <<  quat2Angles(QL_vector[0]).transpose() * 180 / M_PI << " : "<< s1 << std::endl;
	std::cout << "Imu 2: " <<  quat2Angles(QL_vector[1]).transpose() * 180 / M_PI << " : "<< s2 << std::endl;
	std::cout << "Imu 3: " <<  quat2Angles(QL_vector[2]).transpose() * 180 / M_PI << " : "<< s3 << std::endl;
	std::cout << "Imu 4: " <<  quat2Angles(QL_vector[3]).transpose() * 180 / M_PI << " : "<< s4 << std::endl;
	std::cout << "Imu 5: " <<  quat2Angles(QL_vector[4]).transpose() * 180 / M_PI << " : "<< s5 << std::endl;
}

Eigen::VectorXd Angles_Sum(std::vector<Eigen::Vector4d> QL_vector)
{
	Eigen::VectorXd Sum;
	Eigen::Vector3d Angles;
	Angles = quat2Angles(QL_vector[0]).transpose() * 180 / M_PI;
	Sum(0) = fabs(Angles(0)) + fabs(Angles(1)) + fabs(Angles(2)); 
	Angles = quat2Angles(QL_vector[1]).transpose() * 180 / M_PI;
	Sum(1) = fabs(Angles(0)) + fabs(Angles(1)) + fabs(Angles(2));
	Angles = quat2Angles(QL_vector[2]).transpose() * 180 / M_PI;
	Sum(2) = fabs(Angles(0)) + fabs(Angles(1)) + fabs(Angles(2));
	Angles = quat2Angles(QL_vector[3]).transpose() * 180 / M_PI;
	Sum(3) = fabs(Angles(0)) + fabs(Angles(1)) + fabs(Angles(2));
	Angles = quat2Angles(QL_vector[4]).transpose() * 180 / M_PI;
	Sum(4) = fabs(Angles(0)) + fabs(Angles(1)) + fabs(Angles(2));
	return Sum;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "glove_relative");
	ROS_INFO("glove_relative started");
	ros::NodeHandle n;
	float f;
	int Counter = 0;
	accFile_.open("savedAccs.txt");
	raw_acc_file.open("/home/emanuele/raw_acc.txt");
	acc_local_file.open("/home/emanuele/acc_local.txt");
	acc_temp_file.open("/home/emanuele/acc_temp.txt");
	imu_linear_acceleration_file.open("/home/emanuele/imu_linear_acceleration.txt");
	imu_file.open("/home/emanuele/IMU_DATA/imu_data.txt");

	std::string acc_topic_name, gyro_topic_name, glove_topic_name;
	n.param<int>("n_imo", num_imu, 6);
	n.param<float>("speed_rate", f, sampleFreq_);
	n.param<std::string>("acc_topic", acc_topic_name, "/qb_class_imu/acc");
	n.param<std::string>("gyro_topic", gyro_topic_name, "/qb_class_imu/gyro");
	n.param<std::string>("glove_topic", glove_topic_name, "/glove_data");

	pub_glove = n.advertise<reactive_grasping::GloveIMUArray>(glove_topic_name, 1);
	pub_glove_local = n.advertise<reactive_grasping::GloveIMUArray>(glove_topic_name + std::string("_local"), 1);
	sub_acc = n.subscribe(acc_topic_name, 1, cb_acc);
	sub_gyro = n.subscribe(gyro_topic_name, 1, cb_gyro);

	acc = std::vector<double>(num_imu * 3, 0.0); // acceleration as measured by the board 
	gyro = std::vector<double>(num_imu * 3, 0.0);  // gyro data as measured by the board
	imu_ids = std::vector<int>(num_imu, 0);
	ros::Rate r(f);
	QL_vector = std::vector<Eigen::Vector4d>(5, Eigen::Vector4d(1.0, 0., 0., 0.));

	// //SIMONE CIOTTI - START
	Eigen::VectorXd Raw_Acc_sum(15);
	unsigned int cnt_row_acc_sum = 0;

	for(int i=0;i<15;i++)
	{
	 	Raw_Acc_sum[i] = 0;
	 	imu_bias_[i] = 0;
	}
	// //SIMONE CIOTTI - END

	//**** EB 11/22/2017 now used to remove bias
	for (double i = 0.; i < 1.; i = i + 1.0 / f) // prev. i<5
	{
		ros::spinOnce();
		// Eigen::MatrixXd All_Q = Eigen::MatrixXd::Zero(4, num_imu-1);
		
		for (int j = 0; j < num_imu - 1; ++j)
		{

			// All_Q.block<4,1>(0,j) = QL_vector[j];
				//QL_vector[j] = MadgwickFilter(j, num_imu - 1, QL_vector[j]);
			// std::cout << "Q( " << j << "): " << QL_vector[j] << " " << std::endl;
		}
		// std::cout << i << std::endl <<  All_Q << std::endl << std::endl << std::endl;
		publish( ros::Time::now() );
		r.sleep();
		// This needs to be checked
		Raw_Acc_sum = Raw_Acc_sum + Raw_Acc_;
 		cnt_row_acc_sum++;
	}
	imu_bias_ = Raw_Acc_sum/cnt_row_acc_sum;
	std::cout << "Bias estimated: " << imu_bias_ << std::endl;
	// getchar();

	for(int i=0;i<15;i++)
	{
	 	Raw_Acc_sum[i] = 0;
	 	// imu_bias_[i] = 0;
	}

	cnt_row_acc_sum = 0;


	for (double i = 0.; i < 1.; i = i + 1.0 / f) // prev. i<5
	{
		ros::spinOnce();
		// Eigen::MatrixXd All_Q = Eigen::MatrixXd::Zero(4, num_imu-1);
		
		for (int j = 0; j < num_imu - 1; ++j)
		{

			// All_Q.block<4,1>(0,j) = QL_vector[j];
				//QL_vector[j] = MadgwickFilter(j, num_imu - 1, QL_vector[j]);
			// std::cout << "Q( " << j << "): " << QL_vector[j] << " " << std::endl;
		}
		// std::cout << i << std::endl <<  All_Q << std::endl << std::endl << std::endl;
		publish( ros::Time::now() );
		r.sleep();
		// This needs to be checked
		Raw_Acc_sum = Raw_Acc_sum + Raw_Acc_;
 		cnt_row_acc_sum++;
	}

	imu_bias_ = Raw_Acc_sum/cnt_row_acc_sum;

	std::cout << "Bias estimated: " << imu_bias_ << std::endl;
	// end EB

 	

	// 	r.sleep();
	// }

	// std::cout << "Bias estimated: " << imu_bias_ << std::endl;
	// getchar();


	
	//**** EB 11/22/2017 
		//print_angles( QL_vector );
	// end EB
	
	// std::cout << "Press RETURN to continue: " << std::endl;
	// getchar();
	std::cout << "Done: enjoy " << std::endl;

	//**** EB 11/22/2017 not needed anymore 
	//for (double i = 0.; i < 5.; i = i + 1.0 / f)
	//{
	//	ros::spinOnce();
		// Eigen::MatrixXd All_Q = Eigen::MatrixXd::Zero(4, num_imu-1);
	
		//for (int j = 0; j < num_imu - 1; ++j)
		//{
			// All_Q.block<4,1>(0,j) = QL_vector[j];
				//QL_vector[j] = MadgwickFilter(j, num_imu - 1, QL_vector[j]);
				// end EB
			// std::cout << "Q( " << j << "): " << QL_vector[j] << " " << std::endl;
		//}
		//end EB
		// std::cout << i << std::endl <<  All_Q << std::endl << std::endl << std::endl;
	//	publish( ros::Time::now() );
	//	r.sleep();
	//}

	//print_angles( QL_vector );
	//std::cout << "Press RETURN to continue: " << std::endl;
	//getchar();

	//end EB

	// Eigen::VectorXd Sum_Angles_0;
	// Eigen::VectorXd Sum_Angles;

	// Sum_Angles_0 = Angles_Sum(  QL_vector  );

	// //SIMONE CIOTTI - START
	// for (double i = 0.; i < 5.; i = i + 1.0 / f)
	// {
	// 	ros::spinOnce();
		
	// 	for (int j = 0; j < num_imu - 1; ++j)
	// 	{
	// 		QL_vector[j] = MadgwickFilter(j, num_imu - 1, QL_vector[j]);
	// 	}
	// 	publish( ros::Time::now() );

	
	// //SIMONE CIOTTI - END

	while (ros::ok())
	{

		for (int j = 0; j < num_imu - 1; ++j)
		{
			//**** EB 11/22/2017 
				//QL_vector[j] = MadgwickFilter(j, num_imu - 1, QL_vector[j]);
			// end EB
		}
		// Sum_Angles = Angles_Sum(  QL_vector  );
		ros::spinOnce();
		publish( ros::Time::now() );

		for (int j=0; j<= num_imu*3 -1; j++)
		{
			raw_acc_file << acc[j] << " ";
		}
		raw_acc_file << "\r\n";

		//**** EB 11/22/2017 
		//for (int i=0; i< num_imu; i++)
		//{
		//	raw_quat_file << QL_vector[i].transpose() << " ";
		//}
		//raw_quat_file << "\r\n";
		// end EB

		r.sleep();
		Counter = Counter + 1;
		if (Counter = 1000000) {
			Counter = 0;
			//print_angles( QL_vector );	
			// std::cout << std::endl;
		}
	}

	return 0;
}
