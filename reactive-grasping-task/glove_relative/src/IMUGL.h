/*
    IMUGL - IMU LIBRARY
    
    Copyright (C) 2016  Emanuele Luberto (emanuele.luberto@gmail.com)
    
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
#include <string>
#include <sstream>
#include <vector>
#include <ctime>
#include <iostream>
#include <csignal>
#include <cstdlib>

#include <cmath>
#include <math.h>
#include <boost/chrono.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio/serial_port.hpp> 
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <qb_interface/inertialSensorArray.h>



#define _DEBUG_DATA_BUFFER_ 0
#define _DEBUG_REAL_DATA_   1

// # step for compute offset angles
#define _OFFSET_STEP_      500


// sgn function
#define sgn(s) (s>=0.0?1:-1)



class IMUGL
{
public:
    /*          Contructor         */
    IMUGL();

    /*          Destructor         */
    ~IMUGL();
    
    // public variables
    Eigen::MatrixXd Acc_;
    Eigen::MatrixXd Acc_Old_;
    Eigen::MatrixXd Gyro_;
    Eigen::MatrixXd Gyro_Old_;


    int nIMU_;
    double sampleFreq_;
    double beta_;
    double thGyro_;
    // std::map <std::string, std::vector<int>> chains_;
    // std::map <std::string, Eigen::MatrixXd> angles_map_;     // map size (nchains, Matrix(nimu4chain,3))
    // std::map <std::string, Eigen::MatrixXd> q_map_;          // map size (nchains, Matrix(nimu4chain,4))
    // std::map <std::string, Eigen::MatrixXd> q_map_off_;      // map size (nchains, Matrix(nimu4chain,4))
    // std::map <std::string, Eigen::MatrixXd> q_map_joints_;   // map size (nchains, Matrix(nimu4chain,4))
    std::vector<Eigen::Vector4d> QL_vector;
    std::vector<Eigen::Vector3d> angles;



    //================================================================     CallBack
    ros::NodeHandle n_;
    void callbackAcc(qb_interface::inertialSensorArray imu);
    void callbackGyro(qb_interface::inertialSensorArray imu);
    ros::Subscriber sub_acc_;
    ros::Subscriber sub_gyro_;
    bool acc_flag_;
    bool gyro_flag_;

    //================================================================     Start
    void start();

    //================================================================     InitialOffset
    void initialOffset();

    //================================================================     ComputeAngles
    void computeAngles();

    //================================================================     WaitBoard

    void waitBoard();

    //================================================================     PrintIMUangles
    void printIMUangles();

    //================================================================     OpenFiles
    void openFiles(std::string s);  
    bool flag_write_;
    std::string pkg_path_;
    FILE *fileAcc_, *fileGyro_, *fileAngles_, *fileParam_, *fileAngles_Offset_;
    
    //================================================================     SaveData 
    void saveData();  

    //================================================================     CloseFiles 
    void closeFiles();  

    


private:

    //================================================================     MadgwickFilter
    Eigen::Vector4d MadgwickFilter( int P, int N, Eigen::Vector4d  qL);

    //================================================================     OffsetCorrector
    void offsetCorrector();

    //================================================================     Quat2Angles
    Eigen::Vector3d quat2Angles( Eigen::Vector4d Q_in );

    //================================================================     Quat2anglesTOT
    void quat2anglesTOT();

    //================================================================     Useful Functions 
    Eigen::Matrix3d rotX( float x );
    Eigen::Matrix3d rotY( float y );
    Eigen::Matrix3d rotZ( float z );
    Eigen::Matrix3d rot( float x, float y, float z );
    Eigen::Matrix3d skew( Eigen::Vector3d a );
    Eigen::Matrix3d Quat2Rot( Eigen::Vector4d Q_in );
    Eigen::Vector4d ConjQ( Eigen::Vector4d Q_in );
    Eigen::Vector4d QxQ( Eigen::Vector4d Q_1, Eigen::Vector4d Q_2 );
    Eigen::Vector4d Rot2Quat( Eigen::Matrix3d R_in );
    Eigen::Vector3d Rot2Angle( Eigen::Matrix3d R_in );

    //================================================================     AbductionCorrector 
    // void abductionCorrector();
    
};

































