/*
this code is developed by Joel Thazatha, a Graduate from UNC Charlotte
The code is designed in such a way that it takes values from a text file (data.txt) containing the input data in the order of (X coordinate, Y coordinate, delta time , gyro yaw) and gives output as a text file(output.txt) with the values of heading in 2 different methods( theta_method1 , theta_method2).



the out values of the estimated pose are stored inside global variables and hence can be used everywhere
*/







#include<iostream>
#include<stdlib.h>
#include<fstream>
#include<vector>
#include<string>
#include<sstream>
#include<cmath>
#include"Eigen/Dense"
#include<stdexcept>
#include <iterator>

using namespace std;



// global Variables

double time_delta = 0;
double gyro_yaw;
double x_prevpos = 0.0;
double y_prevpos = 0.0;
double x_current = 0.0;
double y_current = 0.0;
double theta_method2 = 0.0;
double theta_method1 = 0.0;
double linear_distance_prev = 0.0;
double result=0;
bool initialized = true;

std::vector <double> output_method1;
std::vector <double> output_method2;

const float pi = 3.142;

Eigen::MatrixXd A(4, 4); // System dynamics matrix
Eigen::MatrixXd C(4, 4); // Output matrix
Eigen::MatrixXd Q(4, 4); // Process noise covariance
Eigen::MatrixXd R(4, 4); // Measurement noise covariance
Eigen::MatrixXd P(4, 4);  // Estimate error covariance
Eigen::MatrixXd I(4, 4);
Eigen::MatrixXd K(4, 4);
Eigen::VectorXd Pos(4);
Eigen::VectorXd Pos_new(4);
Eigen::MatrixXd Measured(4, 1);
/*
Eigen::MatrixXd A(3, 3); // System dynamics matrix
  Eigen::MatrixXd C(1, 3); // Output matrix
  Eigen::MatrixXd Q(3, 3); // Process noise covariance
  Eigen::MatrixXd R(1, 1); // Measurement noise covariance
  Eigen::MatrixXd P(3, 3); // Estimate error covariance
  Eigen::MatrixXd I(3, 3);
  Eigen::MatrixXd K(3, 1);
  Eigen::VectorXd Pos(3);
  Eigen::VectorXd Pos_new(3);
  Eigen::MatrixXd Measured(1, 1);*/
  


/******************************************************************************************************************
Method 1 calculates the value of heading when the robot is travelling from previous coordinates to next coordinate
********************************************************************************************************************/
double estimate_heading_method1(double x_current,double y_current, double gryo_yaw, double time_delta)
{
    double x_delta= x_current - x_prevpos;
    double y_delta= y_current - y_prevpos;
    double result= 0;
    if (x_prevpos==0 && y_prevpos==0)
        result= gyro_yaw*time_delta;
    else
        result = atan2(y_delta, x_delta) + gyro_yaw*time_delta;
    if (result<0)
        result +=(2*pi);
    x_prevpos = x_current;
    y_prevpos = y_current;
    output_method1.push_back(theta_method1);
    return result;
}



/******************************************************************************************************************
Method 2 uses Kalman Filter to predict one of the possible heading of the robot.
********************************************************************************************************************/
double estimate_heading_method2(double x_current,double y_current, double gryo_yaw, double time_delta)
{

    double w_deltatime = (gyro_yaw*time_delta);
    double theta= result + w_deltatime;

// Discrete LTI projectile motion, measuring position only
    A << 1, 0, -1, 0, 0, 1, 0, -1, 0, 0, 1, 0, 0, 0, 0, 1;
    C << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
// Reasonable covariance matrices
    Q << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1;
    R << 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1;
    I.setIdentity();
    Measured << x_current, y_current, theta_method1, 0;
    Pos_new = A * Pos;
    P = A*P*A.transpose() + Q;
    K = P*C.transpose()*(C*P*C.transpose() + R).inverse();

    Pos_new += K * (Measured - C*Pos_new);
    P = (I - K*C)*P;
    Pos = Pos_new;
    result = Pos[2] ;
    x_prevpos = Pos[0];
    y_prevpos = Pos[1];
    result = std::fmod(result, (2*pi));
    if (result<0)
        result +=(2*pi);

    output_method2.push_back(theta_method2);
    return result;
}




int main()
{
// reading the input file and converting the data into a vector
    ifstream data_file;
    data_file.open("data.txt");
    std::vector<vector <string> > input_array;
    while (data_file) {
        string s;
        if (!getline( data_file, s )) break;

        istringstream ss( s );
        vector <string> line;

        while (ss) {
            string s;
            if (!getline( ss, s, ',' )) break;
            line.push_back(s);
        }

        input_array.push_back(line);
    }
    data_file.close();
    /* for (int i = 0; i < input_array.size(); i++) {
     	for (int j = 0; j < input_array[i].size(); j++)
     	    cout << input_array[i][j] << " ";
     	cout << endl;
    }*/
    P << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    int i=0;
    std::vector<vector <double> > output_array;
    while(i< input_array.size()) {
        x_current = atof(input_array[i][0].c_str());
        y_current = atof(input_array[i][1].c_str());
        time_delta = atof(input_array[i][2].c_str());
        gyro_yaw = atof(input_array[i][3].c_str());
       
        theta_method1= estimate_heading_method1(x_current, y_current, gyro_yaw, time_delta);
        theta_method2= estimate_heading_method2(x_current, y_current, gyro_yaw, time_delta);
        //cout<<theta_method2<<"      			"<<theta_method1<<"     				"<<i<<endl;
	
        i++;

    }

// Generating the output file
 std::ofstream output_file("./output.txt");
   for (int i = 0; i < output_method1.size(); i++) {
	output_file << output_method1[i]<< " 					"<< output_method2[i]<<endl;
}
   output_file.close(); 
}
