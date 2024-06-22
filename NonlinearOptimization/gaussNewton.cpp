#include<Eigen/Core>
#include<Eigen/Dense>
#include<opencv4/opencv2/core.hpp>
#include<iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv){

    double at = 1.0, bt = 2.0, ct = 1.0; // ground truth values of a,b,c
    double ae = 2.0, be = -1.0, ce = 5.0; // initial estimates of a,b,c
    cout << "The ground truth values of a,b,c are :\n" << at << '\t' << bt << '\t' << ct << endl;
    double w_sigma = 1.0; // variance or sigma of the noise
    double inv_w_sigma = 1.0 / w_sigma; 
    cv::RNG rng;
    int N = 100;
    // Generating data for fitting by inducing noise with ground truth parameters
    vector<double> x_data, y_data;
    for(int i=0; i<N; i++){
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp( at*x*x + bt*x + ct) + rng.gaussian(w_sigma*w_sigma));    
    }
    // now solving for the dx state vector to find the estimates of a,b,c
    int iterations=100;
    double cost = 0, last_cost = 0;
    for(int i=0; i<iterations;i++){
        //computing the Jacobian and error values for all the data points
        Matrix3d H = Matrix3d::Zero();
        Vector3d b = Vector3d::Zero();
        double ei=0;
        
        for(int j=0; j<N; j++){
            Vector3d J;
            double xi = x_data[j], yi = y_data[j];
            double error = yi - exp(ae*xi*xi + be*xi + ce);
            
            J[0] = -xi*xi*exp(ae*xi*xi + be*xi + ce);
            J[1] = -xi * exp(ae*xi*xi + be*xi + ce);
            J[2] = -exp(ae*xi*xi + be*xi + ce);

            H+= inv_w_sigma*inv_w_sigma*J*J.transpose();
            b+= -inv_w_sigma * inv_w_sigma * error * J;
            cost += error*error;
        }
        Vector3d dx = H.ldlt().solve(b); // Solve Hx=b
        if(isnan(dx[0])){
            cout << "result is nan!" << endl;
            break;
        }

        if(i>10 && cost >= last_cost){
            cout << "cost is : " << cost << " last cost is : " << last_cost << endl;
            break; 
        }
        // updating the estimates
        ae +=dx[0]; 
        be +=dx[1];
        ce +=dx[2];
        last_cost = cost;
        
    }

    cout << "\nThe estimated values of a, b, c are:\n" << ae << '\t' << be << '\t' << ce << endl;


    return 0;
}