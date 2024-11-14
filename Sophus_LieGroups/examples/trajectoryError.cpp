#include <iostream>
#include <fstream>
using namespace std;

#include <pangolin/pangolin.h>
#include "sophus/se3.hpp"
#include <unistd.h>

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Eigen::Isometry3d>> Trajectorytype;

string groundtruth_file = "/home/karan/Slambook/Sophus_LieGroups/examples/groundtruth.txt";
string estimated_file = "/home/karan/Slambook/Sophus_LieGroups/examples/estimated.txt";

void DrawTrajectory(const Trajectorytype &gt, const Trajectorytype &et);
Trajectorytype ReadTrajectory(const string &path);

int main(int argc, char **argv){
    Trajectorytype gt = ReadTrajectory(groundtruth_file);
    Trajectorytype et = ReadTrajectory(estimated_file);
    assert(!gt.empty() && !et.empty());
    assert(gt.size() == et.size());

    // Calculate RMSE error or Absolute Trajectory Error
    double rmse = 0;
    for(int i=0;i<gt.size(); i++){
        auto err = (gt[i].inverse()*et[i]).log().norm();
        rmse = rmse + (double)(err*err);
    }
    rmse = rmse/((double)gt.size());
    rmse = sqrt(rmse);
    cout << "RMSE = " << rmse << endl;

    // Plot both the trajectories
    DrawTrajectory(gt, et);
}

Trajectorytype ReadTrajectory(const string &path){
    ifstream fin(path);
    Trajectorytype poses;

    if(!fin.is_open()){
        cerr << "Error opening the file" << endl;
        return poses;
    }

    while(!fin.eof()){
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3d p1(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
        poses.push_back(p1);
    }
    return poses; 
}

void DrawTrajectory(const Trajectorytype &gt, const Trajectorytype &et){
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768); // window size
    glEnable(GL_DEPTH_TEST); // to remove hidden surface by enabling depth-buffer
    glEnable(GL_BLEND); // enables blending which could be used to implement transperency within objects
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // func to assign values to C_source  
                                                     // and C_destinationin blend equation

    pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    ); // setting up camera state by passing projection matrix and 
    // ModelViewLookAt(where camera is?, where object is?, where camera is looking?)

    pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam)); // similarly setting up display cam parameters

    while(pangolin::ShouldQuit() == false){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);

        for (size_t i = 0; i < gt.size() - 1; i++) {
            glColor3f(0.0, 0.0, 1.0);
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < et.size() - 1; i++) {
            glColor3f(1.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = et[i], p2 = et[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();

        usleep(5000);   // sleep 5 ms    
    }

}