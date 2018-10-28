#ifndef qptrajectory_H
#define qptrajectory_H
#include <stdio.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <Python.h>

//#include <cvxopt/cvxopt.h>





//typedef CGAL::Quadratic_program<int> Program;
//typedef CGAL::Quadratic_program_solution<ET> Solution;
using namespace Eigen;


typedef struct position{

    Eigen::Vector3d pos;

    position(double x , double y , double z)
    {
      pos[0] = x;
      pos[1] = y;
      pos[2] = z;
    }
}position_type;


struct pose_type{
    position_type position; // x y z
    //orientation_type orientation; //
};

struct waypoint{
    double time;
    pose_type pose;
};



class trajectory_profile
{
public:
    trajectory_profile(){}
    trajectory_profile(Eigen::Vector3d pos_,
                       Eigen::Vector3d vel_,
                       Eigen::Vector3d acc_,
                       double time
                       ):
        pos(pos_) , vel(vel_) , acc(acc_)

    {

    }
    double time;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
};





class segments{

public:
    segments(){}
    segments(trajectory_profile b_c_ ,trajectory_profile t_c_ ,double t )
    {
        b_c = b_c_;
        t_c = t_c_;
        time_interval =t;
    }
trajectory_profile b_c;
trajectory_profile t_c;
double time_interval;

};

class profile{
public:
    profile(){}
    double position;
    double velocity;
    double acceleration;
    double jerk;
    Eigen::Vector4d V;
    profile(double p , double v , double a , double j){
        position = p ;
        velocity = v ;
        acceleration = a ;
        jerk =j;
        V[0] = p;
        V[1] = v;
        V[2] = a;
        V[3] = j;
    }

};

typedef  std::vector<segments> path_def;

class qptrajectory{


public:
    qptrajectory();
    ~qptrajectory();
    void set_waypoints(waypoint data);
    double get_position(double time);
    std::vector<trajectory_profile> get_profile(std::vector<segments> seg , double time_interval , double dt );
    std::vector<double> qpsovle(profile begin , profile end, double time_interval);

    double polynomial(std::vector<double> data ,double t);
    double polynomial_d1(std::vector<double> data ,double t);
    double polynomial_d2(std::vector<double> data ,double t);
    double polynomial_d3(std::vector<double> data ,double t);

    //Program *qp;

private:




};

#endif // qptrajectory_H

