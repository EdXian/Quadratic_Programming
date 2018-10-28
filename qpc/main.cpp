#include <stdio.h>
#include <iostream>
#include <chrono>
#include <ctime>

#include <qptrajectory.h>

double t=1.6;
double b0=2;
double b1=6;


double q[4][4] = {
  0.0, 0.0 , 0.0 ,0.0,
  0.0, 0.0 , 0.0 ,0.0,
  0.0, 0.0 , (double)(b0*b0*t) ,(double)(b0*b1*t*t/2),
  0.0, 0.0 , (double)(b0*b1*t*t/2) ,(double)(b1*b1*t*t*t/3),
};


double a[4][4] = {
  1.0, 0.0 , 0.0 ,0.0,
  0.0, 1.0 , 0.0 ,0.0,
  1.0, t , t*t ,t*t*t,
  0.0, 1.0 , 2*t ,3*t*t,
};

double b[4] = {
1.6,4.3,2.5,5.2
};
int main(int *argc , char ** argv){

    auto start = std::chrono::system_clock::now();


    qptrajectory plan;

    path_def path;
    trajectory_profile wp1 , wp2 ,wp3  ;
    std::vector<trajectory_profile> data;

    wp1.pos << 1.0,0,0;
    wp1.vel << 1.0,-1.0,0;
    wp1.acc << 0.3,0.4,0;

    wp2.pos<< 0.0,3.0,0;
    wp2.vel<< -1,0.20,0;
    wp2.acc<< 0.4,0.3,0;

    wp3.pos<< -3.0,-1.0,0;
    wp3.vel<< 1.0,-0.0,0;
    wp3.acc<< 0.5,-0.5,0;

    ///path.push_back(segments(wp1 , wp2 ,1.1));
    path.push_back(segments(wp1,wp2,1.1));
    path.push_back(segments(wp2,wp3,2.3));
    path.push_back(segments(wp3,wp1,1.4));

    data = plan.get_profile(path,1.0,0.01);

    std::cout <<std::endl;

    std::cout <<data[0].acc.transpose()<<std::endl;

    std::cout << "size " <<data.size()<<std::endl;

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "elapsed time : " <<elapsed_seconds.count()<<std::endl;
}
