#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <CGAL/QP_solver/assertions.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <qptrajectory.h>
#include <stdio.h>
double f(std::vector<double> data , double t){
    double sum =0.0 , var =1;
    for(int i =0 ; i<data.size();i++){
        sum+= data[i]*var;
        var*=t;
    }
    return sum;
}
void MainWindow::update(){

  //  plan = new qptrajectory;
//    ui->customplot->replot();
}






MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("Quadratic Programming");


    newCurve = new QCPCurve(ui->customplot->xAxis, ui->customplot->yAxis);
    newCurve2 = new QCPCurve(ui->customplot->xAxis, ui->customplot->yAxis);
    newCurve3 = new QCPCurve(ui->customplot->xAxis, ui->customplot->yAxis);
    time =new QTimer(this);
    newCurve->setName("Trajectory 1st");
    newCurve2->setName("Trajectory 2nd");
    newCurve3->setName("Trajectory 3rd");
    ui->customplot->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);
    ui->customplot->axisRect()->setupFullAxesBox(true);
    ui->customplot->legend->setVisible(true);
    ui->customplot->xAxis->setRange(-5,5);
    ui->customplot->yAxis->setRange(-5,5);
    connect(time,SIGNAL(timeout()),this,SLOT(update()));

    std::vector<segments> path;
    trajectory_profile p1 , p2 ,p3  ;
    p1.pos<< 1,0,0;
    p1.vel<< -1,1,0;
    p1.acc<< 0.3,-0.2,0;

    p2.pos<< 0.0,2.0,0;
    p2.vel<< 0.3,-1.0,0;
    p2.acc<< 1.2,-0.2,0;

    p3.pos<< -3.0,1.0,0;
    p3.vel<< -0.2,-1.0,0;
    p3.acc<< 0.0,0.3,0;
    std::cout << "here" <<std::endl;
    path.push_back(segments(p1,p2,2));
    path.push_back(segments(p2,p3,2));
    path.push_back(segments(p3,p1,2));

   profile = plan->get_profile(path,2.0,0.01);

   for(int i=0;i<profile.size();i++){
       std::cout <<  "============"<<i <<std::endl;
       std::cout  << profile[i].pos.transpose() <<std::endl;

   }

   data.clear();
   for(int i=0 ;i<profile.size();i++){
       data.push_back(QCPCurveData(i, profile[i].pos[0]  ,profile[i].pos[1])) ;
   }
   newCurve->data()->set(data);

    ui->customplot->replot();
    ui->customplot2->addGraph();

    for(int i=0;i<profile.size();i++){

        ui->customplot2->graph()->addData(i*0.01,profile[i].vel[0]);
    }

    ui->customplot2->setInteractions(QCP::iRangeDrag);

    ui->customplot2->xAxis->setRange(0,3);
    ui->customplot2->yAxis->setRange(-10,10);

    ui->customplot2->replot();
   // segments seg1 , seg2 ,seg3 ,seg4;
   // seg1(p1,p2,1.0);


    //profile = plan->get_profile(seg1x   , seg2x  ,  );


   //time->start(50);
}

MainWindow::~MainWindow()
{
    delete ui;
}
