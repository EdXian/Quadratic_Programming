#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
//#include <qptrajectory.h>
#include <stdio.h>
#include <QMouseEvent>



void MainWindow::update(){
    link->setVisible(true);
    double dt = 0.02;
    if(loop_count > loop_size){
        loop_count = 0;
    }




    link->start->setCoords(px_2 ,py_2);
    link->end->setCoords(lpx ,lpy);

     px += vx *dt; py += vy *dt;
     vx += ax *dt; vy += ay *dt;
     ax = profile[loop_count].acc[0] ;
     ay = profile[loop_count].acc[1] ;

     path_data.push_back(QCPCurveData(path_data.size(),px_2,py_2));
     payload_data.push_back(QCPCurveData(path_data.size() , (px_2 + lpx)/2,(py_2 + lpy)/2    ));
     if(path_data.size()>120){
         path_data.pop_front();
     }
     if(payload_data.size()>120){
         path_data.pop_front();
     }

    px_2 += vx_2*dt;
    py_2 += vy_2*dt;
    vx_2 += ax_2 *dt;
    vy_2 += ay_2 *dt;
    ax_2 = ax + (1 / mass) * ( damping * (vx - vx_2) + spring* (px - px_2 ) + forcex );
    ay_2 = ay + (1 / mass) * ( damping * (vy - vy_2) + spring* (py - py_2 ) + forcey );

    //unit vector of the link
    u_lx = (px_2 - lpx)/sqrt((px_2 - lpx)*(px_2 - lpx) + (py_2 - lpy)*(py_2 - lpy))   ;
    u_ly = (py_2 - lpy)/sqrt((px_2 - lpx)*(px_2 - lpx) + (py_2 - lpy)*(py_2 - lpy))   ;

//    fint_x = l_damping*(vx_2 - lvx) + l_spring*(px_2-lpx)*u_lx;
//    fint_y = l_damping*(vy_2 - lvy) + l_spring*(py_2-lpy)*u_ly;


    //std::cout <<sqrt((u_lx)*(u_lx)+(u_ly)*(u_ly))<<std::endl;
    //estimate  force from the ukf
    fint_x =  (l_spring*(px_2-lpx - u_lx*length) + l_damping*(vx_2-lvx));
    fint_y =  (l_spring*(py_2-lpy - u_ly*length) + l_damping*(vy_2-lvy));
   // std::cout <<  fint_x <<" : " <<fint_y <<std::endl;
    lax = lm*(fint_x-0)*1.0- (lvx - vx_2)*2.0;
    lay = lm*(fint_y - 0)*1.0- (lvy - vy_2)*2.0;
    lvx +=lax*dt;      lvy +=lay*dt;
    lpx +=lvx*dt;      lpy +=lvy*dt;

    forcex =-1* fint_x;
    forcey = -1*fint_y;

   // lpx = lvx *dt; lpx = lvx *dt;



    circle_data.clear();
    circle_data.resize(100);
    for(int i = 0 ; i<100 ;i++){
       circle_data[i]=QCPCurveData(i+1, px_2+0.2*cos((2*3.14159/100)*i), py_2+0.2*sin((2*3.14159/100)*i) );
    }

    follower_data.clear();
    follower_data.resize(100);
    for(int i=0;i<100;i++){
        follower_data[i] = QCPCurveData(i+1 , lpx+0.2*cos((2*3.14159/100)*i) ,lpy+0.2*sin((2*3.14159/100)*i));
    }


    QPen pen;
    pen.setWidth(6);

    pen.setColor(Qt::black);
    circle->setPen(pen);
    circle->data()->set(circle_data,true);
    pen.setColor(Qt::red);
    follower->setPen(pen);
    follower->data()->set(follower_data ,true);
    pen.setStyle(Qt::DashDotLine);
    pen.setColor(Qt::darkMagenta);
    payload_curve->setPen(pen);
    payload_curve->data()->set(payload_data,true);
    pen.setStyle(Qt::SolidLine);

    pen.setColor(Qt::cyan);
    pen.setWidth(7);
    pen.setStyle(Qt::DashLine);
    pen.setWidth(7);

    path_curve->setPen(pen);
    path_curve->data()->set(path_data,true);
    pen.setStyle(Qt::SolidLine);

    loop_count++;
    ui->customplot->replot();

}






MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("Quadratic Programming");
    start=false;
    ui->customplot2->setInteractions(QCP::iRangeZoom );
    ui->customplot3->setInteractions(QCP::iRangeZoom );
    ui->customplot4->setInteractions(QCP::iRangeZoom );

    QPen pen;
    pen.setWidth(6);
    newCurve = new QCPCurve(ui->customplot->xAxis, ui->customplot->yAxis);
    newCurve2 = new QCPCurve(ui->customplot->xAxis, ui->customplot->yAxis);
    newCurve3 = new QCPCurve(ui->customplot->xAxis, ui->customplot->yAxis);
    circle = new QCPCurve(ui->customplot->xAxis , ui->customplot->yAxis);
    path_curve = new QCPCurve(ui->customplot->xAxis , ui->customplot->yAxis);
    payload_curve = new QCPCurve(ui->customplot->xAxis , ui->customplot->yAxis);
    follower = new QCPCurve(ui->customplot->xAxis , ui->customplot->yAxis);
    circle->setName("Leader");
    follower->setName("Follower");
    path_curve->setName("the trajectory\n of the leader");
    payload_curve->setName("the trajecteory\n of the payload");

    payload_data.clear();
    path_data.clear();
    follower_data.clear();

   // newCurve->removeFromLegend();
    newCurve2->removeFromLegend();
    newCurve3->removeFromLegend();

    ui->customplot->legend->setVisible(true);
    time =new QTimer(this);
    newCurve->setName("Desired trajectory ");
    newCurve2->setName("Trajectory 2nd");
    newCurve3->setName("Trajectory 3rd");
    item = new QCPItemLine(ui->customplot);
    item->setVisible(false);
    link = new QCPItemLine(ui->customplot);

    pen.setColor(Qt::gray);
    link->setPen(pen);

    ui->customplot->plotLayout()->insertRow(0);
    QCPTextElement *title = new QCPTextElement(ui->customplot, "Trajectory", QFont("system",15, QFont::Times));
    ui->customplot->plotLayout()->addElement(0, 0, title);

    ui->customplot2->plotLayout()->insertRow(0);
    QCPTextElement *title2 = new QCPTextElement(ui->customplot2, "displacement profile", QFont("system",15, QFont::Times));
    ui->customplot2->plotLayout()->addElement(0, 0, title2);

    ui->customplot3->plotLayout()->insertRow(0);
    QCPTextElement *title3 = new QCPTextElement(ui->customplot3, "velocity profile", QFont("system",15, QFont::Times));
    ui->customplot3->plotLayout()->addElement(0, 0, title3);

    ui->customplot4->plotLayout()->insertRow(0);
    QCPTextElement *title4 = new QCPTextElement(ui->customplot4, "acceleration profile", QFont("system",15, QFont::Times));
    ui->customplot4->plotLayout()->addElement(0, 0, title4);

    ui->customplot->axisRect()->setupFullAxesBox(true);
   // ui->customplot->legend->setVisible(true);


    l_damping = 1.0;
    l_spring = 7.0;
    mass = 1.0;
    damping = 1.0;
    spring  =1.0;
    forcex=0.0;
    forcey = 0.0;
    lm = 1.0;

    ui->customplot->xAxis->setRange(-6,6);
    ui->customplot->yAxis->setRange(-6,6);

    std::vector<segments> path;
    trajectory_profile p1 , p2 ,p3  ;


    //setup init condition
    loop_size = 0;
    loop_count= 0;
    fint_x = 0.0;
    fint_y = 0.0;
    px = 1.0; py = 0.0;
    vx = 0.0; vy = 0.0;
    ax = 0.0; ay=  0.0;

    lpx = 2.0; lpy = 0.0;
    lvx = 0.0; lvy = 0.0;
    lax = 0.0; lay = 0.0;

    length = 0.9;

    px_2 = 1.0; py_2=0.0;
    ax_2 = 0.0; ay_2=1.0;
    vx_2 = 0.0; vy_2=1.0;

    p1.pos << 1.0,0,0;
    p1.vel << 0.0,0.0,0;
    p1.acc << 0.00,-0.0,0;

    p2.pos<< 0.0,3.0,0;
    p2.vel<< -0.3,-0.20,0;
    p2.acc<< -0.1,-0.3,0;

    p3.pos<< -3.0,-1.0,0;
    p3.vel<< 0.3,-0.1,0;
    p3.acc<< 0.3,0.2,0;

   path.push_back(segments(p1,p2,3));
   path.push_back(segments(p2,p3,3));
   path.push_back(segments(p3,p1,3));

   profile = plan->get_profile(path,2.0,0.02);

   loop_size= profile.size();

   link->start->setCoords(px_2 ,py_2);
   link->end->setCoords(lpx ,lpy);

   //std::cout << profile.size() <<std::endl;
   data.clear();
   for(int i=0 ;i<profile.size();i++){
       data.push_back(QCPCurveData(i, profile[i].pos[0]  ,profile[i].pos[1])) ;
   }
   newCurve->data()->set(data,true);

   circle_data.clear();
   circle_data.resize(100);
   for(int i = 0 ; i<100 ;i++){
      circle_data[i]=QCPCurveData(i+1, 1+0.2*cos((2*3.14159/100)*i), 0.2*sin((2*3.14159/100)*i) );
   }

    pen.setColor(Qt::black);
    circle->setPen(pen);
    circle->data()->set(circle_data);

    follower_data.clear();
    follower_data.resize(100);
    for(int i=0;i<100;i++){
        follower_data[i] = QCPCurveData(i+1 , lpx+0.2*cos((2*3.14159/100)*i) ,lpy+0.2*sin((2*3.14159/100)*i));
    }

    pen.setColor(Qt::red);
    follower->setPen(pen);
    follower->data()->set(follower_data);


    ui->customplot2->addGraph();
    ui->customplot2->addGraph();
    pen.setColor(Qt::red);
    ui->customplot2->graph(0)->setPen(pen);
    pen.setColor(Qt::blue);
    ui->customplot2->graph(1)->setPen(pen);
    ui->customplot2->graph(0)->setName("x");
    ui->customplot2->graph(1)->setName("y");

    for(int i=0;i<profile.size();i++){
        ui->customplot2->graph(0)->addData(i*0.02,profile[i].pos[0]);
        ui->customplot2->graph(1)->addData(i*0.02,profile[i].pos[1]);
    }

    ui->customplot3->addGraph();
    pen.setColor(Qt::red);
    ui->customplot3->graph(0)->setPen(pen);
    pen.setColor(Qt::blue);
    ui->customplot3->addGraph();
    ui->customplot3->graph(1)->setPen(pen);
    ui->customplot3->graph(0)->setName("x");
    ui->customplot3->graph(1)->setName("y");
    for(int i=0;i<profile.size();i++){

        ui->customplot3->graph(0)->addData(i*0.02,profile[i].vel[0]);
        ui->customplot3->graph(1)->addData(i*0.02,profile[i].vel[1]);
    }

    ui->customplot4->addGraph();
    ui->customplot4->addGraph();
    pen.setColor(Qt::red);
    ui->customplot4->graph(0)->setPen(pen);
    pen.setColor(Qt::blue);
    ui->customplot4->graph(1)->setPen(pen);
    ui->customplot4->graph(0)->setName("x");
    ui->customplot4->graph(1)->setName("y");
    for(int i=0;i<profile.size();i++){
        ui->customplot4->graph(0)->addData(i*0.02,profile[i].acc[0]);
        ui->customplot4->graph(1)->addData(i*0.02,profile[i].acc[1]);
    }

    pen.setColor(Qt::blue);
    pen.setWidth(5);
    newCurve->setPen(pen);
    newCurve2->setPen(pen);
    newCurve3->setPen(pen);
    ui->customplot2->setInteractions(QCP::iRangeDrag);
    ui->customplot3->setInteractions(QCP::iRangeDrag);
    ui->customplot4->setInteractions(QCP::iRangeDrag);


    ui->customplot->xAxis->setLabel("x");
    ui->customplot2->xAxis->setLabel("time (s)");
    ui->customplot3->xAxis->setLabel("time (s)");
    ui->customplot4->xAxis->setLabel("time (s)");

    ui->customplot->yAxis->setLabel("y");
    ui->customplot2->yAxis->setLabel("position (m)");
    ui->customplot3->yAxis->setLabel("velocity (m/s)");
    ui->customplot4->yAxis->setLabel("acceleration (m/s/s)");


    ui->customplot2->xAxis->setRange(0,profile.size()*0.02);
    ui->customplot2->yAxis->setRange(-10,10);
    ui->customplot3->xAxis->setRange(0,profile.size()*0.02);
    ui->customplot3->yAxis->setRange(-10,10);
    ui->customplot4->xAxis->setRange(0,profile.size()*0.02);
    ui->customplot4->yAxis->setRange(-10,10);

    ui->customplot2->legend->setVisible(true);
    ui->customplot3->legend->setVisible(true);
    ui->customplot4->legend->setVisible(true);

    ui->customplot->replot();
    ui->customplot2->replot();
    ui->customplot3->replot();
    ui->customplot4->replot();

    connect(time,SIGNAL(timeout()),this,SLOT(update()));
    connect(ui->customplot, SIGNAL(mousePress(QMouseEvent*)),this,SLOT(mouseClick(QMouseEvent*)));
    connect(ui->customplot, SIGNAL(mouseRelease(QMouseEvent*)),this,SLOT(mouseClick(QMouseEvent*)));
    connect(ui->customplot, SIGNAL(mouseMove(QMouseEvent*)),this,SLOT(mouseClick(QMouseEvent*)));



//    Py_Initialize();

//    if (import_cvxopt() < 0) {
//     // fprintf(stderr, "error importing cvxopt");

//    }
//    //create solver object
//    PyObject *solvers = PyImport_ImportModule("cvxopt.solvers");

//    if (!solvers) {
//       // fprintf(stderr, "error importing cvxopt.solvers");
//    }

//    //linear programming
//     PyObject *lp = PyObject_GetAttrString(solvers, "qp");
//       if (!lp) {
//         fprintf(stderr, "error referencing cvxopt.solvers.lp");
//         Py_DECREF(solvers);

//       }

//    PyObject *G = (PyObject*)Matrix_New(1,3,DOUBLE);
//    PyObject *p = (PyObject*)Matrix_New(3,1,DOUBLE);
//    PyObject *Q = (PyObject*)Matrix_New(3,3,DOUBLE);
//    PyObject *h = (PyObject*)Matrix_New(1,1,DOUBLE);

//    PyObject *A = (PyObject*)Matrix_New(2,3,DOUBLE); //ok
//    PyObject *B = (PyObject*)Matrix_New(2,1,DOUBLE);  //ok


//    PyObject *pArgs = PyTuple_New(6);
//    if(!G || !Q || !pArgs){
//        fprintf(stderr , "error creating matrices");
//        Py_DECREF(solvers); Py_DECREF(lp);
//        Py_XDECREF(G); Py_XDECREF(Q); Py_XDECREF(h); Py_XDECREF(pArgs);
//    }


//    MAT_BUFD(G)[0] = 0.0;
//    MAT_BUFD(G)[1] = 0.0;
//    MAT_BUFD(G)[2] = 0.0;

//    MAT_BUFD(p)[0] = 0.0;
//    MAT_BUFD(p)[1] = 0.0;
//    MAT_BUFD(p)[2] = 0.0;

//    MAT_BUFD(Q)[0] =  1.0;
//    MAT_BUFD(Q)[1] =  1.3;
//    MAT_BUFD(Q)[2] =  0.0;
//    MAT_BUFD(Q)[3] =  1.3;
//    MAT_BUFD(Q)[4] =  2.0;
//    MAT_BUFD(Q)[5] =  0.0;
//    MAT_BUFD(Q)[6] =  0.0;
//    MAT_BUFD(Q)[7] =  0.0;
//    MAT_BUFD(Q)[8] =  2.0;

////    PyObject *x = PyDict_GetItemString(sol, "x");
////    std::cout<<   std::endl<<MAT_BUFD(x)[0]
////                <<std::endl<<MAT_BUFD(x)[1]
////                <<std::endl<<MAT_BUFD(x)[2]<<std::endl;


//    MAT_BUFD(h)[0] = 0.0;
//    MAT_BUFD(h)[1] = 0.0;

//    MAT_BUFD(A)[0] =  1.3;
//    MAT_BUFD(A)[1] =  1.5;
//    MAT_BUFD(A)[2] =  1.2;
//    MAT_BUFD(A)[3] =  2.6;
//    MAT_BUFD(A)[4] =  0.4;
//    MAT_BUFD(A)[5] =  -1.2;

//    MAT_BUFD(B)[0] = 1.6;
//    MAT_BUFD(B)[1] = 4.3;


//    PyTuple_SetItem(pArgs, 0, Q);
//    PyTuple_SetItem(pArgs, 1, p);
//    PyTuple_SetItem(pArgs, 2, G);
//    PyTuple_SetItem(pArgs, 3, h);
//    PyTuple_SetItem(pArgs, 4, A);
//    PyTuple_SetItem(pArgs, 5, B);



//    PyObject *sol = PyObject_CallObject(lp, pArgs);
//    if (!sol) {
//      PyErr_Print();
//      Py_DECREF(solvers); Py_DECREF(lp);
//      Py_DECREF(pArgs);
//    }

//        PyObject *x = PyDict_GetItemString(sol, "x");
//        std::cout<<   std::endl<<MAT_BUFD(x)[0]
//                    <<std::endl<<MAT_BUFD(x)[1]
//                    <<std::endl<<MAT_BUFD(x)[2]<<std::endl;



//        Py_DECREF(solvers);
//        Py_DECREF(lp);
//        Py_DECREF(pArgs);
//        Py_DECREF(sol);

//        Py_Finalize();




    //time->start(20);

}

void MainWindow::mouseClick(QMouseEvent *event){

    if(event->buttons() == Qt::LeftButton){
        force_gain = 1.0;
        mouse_corx = this->ui->customplot->xAxis->pixelToCoord(event->pos().x());
        mouse_cory = this->ui->customplot->yAxis->pixelToCoord(event->pos().y());
        item->start->setCoords( QPointF(px_2 , py_2));
        item->end->setCoords(mouse_corx,mouse_cory);
        forcex =force_gain*( mouse_corx - px_2) ;
        forcey =force_gain*( mouse_cory - py_2) ;
        std::cout << forcex  <<" : " <<forcey  <<std::endl;

        item->setHead(QCPLineEnding::esSpikeArrow);
        QPen pen;
        pen.setColor(Qt::green);
        pen.setWidth(10);
        item->setPen(pen);
        item->setVisible(true);
    }else{
        item->setVisible(false);
        forcex = 0;
        forcey = 0;
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_start_button_clicked()
{
    if(start){
        time->stop();
        start = false;
    }else{
        time->start(20);
        start = true;
    }
}
