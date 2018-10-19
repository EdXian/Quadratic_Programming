#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <iostream>
#include <fstream>
#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <qcustomplot.h>
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
#include "qptrajectory.h"
typedef CGAL::Gmpz ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif
#include <QMainWindow>
typedef CGAL::Quadratic_program<int> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QCPCurve *newCurve;// = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
    QVector<QCPCurveData> data;
    QCPCurve *newCurve2;
    QVector<QCPCurveData> data2;
    QVector<QCPCurveData> data3;
    QCPCurve *newCurve3;

    double ax,ay;
    double vx,vy;
    double px,py;
    qptrajectory *plan;
    std::vector<trajectory_profile> profile;
    double t;
    QTimer *time;


private:
    Ui::MainWindow *ui;

private slots:

    void update();

};

#endif // MAINWINDOW_H
