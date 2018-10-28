#include "qptrajectory.h"
#include <cvxopt/cvxopt.h>
qptrajectory::qptrajectory(){




}
qptrajectory::~qptrajectory(){

}



std::vector<double>
qptrajectory::qpsovle(profile begin, profile end, double time_interval){
//Program solver(CGAL::EQUAL, false, 0, false, 0);
//CGAL::Quadratic_program_options options;
//options.set_pricing_strategy(CGAL::QP_BLAND);     // Bland's rule



std::vector<double> polynomial;
polynomial.clear();
double t =time_interval ;
double b0 = 24,
       b1 = 120,
       b2 = 360,
       b3 = 840;

Eigen::MatrixXd D(8,8);
Eigen::MatrixXd d(4,4);
Eigen::MatrixXd A(8,8);
Eigen::MatrixXd B(8,1);
//std::cout << "here" <<std::endl;

D.setZero();
d.setZero();
A.setZero();
B.setZero();



double d11 = b0*b0*t        , d12 = b0*b1*t*t        , d13 = b0*b2*t*t*t         , d14 = b0*b3*t*t*t*t ;
double d21 = b0*b1*t*t      , d22 = b1*b1*t*t*t      , d23 = b1*b2*t*t*t*t       , d24 = b1*b3*t*t*t*t*t ;
double d31 = b0*b2*t*t*t    , d32 = b2*b1*t*t*t*t    , d33 = b2*b2*t*t*t*t*t     , d34 = b2*b3*t*t*t*t*t*t;
double d41 = b3*b0*t*t*t*t  , d42 = b3*b1*t*t*t*t*t  , d43 = b3*b2*t*t*t*t*t*t   , d44 = b3*b3*t*t*t*t*t*t*t ;

Py_Initialize();
if (import_cvxopt() < 0) {
 // fprintf(stderr, "error importing cvxopt");

}
//create solver object
PyObject *solvers = PyImport_ImportModule("cvxopt.solvers");

if (!solvers) {
   // fprintf(stderr, "error importing cvxopt.solvers");
}
PyObject *lp = PyObject_GetAttrString(solvers, "qp");



  if (!lp) {
    fprintf(stderr, "error referencing cvxopt.solvers.lp");
    Py_DECREF(solvers);

  }


  PyObject *G = (PyObject*)Matrix_New(1,8,DOUBLE);
  PyObject *p = (PyObject*)Matrix_New(8,1,DOUBLE);
  PyObject *Q = (PyObject*)Matrix_New(8,8,DOUBLE);
  PyObject *h = (PyObject*)Matrix_New(1,1,DOUBLE);

  PyObject *A_ = (PyObject*)Matrix_New(8,8,DOUBLE); //ok
  PyObject *B_ = (PyObject*)Matrix_New(8,1,DOUBLE);  //ok


  PyObject *pArgs = PyTuple_New(6);
  if(!G || !Q || !pArgs){
      fprintf(stderr , "error creating matrices");
      Py_DECREF(solvers); Py_DECREF(lp);
      Py_XDECREF(G); Py_XDECREF(Q); Py_XDECREF(h); Py_XDECREF(pArgs);
  }


d << (1/1.0)*d11*1.0 , (1/2.0)*d21*1.0 ,(1/3.0)*d31*1.0  ,      (1/4.0)*d41*1.0     ,
     (1/2.0)*d21*1.0 , (1/3.0)*d22*1.0 ,(1/4.0)*d32*1.0  ,      (1/5.0)*d42*1.0     ,
     (1/3.0)*d31*1.0 , (1/4.0)*d32*1.0 , (1/5.0)*d33*1.0 ,     (1/6.0)*d43*1.0    ,
     (1/4.0)*d41*1.0 , (1/5.0)*d42*1.0 , (1/6.0)*d43*1.0 , (1/7.0)*d44*1.0 ;

    D.block<4,4>(4,4) = d;



     A<< 1 ,  0  ,   0  ,  0  , 0 , 0 , 0  , 0 ,
         0 ,  1  ,   0  ,  0  , 0 , 0 , 0  , 0 ,
         0 ,  0  ,   2  ,  0  , 0 , 0 , 0  , 0 ,
         0 ,  0  ,   0  ,  6  , 0 , 0 , 0  , 0 ,
         1 , 1*t , 1*t*t , 1*t*t*t , 1*t*t*t*t , 1*t*t*t*t*t , 1*t*t*t*t*t*t , 1*t*t*t*t*t*t*t,
         0 ,  1  ,   2*t , 3*t*t   , 4*t*t*t   , 5*t*t*t*t   , 6*t*t*t*t*t   , 7*t*t*t*t*t*t  ,
         0 ,  0  ,    2 ,    6*t    ,12*t*t    , 20*t*t*t     ,30*t*t*t*t    , 42*t*t*t*t*t   ,
         0 ,  0  ,    0  ,    6     ,24*t      , 60*t*t       ,120*t*t*t     , 210*t*t*t*t    ;

//     B <<  begin.pos, begin.vel , begin.acc  , begin.jerk ,
//             end.pos,  end.vel  , end.acc    , end.jerk ;
        B<< begin.V , end.V;


     for(int i=0 ; i<8; i++){
         for(int j=0;j<(i+1);j++){
             MAT_BUFD(Q)[i*8+j] = D(j,i)   ;
             //solver.set_d(i, j, D(i,j));

         }
     }
     for(int i=0;i<8;i++){
         for(int j=0;j<8;j++){
             MAT_BUFD(A_)[i*8+j] = A(j,i)   ;
         }
     }
     for(int i=0;i<8;i++){
            //solver.set_b(i,B(i,0));
           MAT_BUFD(B_)[i] = B(i,0)   ;

     }


     PyTuple_SetItem(pArgs, 0, Q);
     PyTuple_SetItem(pArgs, 1, p);
     PyTuple_SetItem(pArgs, 2, G);
     PyTuple_SetItem(pArgs, 3, h);
     PyTuple_SetItem(pArgs, 4, A_);
     PyTuple_SetItem(pArgs, 5, B_);


     PyObject *sol = PyObject_CallObject(lp, pArgs);

     if (!sol) {
       PyErr_Print();
       Py_DECREF(solvers); Py_DECREF(lp);
       Py_DECREF(pArgs);
     }

       PyObject *x = PyDict_GetItemString(sol, "x");

       for(int i=0;i<8;i++){
          polynomial.push_back(MAT_BUFD(x)[i]);

       }
//         std::cout<<   std::endl<<MAT_BUFD(x)[0]
//                     <<std::endl<<MAT_BUFD(x)[1]
//                     <<std::endl<<MAT_BUFD(x)[2]
//                     <<std::endl<<MAT_BUFD(x)[3]<<std::endl;

         Py_DECREF(solvers);
         Py_DECREF(lp);
         Py_DECREF(pArgs);
         Py_DECREF(sol);
         Py_Finalize();

//    Solution s =CGAL::solve_quadratic_program(solver, ET() );
//    assert (s.is_valid());


//    for (Solution::Variable_value_iterator it_ =  s.variable_values_begin();
//         it_ != s.variable_values_end();
//         ++it_) {
//        CGAL::Quotient<ET> data = *it_;

//      polynomial.push_back(data.numerator().to_double()/data.denominator().to_double());


//    }

    return polynomial;
}



std::vector<trajectory_profile> qptrajectory::get_profile(std::vector<segments> seg , double time_interval , double dt ){
    double t=0.0 ;
    std::vector<double> polyx , polyy;
    std::vector<trajectory_profile> tprofile;
    tprofile.clear();
    Eigen::Vector3d d(0,0,0);
    trajectory_profile data(d ,d,d ,0.01);

    for(int i=0 ; i < seg.size() ; i++){

        profile begin ,end;

        begin.V<< seg[i].b_c.pos[0] , seg[i].b_c.vel[0] , seg[i].b_c.acc[0] , 0;
        end.V  << seg[i].t_c.pos[0] , seg[i].t_c.vel[0]   , seg[i].t_c.acc[0] , 0;

        polyx  = qpsovle(begin , end , seg[i].time_interval);

        begin.V.setZero();
        end.V.setZero();
        begin.V<< seg[i].b_c.pos[1] , seg[i].b_c.vel[1] , seg[i].b_c.acc[1] , 0;
        end.V<< seg[i].t_c.pos[1] , seg[i].t_c.vel[1] , seg[i].t_c.acc[1] , 0;
        polyy =   qpsovle(begin , end , seg[i].time_interval);
        t=0;

        for(int j=0;j<(seg[i].time_interval/dt);j++){
            t =(double) dt*j;
            data.pos << polynomial(polyx , t) ,polynomial(polyy , t) , 0;
            data.vel << polynomial_d1(polyx,t),polynomial_d1(polyy , t) ,0;
            data.acc << polynomial_d2(polyx,t) , polynomial_d2(polyy,t) , 0;
            tprofile.push_back(data);
        }

    }

return tprofile;
}


double qptrajectory::polynomial(std::vector<double> data ,double t){
    double sum =0.0 , var =1;
    for(int i =0 ; i<data.size();i++){
        sum+= data[i]*var;
        var*=t;
    }
    return sum;
}
double cpow(double t,int times){
    double var =1.0;
    for(int i=0;i<times ;i++){
        var*=t;
    }
    return var;
}

double qptrajectory::polynomial_d1(std::vector<double> data ,double t){
    double sum=0.0;
    for(int i=1;i<data.size();i++){
        sum+=  (double) data[i] * i * cpow( t , i-1);
    }
    return sum;
}
double qptrajectory::polynomial_d2(std::vector<double> data ,double t){
    double sum =0.0 , var =1.0;
    for(int i =2 ; i<data.size();i++){
        sum += data[i] * i *(i-1)* cpow( t , i-2);
    }
    return sum;
}
double qptrajectory::polynomial_d3(std::vector<double> data ,double t){
    double sum =0.0 , var =1.0;
    for(int i = 3 ; i<data.size();i++){
        sum += data[i] * i *(i-1)* (i-2) * cpow( t , i-3);
    }
    return sum;
}

