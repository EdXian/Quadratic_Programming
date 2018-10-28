#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <qptrajectory.h>
geometry_msgs::Point pos;
geometry_msgs::Point vel;
geometry_msgs::Point acc;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qptest");
  ros::NodeHandle nh;

  ros::Publisher pos_pub = nh.advertise<geometry_msgs::Point>("/pos",10);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Point>("/vel",10);
  ros::Publisher acc_pub = nh.advertise<geometry_msgs::Point>("/acc",10);
  std::cout << "Trajectory generator"<<std::endl;
  double max;
  double count ;
  ros::Rate loop_rate(100);

  qptrajectory plan;
  path_def path;
  trajectory_profile p1 , p2, p3;
  std::vector<trajectory_profile> data;
  p1.pos << 1.0,0,0;
  p1.vel << 0.0,0.0,0;
  p1.acc << 0.00,-0.0,0;

  p2.pos<< 0.0,3.0,0;
  p2.vel<< -0.3,-0.20,0;
  p2.acc<< -0.1,-0.3,0;

  p3.pos<< -3.0,-1.0,0;
  p3.vel<< 0.3,-0.1,0;
  p3.acc<< 0.3,0.2,0;


  path.push_back(segments(p1,p2,1.5));
  path.push_back(segments(p2,p3,1.5));
  path.push_back(segments(p3,p1,1.5));

  data = plan.get_profile(path , 0,0.01);
  max = data.size();
  for(int i =0; i<data.size();i++){
    std::cout << "===pos==="<<i<<"========="<<std::endl;
    std::cout << data[i].pos.transpose()<<std::endl;
  }

//  for(int i =0; i<data.size();i++){
//    std::cout << "===vel==="<<i<<"========="<<std::endl;
//    std::cout << data[i].vel.transpose()<<std::endl;
//  }

//  for(int i =0; i<data.size();i++){
//    std::cout << "===acc==="<<i<<"========="<<std::endl;
//    std::cout << data[i].acc.transpose()<<std::endl;
//  }

  for(int i=0 ;i<100;i++){
      loop_rate.sleep();
  }
  while(ros::ok()){


    if(count >=max){
      count = 0;
    }else{
      pos.x = data[count].pos[0];
      pos.y = data[count].pos[1];
      pos.z = data[count].pos[2];

      vel.x = data[count].vel[0];
      vel.y = data[count].vel[1];
      vel.z = data[count].vel[2];

      acc.x = data[count].acc[0];
      acc.y = data[count].acc[1];
      acc.z = data[count].acc[2];

    }


    //pos_pub.publish
    acc_pub.publish(acc);
    vel_pub.publish(vel);
    pos_pub.publish(pos);

    count++;
    ros::spinOnce();
    loop_rate.sleep();
  }

}
