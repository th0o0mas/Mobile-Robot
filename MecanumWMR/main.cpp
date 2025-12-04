#include <iostream>
#include <csignal>
#include "MecanumWMR.h"

/*volatile sig_atomic_t stop = 0;

void handle_sigint(int signum) {
    stop = 1;
}*/

int main(int argc, char** argv) {
    //signal(SIGINT, handle_sigint);
    MecanumWMR wmr;
    if (wmr.init()==false) return 1;
    wmr.setInitCond(0,0,0);
    std::array<float, 3> target_point = {0.0};
    for (int i=0;i<3;i++) {
        std::cout<<"Please enter a target point [x, y, theta]: ";
        std::cin>>target_point[0]>>target_point[1]>>target_point[2];
        wmr.Point2PointMove(target_point);
        auto p_est = wmr.getPosition();
        std::cout<<"current point [x, y, theta]: "<<p_est[0]<<","<<p_est[1]<<","<<p_est[2]<<std::endl;
    }
    wmr.shutdown();

	return 0;
}


