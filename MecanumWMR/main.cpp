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
    wmr.setPIDgain_translation(1.5, 0.25, 0.05, 0.25);
    ControllerParameter param = wmr.getControllerParam();
    std::array<float, 3> target_point = {0.0};
    for (int i=0;i<3;i++) {
        std::cout << "kp=" << param.kp[0] << ", ki=" << param.ki[0] << ", kd=" << param.kd[0] << ", kb=" << param.kb[0] << std::endl;
        std::cout<<"Please enter a target point [x, y, theta]: ";
        std::cin>>target_point[0]>>target_point[1]>>target_point[2];
        wmr.Point2PointMove(target_point);
        auto p_est = wmr.getPosition();
        std::cout<<"current point [x, y, theta]: "<<p_est[0]<<","<<p_est[1]<<","<<p_est[2]<<std::endl;
        std::cout << "Would you  like to change the gains (y/n)? " << std::endl;
        char choice;
        std::cin >> choice;
        if (choice == 'y' || choice == 'Y') {
            float kp, ki, kd, kb;
            std::cout << "Which gain would you like to change (0 = translation, 1 = angular rotation )? "<< std::endl;
            int axis;
            std::cin >> axis;
            std::cout << "Enter new gains (kp ki kd kb): ";
            if (axis == 0) {
                std::cin >> kp >> ki >> kd >> kb;
                wmr.setPIDgain_translation(kp, ki, kd, kb);
            }
            else
            {
                std::cin >> kp >> ki >> kd >> kb;
                wmr.setPIDgain_rotation(kp, ki, kd, kb);
            }
        }
        
    
    }

    wmr.shutdown();

	return 0;
}


