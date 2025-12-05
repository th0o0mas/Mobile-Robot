#include <iostream>
#include <csignal>
#include <sstream>
#include <limits>
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
    wmr.setPIDgain_translation(0.9, 1.2, 0, 0.25);
    
    std::array<float, 3> target_point = {0.0};
    // Ask user how many points they want to reach
    int num_points;
    std::cout << "How many target points would you like to reach? ";
    std::cin >> num_points;
    if (num_points == -1) {
        ControllerParameter param = wmr.getControllerParam();
        std::cout << "Test Mode activated"  << std::endl;
        std::cout << "Current Translation Gains: kp=" << param.kp[0] << ", ki=" << param.ki[0] 
                  << ", kd=" << param.kd[0] << ", kb=" << param.kb[0] << std::endl;
        std::cout << "Current Rotation Gains: kp=" << param.kp[1] << ", ki=" << param.ki[1] 
                  << ", kd=" << param.kd[1] << ", kb=" << param.kb[1] << std::endl;
        
        std::cout << "\n=== Target Point " << 1 << " ===" << std::endl;
        auto target_point = std::array<float, 3>{1.0, 0.0, 0.0};
        std::cout << "Moving to target [x y theta]: " << target_point[0] << " " << target_point[1] << " " << target_point[2] << std::endl;
        wmr.Point2PointMove(target_point);
        auto p_est = wmr.getPosition();
        std::cout << "Reached position [x y theta]: " << p_est[0] << " " << p_est[1] << " " << p_est[2] << std::endl;
        
        std::cout << "\n=== Target Point " << 2 << " ===" << std::endl;
        target_point = {0.0, 0.0, 0.0};
        std::cout << "Moving to target [x y theta]: " << target_point[0] << " " << target_point[1] << " " << target_point[2] << std::endl;
        wmr.Point2PointMove(target_point);
        p_est = wmr.getPosition();
        std::cout << "Reached position [x y theta]: " << p_est[0] << " " << p_est[1] << " " << p_est[2] << std::endl;

        std::cout << "\n=== Target Point " << 3 << " ===" << std::endl;
        target_point ={2.0, 0.0, 0.0};
        std::cout << "Moving to target [x y theta]: " << target_point[0] << " " << target_point[1] << " " << target_point[2] << std::endl;
        wmr.Point2PointMove(target_point);
        p_est = wmr.getPosition();
        std::cout << "Reached position [x y theta]: " << p_est[0] << " " << p_est[1] << " " << p_est[2] << std::endl;
        num_points = 0; // Skip the main loop
    }
    
    // Main loop for reaching target points
    for (int i = 0; i < num_points; i++) 
    {
        ControllerParameter param = wmr.getControllerParam();
        std::cout << "Would you like to modify PID gains? (y/n): ";
        std::cout << "Would you like to modify PID gains? (y/n): "<< std::endl;
        std::cout << "Current Translation Gains: kp=" << param.kp[0] << ", ki=" << param.ki[0] 
                  << ", kd=" << param.kd[0] << ", kb=" << param.kb[0] << std::endl;
        std::cout << "Current Rotation Gains: kp=" << param.kp[1] << ", ki=" << param.ki[1] 
                  << ", kd=" << param.kd[1] << ", kb=" << param.kb[1] << std::endl;
        char modify_gains;
        std::cin >> modify_gains;
        if (modify_gains == 'y' || modify_gains == 'Y') 
        {
            float kp, ki, kd, kb;
            
            // Translation gains
            std::cout << "\nWhich gains would you like to modify?" << std::endl;
            std::cout << "1. Translation only" << std::endl;
            std::cout << "2. Rotation only" << std::endl;
            std::cout << "3. Both" << std::endl;
            std::cout << "Enter choice (1/2/3): ";
            int gain_choice;
            std::cin >> gain_choice;
            
            if (gain_choice == 1 || gain_choice == 3) {
                // Translation gains
                std::cout << "\n=== Translation PID Gains ===" << std::endl;
                std::cout << "Current: kp=" << param.kp[0] << ", ki=" << param.ki[0] 
                        << ", kd=" << param.kd[0] << ", kb=" << param.kb[0] << std::endl;
                std::cout << "Enter new values (kp ki kd kb) - leave empty to keep current: ";
                
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::string line;
                std::getline(std::cin, line);
                std::istringstream iss(line); // Use istringstream to parse input
                

                float kp = param.kp[0], ki = param.ki[0], kd = param.kd[0], kb = param.kb[0];
                if (iss >> kp) iss >> ki >> kd >> kb; // Read new values if provided

                wmr.setPIDgain_translation(kp, ki, kd, kb);
            }
            
            if (gain_choice == 2 || gain_choice == 3) {
                // Rotation gains
                std::cout << "\n=== Rotation PID Gains ===" << std::endl;
                std::cout << "Current: kp=" << param.kp[1] << ", ki=" << param.ki[1] 
                        << ", kd=" << param.kd[1] << ", kb=" << param.kb[1] << std::endl;
                std::cout << "Enter new values (kp ki kd kb) - leave empty to keep current: ";
                
                std::string line;
                std::getline(std::cin, line);
                std::istringstream iss(line); // Use istringstream to parse input
                
                float kp = param.kp[1], ki = param.ki[1], kd = param.kd[1], kb = param.kb[1];
                if (iss >> kp) iss >> ki >> kd >> kb;
                
                wmr.setPIDgain_rotation(kp, ki, kd, kb);
            }
        }


        std::cout << "\n=== Target Point " << (i + 1) << "/" << num_points << " ===" << std::endl;
        std::cout << "Enter target [x y theta]: ";
        std::cin >> target_point[0] >> target_point[1] >> target_point[2];
        
        wmr.Point2PointMove(target_point);
        
        auto p_est = wmr.getPosition();
        std::cout << "Reached position [x y theta]: " << p_est[0] << " " << p_est[1] << " " << p_est[2] << std::endl;
        
    
    }

    wmr.shutdown();

	return 0;
}


