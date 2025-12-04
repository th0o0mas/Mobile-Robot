#include <csignal>
#include <sys/time.h>
#include <cmath>
#include "MecanumWMR.h"

MecanumWMR* MecanumWMR::instance = nullptr;

bool MecanumWMR::init() {
    if (!i2c.init()) { 
        std::cout<<"pigpiod error"<<std::endl;
        return false;
    }
    std::cout<<"Robot setup start"<<std::endl;
    if (!IMU.setup(MPU9250_ADDRESS, i2c)) {
        std::cout<<"MPU9250 setup error"<<std::endl;
        return false;
    }
    for (short i=0;i<20;i++) {
        IMU.update();
        IMU.update2();
        i2c.delay(50);
    }
    std::cout<<"Robot setup done"<<std::endl;
    if (log) {
        if ((p_fptr=fopen("pos_performance.txt", "w")) == NULL) return false;
        if ((v_fptr=fopen("vel_performance.txt", "w")) == NULL) return false;
        fprintf(p_fptr, "x_est y_est heading_est x_ref y_ref heading_ref magdwick gyro mag_x mag_y mag_z\n");
        fprintf(v_fptr, "1_est 2_est 3_est 4_est 1_ref 2_ref 3_ref 4_ref\n");
    }

    return true;
}

void MecanumWMR::shutdown() {
    i2c.shutdown();
    if (log) {
        fclose(p_fptr);
        fclose(v_fptr);
    }
    std::cout<<"Shutdown finish"<<std::endl;
}

void MecanumWMR::setInitCond(const float x_0, const float y_0, const float theta_0) {
    p_est[0] = x_0;
    p_est[1] = y_0;
    init_angle = theta_0;
}

void MecanumWMR::setPIDgain_translation(const float kp, const float ki, const float kd, const float kb) {
    ControllerParam.kp[0] = kp;
    ControllerParam.ki[0] = ki;
    ControllerParam.kd[0] = kd;
    ControllerParam.kb[0] = kb;
    ControllerParam.kp[1] = kp;
    ControllerParam.ki[1] = ki;
    ControllerParam.kd[1] = kd;
    ControllerParam.kb[1] = kb;
}

void MecanumWMR::setPIDgain_rotation(const float kp, const float ki, const float kd, const float kb) {
    ControllerParam.kp[2] = kp;
    ControllerParam.ki[2] = ki;
    ControllerParam.kd[2] = kd;
    ControllerParam.kb[2] = kb;
}

void MecanumWMR::setVelocityLimit(const float linear_v_max, const float angular_v_max) {
    ControllerParam.max_vel[0] = linear_v_max;
    ControllerParam.max_vel[1] = linear_v_max;
    ControllerParam.max_vel[2] = angular_v_max;
}

std::array<float, 3> MecanumWMR::getPosition(void) const {return p_est;}

bool MecanumWMR::Point2PointMove(const std::array<float, 3> target_point) {
    starting_point = p_est;
    float dist = sqrt(std::pow(target_point[0]-starting_point[0], 2) + std::pow(target_point[1]-starting_point[1], 2));
    target_angle = atan2f(target_point[1]-starting_point[1], target_point[0]-starting_point[0]);
    float angle_dist = fabs(target_point[2]-starting_point[2]);
    angle_direction = (target_point[2]-starting_point[2])<0.f?-1:1;
    if (!translation_tg.generateSCurve(dist)) {
        std::cout<<"Translation S-curve generation fail"<<std::endl;
        return false;
    }
    if (!rotation_tg.generateSCurve(angle_dist)) {
        std::cout<<"Rotation S-curve generation fail"<<std::endl;
        return false;
    }
	i2c.EnableSTM32();
	timer_start();
	while(!isDone)
	{
        
	}
	timer_stop();
	i2c.DisableSTM32();
    isDone = false;
    return true;
}

void MecanumWMR::localization(void) {
	float v_est_local[3] = {0.0}, v_est_global[3] = {0.0};
    IMU.update();
    IMU.update2();
    // p_est[2] = IMU.getYaw()*PI/180 + init_angle;  // Madgwick
    p_est[2] = IMU.getyaw()*PI/180 + init_angle; // pure integral
	ForwardKinematicTrans(v_est_local, wheel_speed_est.value);
	VelRobot2World(v_est_local, v_est_global);
	for (short i=0;i<2;i++) p_est[i] += v_est_global[i]*SAMPLE_TIME;
}

void MecanumWMR::pos_controller(const std::array<float, 3> p_ref, const std::array<float, 3> v_ref) {
    static float sum_error[3]={0.0};
    static float prev_error[3]={0.0};
    float error, derivative, u;
    float v_global[3] = {0.0}, v_local[3]={0.0};
    for (int i=0;i<3;i++) {
        error = p_ref[i] - p_est[i];
        derivative = (error - prev_error[i]) / SAMPLE_TIME;
        v_global[i] = v_ref[i] + ControllerParam.kp[i]*error + ControllerParam.ki[i]*sum_error[i] + ControllerParam.kd[i]*derivative;
        u = sat(v_global[i], ControllerParam.max_vel[i], ControllerParam.max_vel[i]);
        sum_error[i] += (error + ControllerParam.kb[i]*(u-v_global[i])) * SAMPLE_TIME;
        v_global[i] = u;
        prev_error[i] = error;
    }
    VelWorld2Robot(v_local, v_global);
    InverseKinematicTrans(v_local, wheel_speed_ref.value);
}


float MecanumWMR::sat(float s,const float width, const float height) {
	float s_out=0.0f;
	if ( s > width) s_out = height;
	else if ( s < -width) s_out = -height;
	else s_out = s*(height/width);
	return s_out;
}

void MecanumWMR::ForwardKinematicTrans(float *const linear_v, const float *const wheel_v) {
	linear_v[0] = WHEEL_R/WHEEL_NUM*(wheel_v[0]+wheel_v[1]+wheel_v[2]+wheel_v[3]);
	linear_v[1] = WHEEL_R/WHEEL_NUM*(-wheel_v[0]+wheel_v[1]+wheel_v[2]-wheel_v[3]);
	linear_v[2] = WHEEL_R/2/L*(-wheel_v[0]+wheel_v[1]-wheel_v[2]+wheel_v[3]);
}

void MecanumWMR::InverseKinematicTrans(const float *const linear_v, float *const wheel_v) {
	wheel_v[0] = (linear_v[0]-linear_v[1]-L/2*linear_v[2])/WHEEL_R;
	wheel_v[1] = (linear_v[0]+linear_v[1]+L/2*linear_v[2])/WHEEL_R;
	wheel_v[2] = (linear_v[0]+linear_v[1]-L/2*linear_v[2])/WHEEL_R;
	wheel_v[3] = (linear_v[0]-linear_v[1]+L/2*linear_v[2])/WHEEL_R;
}

void MecanumWMR::VelRobot2World(const float *const v_local, float *const v_global) {
	v_global[0] = v_local[0]*cos(p_est[2])-v_local[1]*sin(p_est[2]);
	v_global[1] = v_local[0]*sin(p_est[2])+v_local[1]*cos(p_est[2]);
	v_global[2] = v_local[2];
}

void MecanumWMR::VelWorld2Robot(float *const v_local, const float *const v_global) {
	v_local[0] = v_global[0]*cos(p_est[2])+v_global[1]*sin(p_est[2]);
	v_local[1] = -v_global[0]*sin(p_est[2])+v_global[1]*cos(p_est[2]);
	v_local[2] = v_global[2];
}

void MecanumWMR::timer_start(void) {
    struct itimerval timer;
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = (int) (SAMPLE_TIME*std::pow(10,6)); //unit is microsecond i.e. 10^-6s
	timer.it_value = timer.it_interval;
	setitimer(ITIMER_REAL, &timer, nullptr);
}
void MecanumWMR::timer_stop(void) {
    struct itimerval timer;
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = 0;
	timer.it_value = timer.it_interval;
	setitimer(ITIMER_REAL, &timer, nullptr);
}

void MecanumWMR::timer_callback(void) {
    std::array<float, 5> translation_traj = {0.f};
    std::array<float, 5> rotation_traj = {0.f};
    std::array<float, 3> p_ref = {0.f};
    std::array<float, 3> v_ref = {0.f};
    if (!translation_isDone) {
        translation_isDone = translation_tg.getTrajectory(translation_traj);
    }
    if (!rotation_isDone) {
        rotation_isDone = rotation_tg.getTrajectory(rotation_traj);
    }
    isDone = translation_isDone && rotation_isDone;
    if (!isDone) {
        if (!translation_isDone) {
            v_ref[0] = translation_traj[3]*cos(target_angle);
            v_ref[1] = translation_traj[3]*sin(target_angle);
            p_ref[0] = starting_point[0] + translation_traj[4]*cos(target_angle);
            p_ref[1] = starting_point[1] + translation_traj[4]*sin(target_angle);
        }
        else {
            v_ref[0] = 0.f;
            v_ref[1] = 0.f;
            p_ref[0] = p_est[0];
            p_ref[1] = p_est[1];
        }
        if (!rotation_isDone) {
            v_ref[2] = angle_direction*rotation_traj[3];
            p_ref[2] = starting_point[2] + angle_direction*rotation_traj[4];
        }
        else {
            v_ref[2] = 0.f;
            p_ref[2] = p_est[2];
        }
        i2c.ReadSTM32(wheel_speed_est.bytes);
        localization();
        auto g = IMU.getGyroZ();
        auto mx = IMU.getMagX();
        auto my = IMU.getMagY();
        auto mz = IMU.getMagZ();
        auto magdwick_yaw = IMU.getYaw()*PI/180;
        pos_controller(p_ref, v_ref);
        i2c.SendSTM32(wheel_speed_ref.bytes);
        if (log) Log2File(p_ref,g,mx,my,mz,magdwick_yaw);
    }
    else {
        translation_isDone = false;
        rotation_isDone = false;
    }
}

void MecanumWMR::Log2File(const std::array<float, 3> p_ref, float g, float mx, float my, float mz, float yaw) {
    for (short i=0;i<3;i++) fprintf(p_fptr, "%.3f ", p_est[i]);
    for (short i=0;i<3;i++) {
        if (i<2) fprintf(p_fptr, "%.3f ", p_ref[i]);
        else fprintf(p_fptr, "%.3f %.3f %.3f %.3f %.3f %.3f\n", p_ref[i], yaw, g, mx, my, mz);
    }
    for (short i=0;i<4;i++) fprintf(v_fptr, "%.3f ", wheel_speed_est.value[i]);
    for (short i=0;i<4;i++) {
	    if (i<3) fprintf(v_fptr, "%.3f ", wheel_speed_ref.value[i]);
	    else fprintf(v_fptr, "%.3f\n", wheel_speed_ref.value[i]);
	}
}
