#include <iostream>
#include <cmath>
#include <array>
#include "TrajectoryGenerator.h"


/* 
 * The implementation of the function "generateSCurve" is referenced from:
 * https://github.com/alexbaucom17/DominoRobot/blob/master/src/robot/src/SmoothTrajectoryGenerator.cpp#L234
 */
bool TrajectoryGenerator::generateSCurve(float dist) {
    bool solution_found = false;
    // If distance to move is tiny, trajectory is all zeros.
    if (fabs(dist) <= solver.min_dist) {
        setSCurveParam2zero();
        solution_found = true;
        return solution_found;
    }

    // Initialize limits to max
    float v_lim = limits.max_vel;
    float a_lim = limits.max_acc;
    float j_lim = limits.max_jerk;

    // Start search loop
    int loop_counter = 0;
    float dt_j, dt_a, dt_v, dv_j, dp_j_nega, dp_j_posi, dp_a;
    while(solution_found==false && (loop_counter < solver.num_loops)) {
        loop_counter++;

        // Constant jerk region
        dt_j = a_lim / j_lim;
        dv_j = 0.5 * j_lim * std::pow(dt_j, 2);
        dp_j_posi = j_lim * std::pow(dt_j, 3) / 6;
        dp_j_nega = (v_lim - dv_j) * dt_j + 0.5 * a_lim * std::pow(dt_j, 2) - j_lim * std::pow(dt_j, 3)/6;

        // Constant accel region
        dt_a = (v_lim - 2 * dv_j) / a_lim;
        if (dt_a <= 0) {
            // If dt_a is negative, it means we couldn't find a solution
            // so adjust accel parameter and try loop again
            a_lim *= std::pow(solver.beta_decay, 1 + solver.exponent_decay * loop_counter);
            continue;
        }
        dp_a = dv_j * dt_a + 0.5 * a_lim * std::pow(dt_a, 2);

        // Constant velocity region
        dt_v = (dist - 2 * dp_j_nega - 2 * dp_j_posi - 2 * dp_a) / v_lim;
        if (dt_v <= 0) {
            // If dt_v is negative, it means we couldn't find a solution
            // so adjust velocity parameter and try loop again
            v_lim *= std::pow(solver.alpha_decay, 1 + solver.exponent_decay * loop_counter);
            continue;
        }

        // If we get here, it means we found a valid solution and can populate the rest of the 
        // switch time parameters
        solution_found = true;
        setSCurveParam(j_lim, a_lim, v_lim, dt_j, dt_a, dt_v);
    }
    return solution_found;
}

void TrajectoryGenerator::setSolverParameter(SolverParameter new_solver) {
    if ((new_solver.alpha_decay>=1 || 0>=new_solver.alpha_decay) || (new_solver.beta_decay>=1 || 0>=new_solver.beta_decay)) {
        std::cout<<"Setting fail, decay parameters not in range (0,1)"<<std::endl;
    }
    else {
        solver.alpha_decay = new_solver.alpha_decay;
        solver.beta_decay = new_solver.beta_decay;
        solver.exponent_decay = new_solver.exponent_decay;
        solver.min_dist = new_solver.min_dist;
        solver.num_loops = new_solver.num_loops;
    }
}

bool TrajectoryGenerator::getTrajectory(std::array<float, 5> &traj) {
    bool isdone = false;
    float jerk, acc, vel, pos;
    if (k == param.k_max) {
        k = 0;
        acc_now = 0;
        vel_now = 0;
        pos_now = 0;
        isdone = true;
        return isdone;
    }
    
    if (k < (size_t)(param.switch_time[0]/SAMPLE_TIME)) jerk = param.j_lim;
    else if (((size_t)roundf(param.switch_time[1]/SAMPLE_TIME) <= k) && (k < (size_t)roundf(param.switch_time[2]/SAMPLE_TIME))) jerk = -param.j_lim;
    else if (((size_t)roundf(param.switch_time[3]/SAMPLE_TIME) <= k) && (k < (size_t)roundf(param.switch_time[4]/SAMPLE_TIME))) jerk = -param.j_lim;
    else if (((size_t)roundf(param.switch_time[5]/SAMPLE_TIME) <= k) && (k < (size_t)roundf(param.switch_time[6]/SAMPLE_TIME))) jerk = param.j_lim;
    else jerk = 0;

    traj[0] = k*SAMPLE_TIME;
    traj[1] = jerk;
    traj[2] = acc_now;
    traj[3] = vel_now;
    traj[4] = pos_now;

    acc = acc_now + jerk*SAMPLE_TIME;
    vel = vel_now + acc_now*SAMPLE_TIME + 0.5*jerk*std::pow(SAMPLE_TIME, 2);
    pos = pos_now + vel_now*SAMPLE_TIME + 0.5*acc_now*std::pow(SAMPLE_TIME, 2) + jerk*std::pow(SAMPLE_TIME, 3)/6;
    acc_now = acc;
    vel_now = vel;
    pos_now = pos;
    
    k++;
    return isdone;
}

bool TrajectoryGenerator::getCircle(std::array<float, 3> &p_ref, std::array<float,3> &v_ref) {
    bool isdone = false;
    if (k == circle.k_max) {
        k = 0;
        isdone = true;
        return isdone;
    }
    float freq = 2*PI/circle.period;
    p_ref[0] = circle.radius*cos(freq*k*SAMPLE_TIME);
    p_ref[1] = circle.radius*sin(freq*k*SAMPLE_TIME);
    p_ref[2] = 0;
    v_ref[0] = -circle.radius*freq*sin(freq*k*SAMPLE_TIME);
    v_ref[1] = circle.radius*freq*cos(freq*k*SAMPLE_TIME);
    v_ref[2] = 0;

    k++;
    return isdone;
}
