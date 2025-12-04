#ifndef __TG_H
#define __TG_H

#include <cstddef>
#include <array>

#ifndef SAMPLE_TIME
#define SAMPLE_TIME 0.05f
#endif
#ifndef PI
#define PI 3.14159265358979323846
#endif


typedef struct {
    float max_jerk {2.5};
    float max_acc {1.5};
    float max_vel {0.25};
} Limits;

typedef struct {
    float min_dist {0.1};
    float num_loops {10};
    float beta_decay {0.8};
    float alpha_decay {0.8};
    float exponent_decay {0.5};
} SolverParameter;

typedef struct {
    float j_lim;
    float a_lim;
    float v_lim;
    float delta_t_j;
    float delta_t_a;
    float delta_t_v;
    float switch_time[7];
    size_t k_max;
} SCurveParameter;

typedef struct {
    float period {10};
    float radius {0.5f};
    size_t k_max {0};
} CircleParameter;



class TrajectoryGenerator {
    public:
        TrajectoryGenerator(): k(0), acc_now(0), vel_now(0), pos_now(0)
        {
            limits = Limits();
            solver = SolverParameter();
            circle = CircleParameter();
            circle.k_max = (size_t) roundf(circle.period/SAMPLE_TIME) + 1;
        }
        void setLimits(float jerk, float acc, float vel) {
            limits.max_jerk = jerk;
            limits.max_acc = acc;
            limits.max_vel = vel;
        }
        void setCircleParam(float period, float radius) {
            circle.period = period;
            circle.radius = radius;
            circle.k_max = (size_t) roundf(period/SAMPLE_TIME) + 1;
        }
        void setSolverParameter(SolverParameter new_solver);
        bool generateSCurve(float dist);
        bool getTrajectory(std::array<float, 5> &traj);
        bool getCircle(std::array<float, 3> &p_ref, std::array<float,3> &v_ref);
    private:
        Limits limits;
        SolverParameter solver;
        SCurveParameter param;
        size_t k;
        float acc_now;
        float vel_now;
        float pos_now;
        CircleParameter circle;
        void setSCurveParam2zero(void) {
            param.j_lim = 0;
            param.a_lim = 0;
            param.v_lim = 0;
            param.delta_t_j = 0;
            param.delta_t_a = 0;
            param.delta_t_v = 0;
            param.switch_time[0] = 0;
            param.switch_time[1] = 0;
            param.switch_time[2] = 0;
            param.switch_time[3] = 0;
            param.switch_time[4] = 0;
            param.switch_time[5] = 0;
            param.switch_time[6] = 0;
            param.k_max = 0;
        }
        void setSCurveParam(const float j_lim, const float a_lim, const float v_lim, const float dt_j, const float dt_a, const float dt_v) {
            param.j_lim = j_lim;
            param.a_lim = a_lim;
            param.v_lim = v_lim;
            param.delta_t_j = dt_j;
            param.delta_t_a = dt_a;
            param.delta_t_v = dt_v;
            param.switch_time[0] = param.delta_t_j;
            param.switch_time[1] = param.delta_t_j + param.delta_t_a;
            param.switch_time[2] = 2*param.delta_t_j + param.delta_t_a;
            param.switch_time[3] = 2*param.delta_t_j + param.delta_t_a +param.delta_t_v;
            param.switch_time[4] = 3*param.delta_t_j + param.delta_t_a +param.delta_t_v;
            param.switch_time[5] = 3*param.delta_t_j + 2*param.delta_t_a +param.delta_t_v;
            param.switch_time[6] = 4*param.delta_t_j + 2*param.delta_t_a +param.delta_t_v;
            param.k_max = (size_t) roundf(param.switch_time[6]/SAMPLE_TIME)+1;
        }     
};



#endif // __TG_H
