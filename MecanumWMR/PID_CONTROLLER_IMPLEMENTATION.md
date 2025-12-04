# PID Controller Implementation for Mecanum WMR

## Overview
The high-level position controller has been upgraded from a **PI (Proportional-Integral) controller** to a full **PID (Proportional-Integral-Derivative) controller**. This enhancement provides better transient response, reduced overshoot, and improved tracking performance.

## PID Control Law

The standard PID control law implemented is:

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

In discrete form with anti-windup:

$$u[k] = v_{ref}[k] + K_p e[k] + K_i \sum e[k] + K_d \frac{e[k] - e[k-1]}{T_s}$$

$$\sum e[k] = \sum e[k-1] + (e[k] + K_b(u_{sat}[k] - u[k])) \cdot T_s$$

Where:
- **$K_p$** = Proportional gain (immediate response to error)
- **$K_i$** = Integral gain (eliminates steady-state error)
- **$K_d$** = Derivative gain (anticipatory action, reduces overshoot)
- **$K_b$** = Back-calculation gain (anti-windup mechanism)
- **$T_s$** = Sample time (0.05 seconds)
- **$e[k]$** = Tracking error at step k
- **$u_{sat}$** = Saturated velocity command

## Implementation Details

### 1. **Header File Updates** (`MecanumWMR.h`)

#### Default Gain Initialization:
```cpp
typedef struct {
    float kp[3] {1,1,1};      // Proportional gains for [vx, vy, omega]
    float ki[3] {0.25,0.25,0.25};   // Integral gains
    float kb[3] {0.25,0.25,0.25};   // Anti-windup gains
    float kd[3] {0.05,0.05,0.05};   // Derivative gains (NEW)
    float max_vel[3] {0.6,0.6,0.8}; // Velocity limits
} ControllerParameter;
```

#### Function Signatures:
```cpp
// Updated to include kd parameter
void setPIDgain_translation(const float kp, const float ki, const float kd, const float kb);
void setPIDgain_rotation(const float kp, const float ki, const float kd, const float kb);
```

### 2. **Implementation Updates** (`MecanumWMR.cpp`)

#### Setter Functions:
```cpp
void MecanumWMR::setPIDgain_translation(const float kp, const float ki, const float kd, const float kb) {
    ControllerParam.kp[0] = kp;
    ControllerParam.ki[0] = ki;
    ControllerParam.kd[0] = kd;  // NEW
    ControllerParam.kb[0] = kb;
    ControllerParam.kp[1] = kp;
    ControllerParam.ki[1] = ki;
    ControllerParam.kd[1] = kd;  // NEW
    ControllerParam.kb[1] = kb;
}

void MecanumWMR::setPIDgain_rotation(const float kp, const float ki, const float kd, const float kb) {
    ControllerParam.kp[2] = kp;
    ControllerParam.ki[2] = ki;
    ControllerParam.kd[2] = kd;  // NEW
    ControllerParam.kb[2] = kb;
}
```

#### Core PID Controller (`pos_controller` function):
```cpp
void MecanumWMR::pos_controller(const std::array<float, 3> p_ref, const std::array<float, 3> v_ref) {
    static float sum_error[3]={0.0};        // Integral term accumulator
    static float prev_error[3]={0.0};       // Previous error (NEW)
    float error, derivative, u;
    float v_global[3] = {0.0}, v_local[3]={0.0};
    
    for (int i=0;i<3;i++) {
        // Error calculation
        error = p_ref[i] - p_est[i];
        
        // Derivative term: D = (error - prev_error) / Ts
        derivative = (error - prev_error[i]) / SAMPLE_TIME;
        
        // PID control law: u = vref + Kp*e + Ki*sum_e + Kd*de/dt
        v_global[i] = v_ref[i] + ControllerParam.kp[i]*error 
                             + ControllerParam.ki[i]*sum_error[i] 
                             + ControllerParam.kd[i]*derivative;
        
        // Saturation: limit velocity to max_vel
        u = sat(v_global[i], ControllerParam.max_vel[i], ControllerParam.max_vel[i]);
        
        // Anti-windup: adjust integral based on saturation
        sum_error[i] += (error + ControllerParam.kb[i]*(u-v_global[i])) * SAMPLE_TIME;
        
        v_global[i] = u;
        
        // Store current error for next iteration
        prev_error[i] = error;  // NEW
    }
    
    VelWorld2Robot(v_local, v_global);
    InverseKinematicTrans(v_local, wheel_speed_ref.value);
}
```

## Key Improvements

### 1. **Derivative Action**
- Anticipatory response: Acts on rate of change of error
- Reduces overshoot and improves settling time
- Particularly effective for step reference changes

### 2. **Better Transient Response**
- Proportional term: Fast initial response
- Derivative term: Damping effect on overshoots
- Integral term: Guarantees zero steady-state error

### 3. **Maintained Anti-Windup**
- Back-calculation term (`kb`) prevents integral windup
- Critical when saturation limits are active
- Ensures smooth transitions when constraints are reached

### 4. **Three Independent Control Channels**
- Vx (forward velocity): gains `[0]`
- Vy (lateral velocity): gains `[1]`
- ω (angular velocity): gains `[2]`

## Tuning Guidelines

### Default Tuning (Starting Point):
- **Translation (X,Y)**: `kp=1, ki=0.25, kd=0.05, kb=0.25`
- **Rotation (θ)**: `kp=1, ki=0.25, kd=0.05, kb=0.25`

### Tuning Steps:

1. **Start with P-only** (ki=0, kd=0, kb=0)
   - Adjust kp until acceptable tracking
   - May have steady-state error

2. **Add I-gain** (kd=0)
   - Eliminates steady-state error
   - Increases overshoot (use kb to mitigate)

3. **Add D-gain**
   - Reduces overshoot and settling time
   - Use conservatively to avoid noise amplification

### Adjustment Strategy:
```
Too slow:        Increase kp or increase kd
Overshoot:       Increase kd or kp slightly less
Oscillations:    Increase kd or decrease ki
Steady-state err: Increase ki (may need more kd)
```

## Performance Characteristics

| Aspect | PI Controller | PID Controller |
|--------|---------------|----------------|
| Steady-State Error | Zero (I-action) | Zero (I-action) |
| Response Time | Moderate | **Faster** |
| Overshoot | Significant | **Reduced** |
| Settling Time | Longer | **Shorter** |
| Noise Sensitivity | Low | Moderate (due to D) |
| Stability | Good | **Better** |

## Usage Example

```cpp
int main(int argc, char** argv) {
    MecanumWMR wmr;
    if (wmr.init()==false) return 1;
    
    wmr.setInitCond(0, 0, 0);
    
    // Set PID gains with derivative
    wmr.setPIDgain_translation(1.0, 0.25, 0.05, 0.25);  // kp, ki, kd, kb
    wmr.setPIDgain_rotation(1.0, 0.25, 0.05, 0.25);
    
    wmr.setVelocityLimit(0.6, 0.8);
    
    std::array<float, 3> target_point = {1.0, 1.0, M_PI/4};
    wmr.Point2PointMove(target_point);
    
    wmr.shutdown();
    return 0;
}
```

## Derivative Filtering (Optional Enhancement)

If noise is an issue with the derivative term, implement first-order filtering:

```cpp
// Pseudo-code for filtered derivative
derivative = (alpha * raw_derivative) + ((1 - alpha) * prev_derivative);
// where alpha ≈ 0.3-0.5 (0 = no filtering, 1 = full derivative)
```

## Files Modified

1. **MecanumWMR.h**
   - Updated `ControllerParameter` default kd values
   - Updated function signatures with kd parameter

2. **MecanumWMR.cpp**
   - Modified `setPIDgain_translation()` to handle kd
   - Modified `setPIDgain_rotation()` to handle kd
   - Enhanced `pos_controller()` with derivative calculation and error history

## Testing Recommendations

1. Compare `pos_performance.txt` and `vel_performance.txt` logs between PI and PID
2. Record video of robot movement to visually assess improvement
3. Measure settling time and overshoot metrics
4. Test on multiple trajectories (straight lines, curves, rotations)

---

**Implementation Date**: December 2025
**Controller Type**: Discrete-time PID with anti-windup
**Sample Time**: 50 ms (20 Hz control loop)
**Status**: Ready for deployment and tuning
