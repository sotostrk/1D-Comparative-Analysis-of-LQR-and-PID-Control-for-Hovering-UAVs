# 1D-Comparative-Analysis-of-LQR-and-PID-Control-for-Hovering-UAVs 

A MATLAB simulation comparing LQR and PID controllers for altitude stabilization of a 1D drone under wind disturbance. The project features state-space modeling, control effort analysis, and side-by-side performance evaluation of two control strategies.

---

## Objective

Stabilize a vertically falling drone using two control strategies:

- **LQR**: Stabilizes the system around its natural equilibrium (where thrust balances gravity), with no explicit position reference.
  - design constraints:
  - Settling time < 2.5 seconds
  - Overshoot < 5%
- **PID**: Tracks a position reference (z = 0) and attempts to bring the drone back to that target over time.
- Also compare their Control effort (J = ∫u² dt) as a metric of controller efficiency.

> **Note:** Both controllers are tested under gravity and sinusoidal wind disturbance.

---

## System Model

We model the drone’s vertical motion using Newton’s 2nd Law:
- m * ẍ = u - mg + F_wind(t)
  
Rewritten as a state-space system:
- x₁ = z (altitude)
- x₂ = ż (vertical velocity)
- ẋ₁ = x₂
- ẋ₂ = (1/m) * (u - mg + F_wind)

---

## Features

 - LQR gain tuning with an automated search for the Q and R matrices using a for-loop to find gain values that satisfy the design constraints 
 - Sinusoidal wind disturbance  
 - Custom-built PID controller (manual Euler simulation)  
 - Control effort calculation (J = ∫u² dt)  
 - Visual comparison of position and control input  
 

---

## Simulation Results

### LQR Closed-Loop Response (with wind)

![LQR Response](https://raw.githubusercontent.com/sotostrk/1D-Comparative-Analysis-of-LQR-and-PID-Control-for-Hovering-UAVs/main/figs/Fig_1.png)

- The LQR controller stabilizes the drone quickly, with minimal overshoot
- Position converges to hover around its equilibrium point (where u ≈ mg), oscillating around ~2.94 meters.
- Thrust output oscillates around 0 N, as gravity is compensated directly in the control law

---

### PID vs LQR Comparison

![PID vs LQR](https://raw.githubusercontent.com/sotostrk/1D-Comparative-Analysis-of-LQR-and-PID-Control-for-Hovering-UAVs/main/figs/Fig_2_updated.png)

- PID exhibits higher overshoot and oscillation  
- LQR is smoother, more stable, and uses less thrust overall  
- PID takes longer to settle and responds more aggressively

---

## Control Effort Comparison

We compute the control effort as:
J = ∫ u(t)² dt
--
**LQR** : J = ~ 12.38 (N²·s) 

**PID** : J = ~ 17,761.74 (N²·s)

>  **Note:** Both simulations were run over 20 seconds with identical initial conditions and active wind disturbance.  
> The LQR controller achieves stability with minimal adjustments, while the PID controller, though effective, reacts more aggressively and continuously to disturbances.  
> This results in a significantly higher total control effort.

---

## Conclusions

- **LQR** provides fast, smooth, and energy-efficient stabilization by optimally balancing state error and control effort through gain tuning.
- **PID** performs reasonably well with hand tuning, but without optimization or full state feedback, it struggles to minimize effort and can become overly reactive — especially under constant disturbances like wind.
- The control effort comparison highlights LQR’s superior efficiency for dynamic systems requiring stability and robustness.

---

##  Project Files

| File | Description |
|------|-------------|
| `main.m` | Runs LQR simulation with wind disturbance |
| `tune_lqr.m` | Finds optimal Q/R values for LQR controller |
| `simulate_closedloop.m` | LQR dynamics simulator |
| `dynamics_closedloop_wind.m` | Adds wind force to closed-loop dynamics |
| `wind_force.m` | Time-varying wind function |
| `simulate_pid.m` | Custom PID controller using Euler integration |
| `compare_pid_lqr.m` | Plots PID vs LQR performance |
| `compute_control_effort.m` | Calculates J = ∫ u² dt |
| `figs/` | Folder for saved plots |

---

## Final Notes

This project demonstrates how different control strategies perform in a realistic dynamic environment. While PID is widely used and easy to implement, LQR offers powerful advantages in efficiency, stability, and robustness — especially when disturbances are present.
