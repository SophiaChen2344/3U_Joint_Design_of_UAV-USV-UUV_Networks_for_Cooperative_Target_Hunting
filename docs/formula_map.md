# Formula Map

## Stability Preliminaries

- Eq. (1): generic nonlinear system
- Eq. (2)-(4): practical fixed-time Lyapunov inequality and settling-time bound
- Eq. (5): Young-type inequality used in proof

## Dynamics

- Eq. (6): follower UAV planar kinematics with disturbance
- Eq. (7): leader UAV planar kinematics

## Disturbance Observer

- Eq. (8): low-pass filters for position and velocity
- Eq. (9)-(10): invariant manifold relation
- Eq. (11): USDE disturbance estimate

## Formation Error and Control

- Eq. (12): formation error under fixed topology
- Eq. (13): derivative of formation error
- Eq. (14): fixed-time virtual control under ideal communication conditions
- Eq. (15): linear velocity and desired bearing angle
- Eq. (16): angular velocity controller

## RL Topology Reconfiguration

- Eq. (17): MDP state
- Eq. (18): action-to-topology-index mapping
- Eq. (19): state transition
- Eq. (20)-(23): reward design
- Eq. (24)-(31): DDPG update rules

## Final Closed-Loop System

- Eq. (32): formation error under topology reconfiguration
- Eq. (33): error derivative under topology reconfiguration
- Eq. (34): final fixed-time control law with topology switching and disturbance compensation

## Stability Verification

- Eq. (35): disturbance estimation error dynamics
- Eq. (36)-(40): UUB proof for disturbance and bearing angle errors
- Eq. (41)-(43): practical fixed-time convergence proof for formation error
