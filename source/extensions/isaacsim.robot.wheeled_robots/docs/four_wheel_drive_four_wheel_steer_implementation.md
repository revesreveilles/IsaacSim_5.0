# Four-Wheel Drive Four-Wheel Steer (4WD4WS) Controller Implementation

## Overview

The `FourWheelDriveFourWheelSteerController` implements a kinematic controller for vehicles with independent steering and drive control on all four wheels. This configuration provides maximum maneuverability and precise control over vehicle motion.

## Mathematical Foundation

### Coordinate Systems

The controller uses two coordinate systems:

1. **World Frame**: Fixed global coordinate system
2. **Vehicle Frame**: Local coordinate system attached to the vehicle chassis center

The transformation between these frames is defined by the vehicle's yaw angle (heading).

### Kinematic Model

For a 4WD4WS vehicle, each wheel can independently:
- **Drive**: Rotate to provide propulsion
- **Steer**: Change orientation to direct motion

#### Wheel Velocity Calculation

For each wheel *i*, the velocity at the contact point is calculated using:

```
v_wheel_i = v_chassis + ω × r_i
```

Where:
- `v_chassis`: Chassis linear velocity vector [vx, vy, vz] in m/s
- `ω`: Chassis angular velocity vector [ωx, ωy, ωz] in rad/s (typically only ωz is non-zero for planar motion)
- `r_i`: Position vector from chassis center to wheel *i* in meters
- `×`: Cross product operator

**Cross Product Details:**
```
ω × r_i = |  i      j      k   |
          | ωx     ωy     ωz   |
          | rx_i   ry_i   rz_i |

        = i(ωy·rz_i - ωz·ry_i) + j(ωz·rx_i - ωx·rz_i) + k(ωx·ry_i - ωy·rx_i)
```

For planar motion (ωx = ωy = 0, rz_i = 0):
```
ω × r_i = [-ωz·ry_i, ωz·rx_i, 0]
```

#### Steering Angle Calculation

Once the wheel velocity vector is computed, the steering angle is determined by:

```
θ_steer_i = -atan2(vy_wheel_i, vx_wheel_i)
```

The negative sign accounts for the Isaac Sim/USD coordinate system convention.

#### Wheel Angular Velocity

The wheel's rotational velocity (in rad/s) is calculated from the linear velocity magnitude:

```
ω_wheel_i = |v_wheel_i| / R_i
```

Where:
- `|v_wheel_i|`: Magnitude of the wheel velocity vector in m/s
- `R_i`: Radius of wheel *i* in meters
- `ω_wheel_i`: Angular velocity in rad/s

## Implementation Details

### Default Wheel Positions

If wheel positions are not explicitly provided, the controller calculates default positions based on wheelbase and track width:

```
Front Left:  [ wheelbase/2,  track_width/2, 0]
Front Right: [ wheelbase/2, -track_width/2, 0]
Rear Left:   [-wheelbase/2,  track_width/2, 0]
Rear Right:  [-wheelbase/2, -track_width/2, 0]
```

### Coordinate Transformation

The controller transforms chassis velocities from world frame to vehicle frame using:

```python
R(-yaw) = | cos(-yaw)  -sin(-yaw)  0 |
          | sin(-yaw)   cos(-yaw)  0 |
          |     0           0      1 |

v_vehicle = R(-yaw) · v_world
```

This ensures the velocity commands are interpreted in the vehicle's local coordinate system.

### Velocity Limits

The controller enforces several limits:

1. **Maximum Wheel Angular Velocity**: Clips wheel velocities to ±`max_wheel_velocity` (rad/s)
2. **Maximum Steering Angle**: Clips steering angles to ±`max_steering_angle` (rad)
3. **Maximum Linear Velocity**: Derived from `max_wheel_velocity × average_wheel_radius`
4. **Maximum Steering Velocity**: Limits the rate of change of angular velocity

### Unit Consistency

**Important**: The controller maintains unit consistency throughout:

- **Input**: 
  - Linear velocity in m/s
  - Angular velocity in rad/s
  
- **Internal Calculation**:
  - Wheel linear velocities in m/s
  - Conversion to angular velocities in rad/s
  
- **Output**:
  - Joint velocities in rad/s (wheel rotational speeds)
  - Joint positions in rad (steering angles)

## Algorithm Flow

```
1. Input: chassis_linear_vel (m/s), chassis_angular_vel (rad/s), yaw (rad), dt (s)

2. Transform velocities to vehicle frame:
   v_vehicle = R(-yaw) · v_chassis

3. For each wheel i = 0, 1, 2, 3:
   a. Calculate wheel position offset: r_i
   b. Calculate rotational contribution: ω × r_i
   c. Calculate total wheel velocity: v_wheel_i = v_vehicle + ω × r_i
   d. Calculate velocity magnitude: |v_wheel_i|
   e. Calculate steering angle: θ_i = -atan2(vy_wheel_i, vx_wheel_i)
   f. Convert to angular velocity: ω_wheel_i = |v_wheel_i| / R_i

4. Apply velocity and angle limits

5. Output: ArticulationAction with joint velocities and positions
```

## Advantages of 4WD4WS

1. **Zero-radius Turning**: Can rotate in place when all wheels point to instantaneous center at chassis center
2. **Crab Walking**: Can move sideways when all wheels are aligned parallel
3. **Diagonal Motion**: Can move in any direction by coordinating wheel angles
4. **Precise Path Following**: Each wheel can be independently controlled for accurate trajectory tracking

## Motion Modes

### Forward/Backward Motion
- All wheels aligned straight (θ = 0°)
- Equal velocities on all wheels
- Angular velocity ω = 0

### Rotation in Place
- All wheels point tangent to circles centered at vehicle center
- Wheel velocities proportional to distance from center
- Linear velocity v = 0

### Crab Walk (Lateral Motion)
- All wheels aligned parallel at 90° to vehicle longitudinal axis
- Equal velocities on all wheels
- Angular velocity ω = 0

### General Motion
- Combination of translation and rotation
- Each wheel has unique angle and velocity
- Wheels point tangent to their circular paths around instantaneous center

## Common Issues and Solutions

### Issue: Wheels Not Moving Despite Non-zero Commands
**Cause**: Velocity limits set too low or wheel radius mismatch
**Solution**: Check `max_wheel_velocity` parameter and ensure `wheel_radii` array matches actual wheel sizes

### Issue: Steering Angles Incorrect
**Cause**: Coordinate system mismatch or yaw angle incorrect
**Solution**: Verify yaw angle source and check negative sign in `atan2` calculation matches your USD file conventions

### Issue: Jerky Motion
**Cause**: Velocity commands changing too rapidly or acceleration limits not set
**Solution**: Implement velocity smoothing or set appropriate `max_acceleration` values

## References

- J. M. Galko and M. Tomizuka, "Designing Control Systems for an Omni-Directional Mobile Robot," IEEE, 1996
- G. Campion, G. Bastin, and B. D'Andréa-Novel, "Structural properties and classification of kinematic and dynamic models of wheeled mobile robots," IEEE Transactions on Robotics and Automation, vol. 12, no. 1, pp. 47-62, 1996.

## Example Usage

```python
import numpy as np
from isaacsim.robot.wheeled_robots.controllers import FourWheelDriveFourWheelSteerController

# Create controller
controller = FourWheelDriveFourWheelSteerController(
    name="my_4wd4ws",
    wheel_base=1.0,        # meters
    track_width=0.8,       # meters
    wheel_radii=np.array([0.1, 0.1, 0.1, 0.1]),  # meters
    max_wheel_velocity=10.0,  # rad/s
    max_steering_angle=np.pi/2,  # 90 degrees
)

# Command: move forward at 1 m/s, no rotation, current yaw = 0, dt = 0.01s
command = [
    [1.0, 0.0, 0.0],  # linear velocity
    [0.0, 0.0, 0.0],  # angular velocity
    0.0,              # yaw
    0.01              # dt
]

action = controller.forward(command)
# action.joint_velocities: wheel angular velocities (rad/s)
# action.joint_positions: steering angles (rad)
```

---

**Last Updated**: November 3, 2025
**Version**: 1.0
