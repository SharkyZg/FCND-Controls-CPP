## Project: Control of a 3D Quadrotor
### Marko Sarkanj
![Quad Image](./misc/screenshot_1.png)

---


### Writeup / README

This writeup contains the description of implementation of the Control of a 3D Quadrotor project. This project has been completed as part of the Self Flying Car Nanodegree program from Udacity.

### Code Implementation

#### 1. Implementation of body rate control in C++

The body rate controller receives desired body rates and current body rates as input. The result of subtracting current body rates from desired body rates is rates error that serves as input in calculating desired 3-axis moment. 

Other two inputs for calculating desired 3-axis moment are gain parameter and moments of inertia. Both of those inputs are predefined in `QuadControlParams.txt` file as parameters.

The desired 3-axis moment is calculated by multiplying moments of inertia, gain parameter and body rates error. 

```cpp
  V3F momentOfInertia;

  momentOfInertia.x = Ixx;
  momentOfInertia.y = Iyy;
  momentOfInertia.z = Izz;

  V3F rateError = pqrCmd - pqr;
  momentCmd = momentOfInertia * kpPQR * rateError;
```


#### 2. Implementation of roll pitch controller in C++

In the case that the collective trust of the controller is zero, the controller returns roll and pitch rates 0. The reason for this is that there is no change in roll and pitch accelerations possible without thrust. Yaw rate is zero because yaw is controlled by another controller separately.

The first value `cd`(acceleration) is calculated by dividing negative collective thrust in Newtons with the mass of the drone. This value is used to calculate `targetR13` and `targetR23` values based on the desired x and y lateral accelerations. `targetR13` and `targetR23` values are used to calculate `R13Err` and `R23Err` error rates between target and current values from the attitude rotation matrix. 

Roll and pitch rates are then calculated by using `R13Err` and `R23Err` error rates, values from the attitude rotation matrix and `kpBank` roll/pitch gain with help of the following formula:
![Quad Image](./misc/screenshot_2.png)

Source: Udacity lectures


```cpp
  if ( collThrustCmd > 0 ) {
    float cd = - collThrustCmd / mass;

    float targetR13 = CONSTRAIN(accelCmd.x / cd, -maxTiltAngle, maxTiltAngle);
    float R13Err = targetR13 - R(0,2);

    float targetR23 = CONSTRAIN(accelCmd.y / cd, -maxTiltAngle, maxTiltAngle);
    float R23Err = targetR23 - R(1,2);

    pqrCmd.x = 1 / R(2,2) * (R(1,0) * kpBank * R13Err - R(0,0) * kpBank * R23Err) ;
    pqrCmd.y = 1 / R(2,2) * (R(1,1) * kpBank * R13Err - R(0,1) * kpBank * R23Err);

  } else {
    pqrCmd.x = 0.0;
    pqrCmd.y = 0.0;
  }

  pqrCmd.z = 0;
```

#### 3. Implementation of altitude controller in C++

Input values of the altitude controller are target altitude position and velocity as well as current altitude position and velocity. In addition to those inputs there are the the time step of the measurements as well as the feed-forward vertical acceleration. 

```cpp
  float P = kpPosZ * (posZCmd - posZ);
  float D = kpVelZ * (velZCmd - velZ) + velZ;
  integratedAltitudeError += (posZCmd - posZ) * dt;
  float I = KiPosZ * integratedAltitudeError;

  float u1Bar = P + D + I + accelZCmd;

  float accelerationCmd = ( u1Bar - CONST_GRAVITY ) / R(2,2);

  thrust = - mass * CONSTRAIN(accelerationCmd, - maxAscentRate / dt, maxAscentRate / dt);
```

#### 4. Implementation of lateral position control in C++

```cpp
  V3F kpPosition;
  kpPosition.x = kpPosXY;
  kpPosition.y = kpPosXY;
  kpPosition.z = 0.f;

  V3F kpVelocity;
  kpVelocity.x = kpVelXY;
  kpVelocity.y = kpVelXY;
  kpVelocity.z = 0.f;

  if ( velCmd.mag() > maxSpeedXY ) {
    velCmd = velCmd.norm() * maxSpeedXY;
  }

  accelCmd = kpPosition * ( posCmd - pos ) + kpVelocity * ( velCmd - vel ) + accelCmd;

  if ( accelCmd.mag() > maxAccelXY ) {
    accelCmd = accelCmd.norm() * maxAccelXY;
  }
```

#### 5. Implementation of yaw control in C++

```cpp
  float constrYawCmd = CONSTRAIN(yawCmd, -2 * F_PI, 2 * F_PI);

  float yawError = constrYawCmd - yaw;

  if ( yawError > F_PI ) {
    yawError = yawError - 2 * F_PI;
  } if ( yawError < -F_PI ) {
    yawError = yawError + 2 * F_PI;
  }

  yawRateCmd = kpYaw * yawError;
```

#### 5. Implementation of calculating the motor commands given commanded thrust and moments in C++
```cpp
    float l = L / sqrtf(2.f);

    float Fx = momentCmd.x / l;
    float Fy = momentCmd.y / l;
    float Fz = - momentCmd.z / kappa;

    cmd.desiredThrustsN[0] = (Fx + Fy + Fz + collThrustCmd)/4.f;
    cmd.desiredThrustsN[1] = (-Fx + Fy - Fz + collThrustCmd)/4.f;
    cmd.desiredThrustsN[2] = (Fx - Fy - Fz + collThrustCmd)/4.f ;
    cmd.desiredThrustsN[3] = (-Fx - Fy + Fz + collThrustCmd)/4.f;
```