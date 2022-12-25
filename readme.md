# CartPole

[![CI](https://github.com/dasimagin/cart_pole/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/dasimagin/cart_pole/actions/workflows/ci.yml)

## Overview
This is a student project that is designed to learn the basics of robotics and control theory.
The environment is some variation of the cart-pole problem described by Barto, Sutton, and Anderson.
A pole is attached by an joint to a cart, which moves along guide axis.
Some stepper drives the cart. The control target is desired acceleration of the cart.

The cart starts at the middle with no velocity and acceleration. The pole is initially at rest state.
The goal is to swing up the pole and maintain it in upright pose by increasing and reducing the cart's velocity.

![CartPole](docs/svg/classic_cart_pole.svg)

Also there is radial variation, where cart moves in a circle.

![RadialCartPole](docs/svg/radial_cart_pole.svg)

## Interface
We declare a common [interface](cart_pole/blob/master/interface.py) both for simulator and control.
It allows us to easily train model and make inference on device, use transfer learning, generate real training samples and etc.

### State
**Field**     | **Unit** | **Description**
------------- | -------- | ---------------
positon       | m        | Position of the cart along the guide axis. The middle is a reference point.
velocity      | m/s      | Instantaneous linear speed of the cart.
accelerration | m/s^2    | Instantaneous acceleration of the cart.
pole_angle    | rad      | Angle of pole (ccw). The he lowest position is the reference point. 
pole_velocity | rad/s    | Instantaneous angular velocity of the pole.
error_code    | enum     | Current error code

### Error code
**Code**       | **Int** | **Description**
---------------| ------- | ---------------
NO_ERROR       | 0       | Position of the cart along the guide axis. The middle is a reference point.
NEED_RESET     | 1       | Environment loose control (use soft or hard reset).
X_OVERFLOW     | 2       | The maximum allowed position is achieved.
V_OVERFLOW     | 3       | The maximum allowed velocity is achieved.
A_OVERFLOW     | 4       | The maximum allowed acceleration is achieved.
MOTOR_STALLED  | 5       | Some problems with stepper.
ENDSTOP_HIT    | 6       | The cart has touched one of endstop hit.
