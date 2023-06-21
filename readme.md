# CartPole

Simle getting started is [here](docs/guide.md). Follow our progress on [YouTube](https://youtube.com/playlist?list=PLR1nN_AQOO9yAG5CHOA4l2x3j89t-3PYf).

## Overview
It is a student project that is designed to learn the basics of robotics and control theory.
The environment is some variation of the cart-pole problem described by Barto, Sutton, and Anderson.
A pole is attached by an joint to a cart, which moves along guide axis.
Some stepper drives the cart. The control target is desired acceleration of the cart.

The cart starts at the middle with no velocity and acceleration. The pole is initially at rest state.
The goal is to swing up the pole and maintain it in upright pose by increasing and reducing cart's velocity. Also there is radial variation, where cart moves in circle.

| **Linear CartPole**                         | **Radial CartPole**                              |
|---------------------------------------------|--------------------------------------------------|
| ![CartPole](docs/svg/linear_cart_pole.svg)  | ![RadialCartPole](docs/svg/radial_cart_pole.svg) |
