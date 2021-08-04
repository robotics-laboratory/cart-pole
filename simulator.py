import numpy as np
import pybullet as pb

from pybullet_utils import bullet_client as pbc

from state import Error, State
from . import util

import gym
import logging
import os
import time


world_center = [0, 0, 0]
no_rotation = [0, 0, 0, 1]

class CartPoleSimulator(gym.Env):
    '''
    Description:
        Сlass implements a physical simulation of the cart-pole device.
        A pole is attached by an joint to a cart, which moves along guide axis.
        The pendulum is initially at rest state. The goal is to maintain it in
        upright pose by increasing and reducing the cart's velocity.
    Source:
        This environment is some variation of the cart-pole problem
        described by Barto, Sutton, and Anderson
    Reward:
        Reward is exp(-cos(a)), where a is pole angle in radians.
    Initial state:
        A pole is at starting position 0 with no velocity and acceleration.
    Episode Termination:
        - Cart position is violated.
        - Episode length is greater than limit.
    Technical details:
        Each environment runs its own isolated pybullet physics engine.
    '''

    def __init__(self,
            time_step=1/50,
            step_n=1000,
            position_limit=0.20,
            gravity=9.8):
        self.time_step = time_step
        self.position_limit = position_limit
        self.step_n = step_n
        self.step_count = 0

        self.client = pbc.BulletClient(pb.DIRECT)
        self.client.setTimeStep(self.time_step)

        self.object_id = self.client.loadURDF(
            'cart_pole_v0.urdf',
            basePosition = world_center,
            baseOrientation = no_rotation,
            useFixedBase=True,
            flags=pb.URDF_USE_SELF_COLLISION,
        )
        
        self.index_by_name = {}
        for index in range(self.client.getNumJoints(self.object_id)):
            _, name, *__ = self.client.getJointInfo(self.object_id, index)
            self.index_by_name[name] = index

        self.slider_to_cart_index = self.index_by_name[b'slider_to_cart']
        self.cart_to_pole_index = self.index_by_name[b'cart_to_pole']

        # enable free pole rotation
        self.client.setJointMotorControl2(
            bodyUniqueId=self.object_id, 
            jointIndex=self.cart_to_pole_index,
            controlMode=pb.VELOCITY_CONTROL,
            force=0
        )
        
        self.client.changeDynamics(
            bodyUniqueId=self.object_id, 
            linkIndex=self.cart_to_pole_index,
            linearDamping=0,
            angularDamping=0
        )

        # configure cart contraints
        self.client.setJointMotorControl2(
            bodyUniqueId=self.object_id, 
            jointIndex=self.slider_to_cart_index,
            controlMode=pb.POSITION_CONTROL,
            force=100.0, # N
            positionGain=1.0,
            velocityGain=0.0,
            maxVelocity=0.5,  # m/s
        )
        
        # disable any damping
        self.client.changeDynamics(
            bodyUniqueId=self.object_id, 
            linkIndex=self.slider_to_cart_index,
            linearDamping=0,
            angularDamping=0
        )

        self.client.setGravity(0, 0, -gravity)
        self.client.setRealTimeSimulation(0)

        # Formally, we need reset env to reset error.
        # Done for similarity with real control, where need to make homing.
        self.error = Error.NOT_INITIALIZED

    def state(self):
        '''
        Returns:
            Current state of system.
        '''

        cart_position, cart_velocity, *_ = self.client.getJointState(
            bodyUniqueId=self.object_id,
            jointIndex=self.slider_to_cart_index
        )

        pole_angle, pole_velocity, *_ = self.client.getJointState(
            bodyUniqueId=self.object_id,
            jointIndex=self.cart_to_pole_index
        )

        return State(
            cart_position=cart_position,
            cart_velocity=cart_velocity,
            pole_angle=pole_angle,
            pole_velocity=pole_velocity,
            error=self.error
        )
        
    def reset(self):
        '''
        Description:
            Resets the environment to an initial state. The pole is at rest position.

        Returns:
            Initial state. 
        '''

        initial_state = State(
            cart_position=0,
            cart_velocity=0,
            pole_angle=0,
            pole_velocity=0
        )

        return self.reset_to(initial_state)

    def reset_to(self, state):
        '''
        Description:
            Resets the environment to required state.
            The method is out of scope gym env and robot control.
            May be useful for training purposes.

        Returns:
            Initial state. 
        '''

        assert state
        self.error = state.error

        self.client.resetJointState(
            bodyUniqueId=self.object_id,
            jointIndex=self.cart_to_pole_index,
            targetValue=state.pole_angle,
            targetVelocity=state.pole_velocity
        )

        self.client.resetJointState(
            bodyUniqueId=self.object_id,
            jointIndex=self.slider_to_cart_index,
            targetValue=state.cart_position,
            targetVelocity=state.cart_velocity
        )

        return self.state()

    def validate(self, action):    
        if self.error:
            return self.error

        if abs(action) > self.position_limit:
            return Error.INVALID_CART_POSITION
        
        return Error.NO_ERROR


    def step(self, action):
        self.error = self.validate(action)
        
        if self.error:
            reward = 0
            finish = True
            self.error = Error.INVALID_CART_POSITION
        
            info = {
                'step_count': self.step_count
            }

            return self.state(), reward, finish, info
        
        self.client.setJointMotorControl2(
            bodyUniqueId=self.object_id,
            jointIndex=self.slider_to_cart_index,
            controlMode=pb.POSITION_CONTROL,
            targetPosition=action
        )

        self.client.stepSimulation()
        self.step_count += 1 
        state = self.state()
        
        reward = util.reward(state)
        finish = False
        if self.step_count >= self.step_n:
            finish = True

        info = {
            'step_count': self.step_count
        }
        
        return state, reward, finish, info


    def render(self, width=300, height=300):
        view_matrix = self.client.computeViewMatrix(
            cameraTargetPosition=[0, 0.5, 0],
            cameraEyePosition=[0, 0.7, 0],
            cameraUpVector=[0, 0, 1]
        )

        proj_matrix = self.client.computeProjectionMatrixFOV(
            fov=60,
            aspect=width / height,
            nearVal=0.1,
            farVal=10.0
        )

        width, height, pixels, depth, segmentation = self.client.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            shadow=1,
            renderer=pb.ER_TINY_RENDERER,
            flags=pb.ER_NO_SEGMENTATION_MASK
        )

        rgb = np.reshape(np.array(pixels, dtype=np.uint8), (height, width, -1))
        return rgb[:,:,:3]