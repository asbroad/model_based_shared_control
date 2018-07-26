#!/usr/bin/env python

import gym
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from model_based_shared_control.msg import State
from pyglet.window import key
import numpy as np

class LunarLander():

  def __init__(self):
    # initalize node
    rospy.init_node('lunar_lander')

    # register shutdown hook
    rospy.on_shutdown(self.shutdown_hook)
    self.called_shutdown = False

    self.user_id = str(rospy.get_param('user_id'))
    self.user_id = self.user_id.zfill(2)
    self.main_joystick = rospy.get_param('main_joystick')
    self.inverted = rospy.get_param('inverted')
    self.data_path = rospy.get_param('data_path')

    # keep track of current keystroke
    self.user_actions = [0, 0]

    # build environment
    self.env = gym.make('LunarLanderMultiFire-v0')
    self.env.reset()
    self.env.render()

    # set up keystroke hooks
    self.env.viewer.window.on_key_press = self.key_press
    self.terminate = False

    # set up joy subscriber
    rospy.Subscriber('/joy', Joy, self.joy_callback)

    # set up state and shutdown publishers
    self.state_pub = rospy.Publisher('/state', State, queue_size=1)
    self.shutdown_pub = rospy.Publisher('/shutdown', String, queue_size=1)
    state_msg = State()
    done = False

    # set up goal locations
    self.goal_x_list = [10]
    self.goal_y_list = [6]
    self.goal_x_idx = 0

    # run system with input from user
    r = rospy.Rate(10)
    self.total_reward, self.total_steps, self.trial_idx = 0, 0, 1
    while not rospy.is_shutdown():
      if self.check_if_success():
        self.trial_idx += 1
        self.env.reset()
      if self.env.legs[0].ground_contact or self.env.legs[1].ground_contact or done:
        self.trial_idx += 1
        self.env.reset()
        done = False
      else:
        # get user input
        main_thruster, side_thruster = self.user_actions
        # write message
        state_msg.x = self.env.lander.position.x - self.goal_x_list[0]
        state_msg.y = self.env.lander.position.y - self.goal_y_list[0]
        state_msg.theta = self.env.lander.angle
        state_msg.x_dot = self.env.lander.linearVelocity.x
        state_msg.y_dot = self.env.lander.linearVelocity.y
        state_msg.theta_dot = self.env.lander.angularVelocity
        state_msg.u_1 = main_thruster
        state_msg.u_2 = side_thruster
        # publish message
        self.state_pub.publish(state_msg)
        # take step
        observation, reward, done, info = self.env.step(np.array([main_thruster, side_thruster]))
        self.total_reward += reward
        self.total_steps += 1

      if self.terminate == True:
        self.shutdown_hook()
        print('Terminating early')
        break

      # update screen and keep time
      self.env.render()
      r.sleep()

  def check_if_success(self):
    dist = np.sqrt(np.power((self.env.lander.position.x - self.env.goal_x), 2) + np.power((self.env.lander.position.y - self.env.goal_y), 2))
    x_vel = np.sqrt(np.power(self.env.lander.linearVelocity.x, 2))
    y_vel = np.sqrt(np.power(self.env.lander.linearVelocity.y,2))
    a_vel = np.sqrt(np.power(self.env.lander.angularVelocity,2))
    if dist < 0.9 and x_vel < 1.5 and y_vel < 1 and a_vel < 0.3:
      return True
    else:
      return False

  def shutdown_hook(self):
    if not self.called_shutdown:
      print('Shutting down')
      self.called_shutdown = True
      # store koopman operator
      koopman_filepath = self.data_path + 'models/' + self.user_id # koopman_operator.hpp sets file extension
      self.shutdown_pub.publish(koopman_filepath)
      print('Saved Koopman model.')

  def joy_callback(self, data):
    invert = 1
    if self.inverted:
      invert = -1
    if self.main_joystick == 'right':
      self.user_actions = [data.axes[3], invert*data.axes[0]]
    elif self.main_joystick == 'left':
      self.user_actions = [data.axes[1], invert*data.axes[2]]

  def key_press(self, k, mod):
    if k == key.SPACE:
      self.terminate = True

if __name__=='__main__':
  ll = LunarLander()
