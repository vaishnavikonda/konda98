# konda98
import numpy as np
import matplotlib.pyplot as pt
import pandas as pd
# generate and send the correct commands to the robot
def _send_robot_commands( self ):
  ...
  v_l, v_r = self._uni_to_diff( v, omega )
  self.robot.set_wheel_drive_rates( v_l, v_r )

def _uni_to_diff( self, v, omega ):
  # v = translational velocity (m/s)
  # omega = angular velocity (rad/s)

  R = self.robot_wheel_radius
  L = self.robot_wheel_base_length

  v_l = ( (2.0 * v) - (omega*L) ) / (2.0 * R)
  v_r = ( (2.0 * v) + (omega*L) ) / (2.0 * R)

  return v_l, v_r
  # update the distances indicated by the proximity sensors
def _update_proximity_sensor_distances( self ):
    self.proximity_sensor_distances = [ 0.02-( log(readval/3960.0) )/30.0 for
        readval in self.robot.read_proximity_sensors() ]
        # update the estimated position of the robot using it's wheel encoder readings
def _update_odometry( self ):
    R = self.robot_wheel_radius
    N = float( self.wheel_encoder_ticks_per_revolution )
    
    # read the wheel encoder values
    ticks_left, ticks_right = self.robot.read_wheel_encoders()
    
    # get the difference in ticks since the last iteration
    d_ticks_left = ticks_left - self.prev_ticks_left
    d_ticks_right = ticks_right - self.prev_ticks_right
    
    # estimate the wheel movements
    d_left_wheel = 2*pi*R*( d_ticks_left / N )
    d_right_wheel = 2*pi*R*( d_ticks_right / N )
    d_center = 0.5 * ( d_left_wheel + d_right_wheel )
    
    # calculate the new pose
    prev_x, prev_y, prev_theta = self.estimated_pose.scalar_unpack()
    new_x = prev_x + ( d_center * cos( prev_theta ) )
    new_y = prev_y + ( d_center * sin( prev_theta ) )
    new_theta = prev_theta + ( ( d_right_wheel - d_left_wheel ) / self.robot_wheel_base_length )
    
    # update the pose estimate with the new values
    self.estimated_pose.scalar_update( new_x, new_y, new_theta )
    
    # save the current tick count for the next iteration
    self.prev_ticks_left = ticks_left
    self.prev_ticks_right = ticks_right
    # return a go-to-goal heading vector in the robot's reference frame
def calculate_gtg_heading_vector( self ):
    # get the inverse of the robot's pose
    robot_inv_pos, robot_inv_theta = self.supervisor.estimated_pose().inverse().vector_unpack()
    
    # calculate the goal vector in the robot's reference frame
    goal = self.supervisor.goal()
    goal = linalg.rotate_and_translate_vector( goal, robot_inv_theta, robot_inv_pos )
    
    return goal
