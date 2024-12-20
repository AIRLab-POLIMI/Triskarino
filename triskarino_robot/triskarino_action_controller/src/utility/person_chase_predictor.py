#!/usr/bin/env python3
from utility.chase_utilities import is_bbox_on_the_edge, get_rectangle_area, get_bbox_angle, cap_number, Predictor

class ChasePersonTunablePredictor(Predictor):
  #Parameter choices are
  #min_bbox_area: minimum area where the robot has to go forward
  #max_bbox_area: maximum area where the robot has to go backwards
  #linear_mov_starter: starting velocity linear
  #decel_linear_mov: deceleration factor for linear movement
  #accel_linear_mov: acceleration factor for linear movement
  #linear_mov_addition: how much velocity we add or subtract when the ball is too close or too far
  #tolerance_angle: how much angle is tolerated to be the center
  #rotation_starter: how much ahould be the initial rotation
  #rotation_increment: how much should the rotation increase or decrease
  #rotation_unknown_starter: how much should the rotation start if the ball is not seen [NOT USED HERE]
 
  def __init__(self,param_choice, min_arr=[-1,-1,-2], max_arr=[1,1,2]):
    super().__init__(min_arr, max_arr)
    self.param_choice = param_choice 
    self.last_valid_angle = None
    self.last_angular_movement = None
    self.last_linear_movement = None

  def _get_linear_prediction(self, area):
    print("Area: ", area)
    if self.last_linear_movement == None:
      if area >= self.param_choice["min_bbox_area"] and area <= self.param_choice["max_bbox_area"]:
        return 0
      elif area <= self.param_choice["min_bbox_area"]:
        #Ball is very far
        return self.param_choice["linear_mov_starter"] 
      else:
        return 0
    else:
      if area >= self.param_choice["min_bbox_area"] and area <= (self.param_choice["max_bbox_area"] + self.param_choice["min_bbox_area"]) / 2:
        return self.last_linear_movement * self.param_choice["decel_linear_mov"]
      elif area >= (self.param_choice["max_bbox_area"] + self.param_choice["min_bbox_area"]) / 2 and area <= self.param_choice["max_bbox_area"]:
        return self.last_linear_movement * self.param_choice["accel_linear_mov"]
      elif area <= self.param_choice["min_bbox_area"]:
      #Ball is very far
        return self.last_linear_movement + self.param_choice["linear_mov_addition"]
      elif area >= self.param_choice["max_bbox_area"]:
      #Ball is very close
        return 0


  def _get_rot_prediction(self, angle):
    #Basically add rotation 
    if self.last_angular_movement == None:
      if abs(angle) <= self.param_choice["tolerance_angle"]:
        return 0
      else:
        return -self.param_choice["rotation_starter"] * angle
    else:
      print(angle)
      if abs(angle) <= self.param_choice["tolerance_angle"]:
        return self.last_angular_movement * self.param_choice["decel_rot_movement"]
      else:
        return self.last_angular_movement - self.param_choice["rotation_increment"] * angle
    

  #Predicts linear and angular velocities from a row made by conf, bb_x1, bb_y1, bb_x2, bb_y2
  def predict(self,x):
    #If we see the ball -> We can check the first number of the bbox as -1
    bbox_area = get_rectangle_area(x)
    #Idea: Very simple predictor -> Add and subtract angular velocity w.r.t the current velocity using the angle of the prediction. With linear velocity subtract if the area is too big, go forward when it is too small
    if x[1] == -1:
      #If we are not seeing the ball -> Linear velocity gets put to 0
      linear_prediction = 0
      #For the rot prediction, look at the last valid angle
      if self.last_valid_angle == None:
        rot_prediction = 0
    else:
      angle = get_bbox_angle(x)
      self.last_valid_angle = angle
      rot_prediction = self._get_rot_prediction(angle)
      if is_bbox_on_the_edge(x):
        linear_prediction = 0
      else:
        linear_prediction = self._get_linear_prediction(bbox_area)
    linear_prediction = cap_number(linear_prediction, self.min_arr[0],self.max_arr[0])
    rot_prediction = cap_number(rot_prediction,self.min_arr[2], self.max_arr[2])
    self.last_linear_movement = linear_prediction
    self.last_angular_movement = rot_prediction
    return [linear_prediction, 0, rot_prediction]
