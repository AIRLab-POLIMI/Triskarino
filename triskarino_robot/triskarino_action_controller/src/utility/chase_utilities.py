#!/usr/bin/env python3

def is_bbox_on_the_edge(bbox,max_w=640,max_h=320):
  if bbox[0] == 0 or bbox[1] == 0 or bbox[2] == max_w or bbox[3] == max_h:
    return True
  else:
    return False

def get_rectangle_area(bbox):
  return (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])

def get_bbox_angle(bbox,H_FOV=62.2,img_width=640):
  angle_min = ((bbox[0] / img_width) * H_FOV) - (H_FOV / 2)
  angle_max = ((bbox[2] / img_width) * H_FOV) - (H_FOV / 2)
  return (angle_max + angle_min) / 2

#Caps the predicted velocity to either the max or the min
def cap_number(num,min_num,max_num):
  if num < min_num:
    return min_num
  elif num > max_num:
    return max_num
  else:
    return num


#Template object to implement velocity command predictors given the input vector x composed of the bbox prediction of YOLO
class Predictor():

  def __init__(self,min_arr,max_arr):
    self.min_arr = min_arr
    self.max_arr = max_arr