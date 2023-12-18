
def create_pinhole(fx,fy,cx,cy):
  import numpy as np
  camera_params = np.eye(3)
  camera_params[0,0] = fx
  camera_params[1,1] = fy
  camera_params[0,2] = cx
  camera_params[1,2] = cy
  return camera_params

def create_pose_matrix(rotation_matrix, position):
  import numpy as np
  pose_matrix = np.eye(4)
  pose_matrix[:3,:3] = rotation_matrix
  pose_matrix[:3,3] = position
  return pose_matrix

def get_rot_trans(cam):
  R = cam[:3,:3]
  T = cam[:3,3].reshape((3,1))
  return R,T

def package_camera(extrinsics,intrinsics,name):
  camera_obj = dict()
  camera_obj['extrinsic'] = extrinsics
  camera_obj['intrinsic'] = intrinsics
  camera_obj['name'] = name
  return camera_obj
