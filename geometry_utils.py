
"""
given the center of the 3D model, sample a camera
around the upper hemisphere of it (so orbiting around it, 
but never under it)

if you specify an initial camera, a "nearby" camera (on the same 
sphere) will be sampled
"""
def make_camera(point_cloud_center,
                radius,
                up_direction,
                initial_camera=None):
  import lesser_utils
  import numpy as np

  if initial_camera is None:
    if up_direction == "y":
      theta = np.random.uniform(0, 2*np.pi/2)
      phi = np.random.uniform(0,np.pi/2)
      x = radius * np.sin(phi) * np.cos(theta)
      z = radius * np.sin(phi) * np.sin(theta)
      y = radius * np.cos(phi)
    else:
      raise NotImplementedError()

    camera_loc = np.array([x,y,z])
    camera_loc += point_cloud_center
  else:
    initial_loc = initial_camera[:3,3].flatten()
    initial_loc -= point_cloud_center
    camera_loc = nearby_coord(initial_loc)
    camera_loc += point_cloud_center

  center_proj_ray = point_cloud_center - camera_loc
  camera_direction = center_proj_ray / np.linalg.norm(center_proj_ray)
 
  if up_direction == "y":
    up_dir = np.array(np.array([0,1,0]))
    x_dir = np.cross(up_dir,camera_direction)
    x_dir = x_dir / np.linalg.norm(x_dir)
  
    y_dir = np.cross(x_dir, camera_direction)
    y_dir = y_dir / np.linalg.norm(y_dir)

    R_wc = np.stack((x_dir, y_dir, camera_direction), axis=1)

    rotation_matrix = R_wc

    pose_matrix = lesser_utils.create_pose_matrix(rotation_matrix, camera_loc)

  else:
    raise NotImplementedError()

  return pose_matrix

"""
assuming xyz is a vector centered at 0, sample a new coord
on the same sphere

up direction matters!
"""
def nearby_coord(xyz,variation=0.25,up_dir='z'):
  import numpy as np

  r = np.linalg.norm(xyz)
  theta = np.arccos(xyz[2] / r)
  phi = np.arctan2(xyz[1],xyz[0])

  theta += np.random.normal(0,variation)
  phi += np.random.normal(0,variation)
  
  x = r * np.sin(theta) * np.cos(phi)
  y = r * np.sin(theta) * np.sin(phi)
  z = r * np.cos(theta)

  return np.array([x,y,z])


"""
cameras : list of N cameras where each camera is a dictionary:

camera['camera'] = pose matrix (4x4)
camera['name'] = string identifier for the camera

pixels is a list of pixel coordinates, one for each camera

[(x1,y1), (x2,y2), ...(xN, yN)] in (row,column) format (not x,y)

pairwise will generate (N choose 2) points, multiview triangulates one 
""" 
def DLT(cameras,pixels,pairwise=True):
  import numpy as np
  import lesser_utils

  points = list()
  if pairwise:
    pairs = list(itertools.combinations(list(range(len(cameras))),2))
    
    for i,j in pairs:
      # [(x,y) z]
      point1 = pixels[i]
      point2 = pixels[j]
      
      camA = np.linalg.inv(cameras[i]['camera'])
      rotA, transA = get_rot_trans(camA)
      camB = np.linalg.inv(cameras[j]['camera'])
      rotB, transB = get_rot_trans(camB)
  
      camA = np.concatenate([rotA,transA],axis=-1)
      P1 = camera_params @ camA
      camB = np.concatenate([rotB,transB],axis=-1)
      P2 = camera_params @ camB

      A = [point1[1]*P1[2,:] - P1[1,:],
         P1[0,:] - point1[0]*P1[2,:],
         point2[1]*P2[2,:] - P2[1,:],
         P2[0,:] - point2[0]*P2[2,:]
        ]
      A = np.array(A).reshape((4,4))
      B = A.transpose() @ A
      from scipy import linalg
      U,s,Vh = linalg.svd(B,full_matrices=False)
      point = Vh[3,0:3]/Vh[3,3]
      points.append(point)
    return points
  else:
    A = list()
    for i in range(len(cameras)):
      # [(x,y) z]
      point = pixels[i]
      camA = np.linalg.inv(cameras[i]['extrinsic'])
      camera_params = cameras[i]['intrinsic']
      rotA, transA = lesser_utils.get_rot_trans(camA)
      camA = np.concatenate([rotA, transA], axis=-1)
      P = camera_params @ camA
      A_ = np.array([point[1]*P[2,:] - P[1,:],
                    P[0,:] - point[0]*P[2,:]])
      A.append(A_)
    A = np.concatenate(A,axis=0)
    B = A.transpose() @ A
    from scipy import linalg
    U, s, Vh = linalg.svd(B, full_matrices = False)
    point = Vh[3,0:3]/Vh[3,3]
    return [point] 


"""
cameras : list of N cameras where each camera is a dictionary:

camera['camera'] = pose matrix (4x4)
camera['name'] = string identifier for the camera

correspondences is a dictionary with both pixel information,
as well as depth (for basic rendering)

coordinates[point_idx] = [ (x,y), z ]

points is an np.array list of 3D point

noise_scale is how much gaussian noise to add to each pixel observation

pairwise = True will perform pairwise triangulation (one world point for
each view edge, while pairwise = False will do multiview triangulation
i.e. one world point for all N cameras that saw the same point_idx)
""" 
def retriangulate(cameras, correspondences, points, noise_scale=0.0, pairwise=True, save_dir=None):
  import numpy as np
  import open3d as o3d
  import os

  triangulated = list()
  errors = list()

  for point_idx in correspondences:
    # each coord is [(x,y) z] and we just want the (x,y) pixels
    pixels = [coords[0] for coords in correspondences[point_idx]]
    for pix_idx in range(len(pixels)):
      pixels[pix_idx] = pixels[pix_idx] + np.random.normal(0,noise_scale,size=2)
      
      result = DLT(cameras,pixels,pairwise=pairwise)
      triangulated.extend(result)
      
      err = np.mean([np.linalg.norm(p - points[point_idx,:]) for p in result])
      errors.append(err)

  triangulated = np.vstack(triangulated)
  triangulated = np.array([triangulated[i,:] for i in range(triangulated.shape[0])])
  new_pcd = o3d.geometry.PointCloud()
  new_pcd.points = o3d.utility.Vector3dVector(triangulated.squeeze())
  print(f"noise level {np.array(noise_scale)}\t\tn_cameras {len(cameras)}\t\tavg error {np.mean(errors)}\t\tmultiview? {not pairwise}")

  if save_dir is not None:
    name = "retriangulated_" + str(noise_scale) + "_" + str(len(cameras)) + "_cameras"
    if pairwise:
      name += str("_pairwise.ply")
    else:
      name += str("_multiview.ply")
    name = os.path.join(save_dir,name)
    o3d.io.write_point_cloud(name, new_pcd)

  return triangulated
