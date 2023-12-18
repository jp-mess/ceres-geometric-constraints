
def create_bal_problem_file(correspondences, n_cameras, point_container, true_cameras, output_file, translation_noise_scale = 0.0, rotation_noise_scale = 0.0, pixel_noise_scale = 0.0):

    from scipy.spatial.transform import Rotation as R
    import sys
    import numpy as np
    import transforms3d

    n_points = len(correspondences)
    point_idx_mapping = {original_idx: new_idx for new_idx, original_idx in enumerate(correspondences.keys())}

    """
    print('rot matrix',true_cameras[0]['extrinsic'][:3,:3])
    rotation_matrix = true_cameras[0]["extrinsic"][:3, :3]
    print(np.linalg.norm(rotation_matrix))
    rotation = R.from_matrix(rotation_matrix)
    angle_axis = rotation.as_rotvec()  # Convert to angle-axis
    print('angle_axis', angle_axis)
    rot_mat = R.from_rotvec(angle_axis).as_matrix()
    print('decoded', rot_mat) 
    print(np.linalg.norm(rot_mat))
    """

    with open(output_file, 'w') as file:
        # Write the header
        num_observations = n_cameras * n_points
        file.write(f"{n_cameras} {n_points} {num_observations}\n")

        # Write the observations
        for original_idx, observations in correspondences.items():
            new_idx = point_idx_mapping[original_idx]
            for camera_idx in range(n_cameras):
                row, col = observations[camera_idx][0]  # Extract pixel coordinates
                noise = np.random.normal(0,pixel_noise_scale,size=2)
                row += noise[0]
                col += noise[1]
                file.write(f"{camera_idx} {new_idx} {row} {col}\n")

         # Write the true camera parameters
        for camera in true_cameras:

            camera["extrinsic"] = np.linalg.pinv(camera["extrinsic"])

            # Convert the rotation matrix to Euler angles (angle-axis)
            rotation_matrix = camera["extrinsic"][:3, :3]
            rotation = R.from_matrix(rotation_matrix)
            angle_axis = rotation.as_rotvec()  # Convert to angle-axis
            angle_axis += np.random.normal(0, rotation_noise_scale, 3)

            # Write the angle-axis rotation
            for i in range(3):
                file.write(f"{angle_axis[i]}\n")

            # Write the translation
            translation_vector = camera["extrinsic"][:3, 3]
            translation_vector += np.random.normal(0, translation_noise_scale, 3)
            for i in range(3):
                file.write(f"{translation_vector[i]}\n")

            # Write the focal length, cx, and cy from the intrinsic matrix
            intrinsic = camera["intrinsic"]
            file.write(f"{intrinsic[0, 0]}\n")  # Focal length
            file.write(f"{intrinsic[0, 2]}\n")  # cx
            file.write(f"{intrinsic[1, 2]}\n")  # cy

        # Write the true 3D points
        for original_idx in correspondences.keys():
            point = point_container[original_idx]
            for i in range(3):
                file.write(f"{point[i]}\n")

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
  import general_utils
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

    up_dir = np.array([0, 1, 0])
    z_dir = camera_direction  # Assuming this is the forward direction (Z axis)
    x_dir = np.cross(up_dir, z_dir)
    x_dir /= np.linalg.norm(x_dir)

    y_dir = np.cross(z_dir, x_dir)  # This ensures a right-handed coordinate system
    y_dir /= np.linalg.norm(y_dir)

    R_wc = np.stack((x_dir, y_dir, z_dir), axis=1)

    #up_dir = np.array(np.array([0,1,0]))
    #x_dir = np.cross(up_dir,camera_direction)
    #x_dir = x_dir / np.linalg.norm(x_dir)
  
    #y_dir = np.cross(x_dir, camera_direction)
    #y_dir = y_dir / np.linalg.norm(y_dir)

    #R_wc = np.stack((x_dir, y_dir, camera_direction), axis=1)

    rotation_matrix = R_wc

    pose_matrix = general_utils.create_pose_matrix(rotation_matrix, camera_loc)

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
  import general_utils

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
      rotA, transA = general_utils.get_rot_trans(camA)
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

coordinates[point_idx] = [ (x,y), depth ]

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
