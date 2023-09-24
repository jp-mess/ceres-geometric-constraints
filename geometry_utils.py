import numpy as np

def get_rot_trans(cam):
  R = cam[:3,:3]
  T = cam[:3,3].reshape((3,1))
  return R,T

"""
cameras : list of N cameras where each camera is a dictionary:

camera['camera'] = pose matrix (4x4)
camera['name'] = string identifier for the camera

pixels is a list of pixel coordinates, one for each camera

[(x1,y1), (x2,y2), ...(xN, yN)] in (row,column) format (not x,y)

pairwise will generate (N choose 2) points, multiview triangulates one 
""" 
def DLT(cameras,pixels,pairwise=True):
  points = list()
  if pairwise:
    pairs = list(itertools.combinations(list(range(len(cameras))),2))
    
    for i,j in pairs:
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
      point = pixels[i]
      camA = np.linalg.inv(cameras[i]['camera'])
      rotA, transA = get_rot_trans(camA)
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
def retriangulate(cameras, correspondences, points, noise_scale=0.0, pairwise=True, save_pcd=False):
  triangulated = list()
  errors = list()

  for point_idx in correspondences:
    pixels = correspondences[point_idx]
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
  print(f"noise level {np.array(errors)} n_cameras {len(cameras)} avg error {np.mean(errors)}")

  if save_pcd:
    name = "retriangulated_" + str(noise_scale) + "_" + str(len(cameras)) + "_cameras"
    if pairwise:
      name += str("_pairwise.ply")
    else:
      name += str("_multiview.ply")
    name = os.path.join(root_dir,name)
    o3d.io.write_point_cloud(name, new_pcd)

  return triangulated
