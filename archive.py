import itertools
import cv2
import matplotlib.pyplot as plt
import os,sys
import open3d as o3d
import numpy as np
from dataclasses import dataclass


#np.random.seed(55) # sideways tubes


img_dim = 600
ux = img_dim // 2
uy = img_dim // 2
foc = 525



diameter = 5
n_points = 500000 // 300


root_dir = '/mnt/c/Users/Jack/Documents/pc_data'
source = os.path.join(root_dir,'bmw.ply')
center = np.array([-9.62, -0.608, -1.345])
up = "y"
#center = np.array([-0.1,-0.32,0.647]) # racecar
#up = "z" # racecar

pcd = o3d.io.read_point_cloud(source)

print(pcd)

def nearby_coord(xyz):
  r = np.linalg.norm(xyz)
  theta = np.arccos(xyz[2] / r)
  phi = np.arctan2(xyz[1],xyz[0])

  theta += np.random.normal(0,0.25)
  phi += np.random.normal(0,0.25)
  
  x = r * np.sin(theta) * np.cos(phi)
  y = r * np.sin(theta) * np.sin(phi)
  z = r * np.cos(theta)

  return np.array([x,y,z])


def render_coordinates(coordinates, image_size, colors):
  image = np.zeros((image_size[0], image_size[1], 3))
  collisions = dict()
  for i,coord in enumerate(coordinates):
    x,y = coord[0]
    x = int(round(x))
    y = int(round(y))

    # color in if not viewed
    if (x,y) not in collisions:
      collisions[(x,y)] = coord[1]
      if 0 <= x < image_size[1] and 0 <= y < image_size[0]:
        image[y,x] = colors[i]
    else:
      if collisions[(x,y)] > coord[1]:
        collisions[(x,y)] = coord[1]
        if 0 <= x < image_size[1] and 0 <= y < image_size[0]:
          image[y,x] = colors[i]
  
  return image






def create_pose_matrix(rotation_matrix, position):
  pose_matrix = np.eye(4)
  pose_matrix[:3,:3] = rotation_matrix
  pose_matrix[:3,3] = position
  return pose_matrix


def make_camera(point_cloud_center, initial_camera=None):
  if initial_camera is None:
    r = diameter
    theta = np.random.uniform(0, 2*np.pi/2)
    phi = np.random.uniform(0,np.pi/2)
    x = r * np.sin(phi) * np.cos(theta)
    #y = r * np.sin(theta) * np.sin(phi)
    #z = r * np.cos(theta)
    z = r * np.sin(phi) * np.sin(theta)
    y = r * np.cos(phi)

    #if True:
    #  camera_loc = np.array([x,z,y])
    #else:
    camera_loc = np.array([x,y,z])
    camera_loc += point_cloud_center
  else:
    initial_loc = initial_camera[:3,3].flatten()
    initial_loc -= point_cloud_center
    camera_loc = nearby_coord(initial_loc)
    camera_loc += point_cloud_center

  print('dist to pc center',np.linalg.norm(camera_loc - point_cloud_center))
  center_proj_ray = point_cloud_center - camera_loc
  camera_direction = center_proj_ray / np.linalg.norm(center_proj_ray)
 
  if up == "y":
    up_dir = np.array(np.array([0,1,0]))
    x_dir = np.cross(up_dir,camera_direction)
    x_dir = x_dir / np.linalg.norm(x_dir)
  
    y_dir = np.cross(x_dir, camera_direction)
    y_dir = y_dir / np.linalg.norm(y_dir)

    R_wc = np.stack((x_dir, y_dir, camera_direction), axis=1)

    rotation_matrix = R_wc

    pose_matrix = create_pose_matrix(rotation_matrix, camera_loc)

  return pose_matrix

def rasterize(cameras,points_to_project,points,all_colors):
  pixels = list()
  colors = list()

  correspondences = dict()
  render_correspondences = dict()
  shifted_points = list()
  shifted_colors = list()

  for camera_obj in cameras:
    for point_idx in points_to_project:
      camera = camera_obj['camera']
      camera_center = camera[:3,3]
      
      point_hom = np.append(points[point_idx,:],1).reshape((4,1))
      point_hom = np.matmul(np.linalg.pinv(camera),point_hom)
      point_hom = point_hom[:3]

      shifted_points.append(point_hom)
      shifted_colors.append(all_colors[point_idx])

      pixel = np.matmul(camera_params,point_hom)
      pixel = pixel[:2] / pixel[2]
      pixel = pixel.flatten().astype(float)

      #print(pixel)

      pixels.append(pixel)
      colors.append(all_colors[point_idx])

      if point_idx not in correspondences:
        correspondences[point_idx] = list()
        render_correspondences[point_idx] = list()
    
      correspondences[point_idx].append(pixel)
      render_correspondences[point_idx].append((pixel,np.linalg.norm(point_hom)))
    
    name = camera_obj['name']
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(np.array(shifted_points).squeeze())
    new_pcd.colors = o3d.utility.Vector3dVector(np.array(shifted_colors).squeeze())
    o3d.io.write_point_cloud(os.path.join(root_dir,name + '.ply'), new_pcd)

  return correspondences, render_correspondences

def package_camera(camera,name):
  camera_obj = dict()
  camera_obj['camera'] = camera
  camera_obj['name'] = name
  return camera_obj

def get_images(correspondences, points, colors):
  images = dict()
  color_map = dict()
  for point_idx in correspondences.keys():
    for idx,pixels in enumerate(correspondences[point_idx]):
      if idx not in images:
        images[idx] = list()
        color_map[idx] = list()
      images[idx].append(pixels)
      color_map[idx].append(colors[point_idx])
  rendered = list()
  for idx in images.keys():
    rendered.append(render_coordinates(images[idx], (img_dim, img_dim), color_map[idx]))
  return rendered


def get_rot_trans(cam):
  R = cam[:3,:3]
  T = cam[:3,3].reshape((3,1))
  return R,T

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
      

def retriangulate(cameras, correspondences, points, noise_scale=0.0, pairwise=True):
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
  bias = np.random.normal(0,1,3)
  bias /= np.linalg.norm(bias)
  triangulated = np.array([triangulated[i,:] for i in range(triangulated.shape[0])])
  new_pcd = o3d.geometry.PointCloud()
  new_pcd.points = o3d.utility.Vector3dVector(triangulated.squeeze())
  print("avg error", np.array(errors).mean())
  name = 'retriangulated_' + str(noise_scale) + '_' + str(len(cameras)) + '_cameras_biased5'
  if pairwise:
    name += str('_pairwise.ply')
  else:
    name += str('_multiview.ply')
  name = os.path.join(root_dir,name)
  o3d.io.write_point_cloud(name, new_pcd)

  return triangulated

if __name__ == "__main__":

  n_cameras = 3
  cameras = list()
  initial_camera = make_camera(center)
  cameras.append(package_camera(initial_camera, 'camera_0'))
  for i in range(1,n_cameras):
    cameras.append(package_camera(make_camera(center,initial_camera),'camera_' + str(i)))
  
  camera_params = np.eye(3)
  camera_params[0,0] = foc
  camera_params[1,1] = foc
  camera_params[0,2] = ux
  camera_params[1,2] = uy


  indices = np.arange(len(pcd.points))
  indices = np.random.permutation(indices)
  indices = indices[:n_points]

  correspondences, render_correspondences = rasterize(cameras,indices,np.array(pcd.points),pcd.colors)

  retriangulate(cameras,correspondences,np.array(pcd.points),noise_scale=0.0,pairwise=False)

  render = False
  if render:
    rendered = get_images(render_correspondences, np.array(pcd.points), pcd.colors) 
    for rend in rendered:
      plt.imshow(rend)
      plt.show()
