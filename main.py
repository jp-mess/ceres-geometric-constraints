import geometry_utils
import image_utils
import lesser_utils
import os
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

def bmw_experiment(root_dir,input_cloud=None,output_cloud_dir=None):
  n_cameras = 3
  
  img_dim = 600
  cx = img_dim // 2
  cy = img_dim // 2
  foc = 525
  camera_parameters = lesser_utils.create_pinhole(foc,foc,cx,cy)

  n_points = 500000 // 30
  camera_radius = 5

  # to find the center of the pcd, use CloudCompare or something
  center = np.array([-9.62, -0.608, -1.345])
  pcd = o3d.io.read_point_cloud(input_cloud)

  # point N cameras at the center of the point cloud (all around the same point on the sphere,
  # with slight variation)  
  up_direction = "y"
  cameras = list()
  initial_camera = geometry_utils.make_camera(center,camera_radius,up_direction)
  cameras.append(lesser_utils.package_camera(initial_camera, camera_parameters, 'camera_0'))
  for i in range(1,n_cameras):
    nearby_camera = geometry_utils.make_camera(center,camera_radius,up_direction,initial_camera)
    cameras.append(lesser_utils.package_camera(nearby_camera, camera_parameters, 'camera_' + str(i)))
  
  # subsample point cloud indices
  indices = np.arange(len(pcd.points))
  indices = np.random.permutation(indices)
  indices = indices[:n_points]

  # correspondences[point_idx] = [(x,y), z] where (x,y) is a pixel coord and z is depth
  correspondences = image_utils.rasterize(cameras=cameras,indices_to_project=indices,
                                                          points=np.array(pcd.points),
                                                          colors=pcd.colors)
  
  images = image_utils.render_all_images(correspondences, np.array(pcd.points), pcd.colors, img_dim)

  geometry_utils.retriangulate(cameras, correspondences, np.array(pcd.points), noise_scale=0.0, pairwise=False, save_dir=output_cloud_dir)
  
  for image in images:
    plt.imshow(image)
    plt.show()


if __name__ == "__main__":
  # where to store and generate data (ideally not in the WSL) 
  root_dir = '/mnt/c/Users/Jack/Documents/pc_data'

  input_cloud = os.path.join(root_dir,'source_point_clouds','bmw.ply')
  output_cloud_dir = os.path.join(root_dir,'output_clouds')

  bmw_experiment(root_dir, input_cloud=input_cloud, output_cloud_dir=output_cloud_dir)

  


