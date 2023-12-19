
def bmw_bundle_adjustment_experiment(root_dir, input_cloud=None, output_cloud_dir=None, output_name = "ba_problem_input.txt"):
  import geometry_utils
  import image_utils
  import general_utils
  import numpy as np
  import open3d as o3d
  import matplotlib.pyplot as plt
  import os
   
  n_cameras = 3
  
  img_dim = 600
  cx = img_dim // 2
  cy = img_dim // 2
  foc = 525
  camera_parameters = general_utils.create_pinhole(foc,foc,cx,cy)

  n_points = 500000 // 30
  camera_radius = 5


  # bundle adjustment noise parameters
  pix_noise = 0.0
  pos_noise = 0.5
  rot_noise = 0.5


  # to find the center of the pcd, use CloudCompare or something
  center = np.array([-9.62, -0.608, -1.345])
  pcd = o3d.io.read_point_cloud(input_cloud)

  # point N cameras at the center of the point cloud (all around the same point on the sphere,
  # with slight variation)  
  up_direction = "y"
  cameras = list()

  cameras = geometry_utils.make_cameras(center, camera_radius, up_direction)
  cameras = [general_utils.package_camera(camera, camera_parameters, 'camera_' + str(i)) for i, camera in enumerate(cameras)]
 
  #initial_camera = geometry_utils.make_camera(center,camera_radius,up_direction)
  #cameras.append(general_utils.package_camera(initial_camera, camera_parameters, 'camera_0'))
  #for i in range(1,n_cameras):
  #  nearby_camera = geometry_utils.make_camera(center,camera_radius,up_direction,initial_camera)
  #  cameras.append(general_utils.package_camera(nearby_camera, camera_parameters, 'camera_' + str(i)))
  
  # subsample point cloud indices
  indices = np.arange(len(pcd.points))
  indices = np.random.permutation(indices)
  indices = indices[:n_points]

  # correspondences[point_idx] = [(x,y), z] where (x,y) is a pixel coord and z is depth
  correspondences = image_utils.rasterize(cameras=cameras,indices_to_project=indices,
                                                          points=np.array(pcd.points),
                                                          colors=pcd.colors)
  output_dir = "problem_encodings"
  output_file = os.path.join(output_dir,output_name)
  geometry_utils.create_bal_problem_file(correspondences, n_cameras, np.array(pcd.points), cameras, output_file,
                                         pixel_noise_scale=pix_noise, translation_noise_scale=pos_noise, rotation_noise_scale=rot_noise)


def bmw_retriangulation_experiment(root_dir,input_cloud=None,output_cloud_dir=None,plot=True):
  import geometry_utils
  import image_utils
  import general_utils
  import numpy as np
  import open3d as o3d
  import matplotlib.pyplot as plt

  n_cameras = 10
  
  img_dim = 600
  cx = img_dim // 2
  cy = img_dim // 2
  foc = 525
  camera_parameters = general_utils.create_pinhole(foc,foc,cx,cy)

  n_points = 500000 // 30
  camera_radius = 5

  # to find the center of the pcd, use CloudCompare or something
  center = np.array([-9.62, -0.608, -1.345])
  pcd = o3d.io.read_point_cloud(input_cloud)

  # point N cameras at the center of the point cloud (all around the same point on the sphere,
  # with slight variation)  
  up_direction = "y"
  cameras = list()
  cameras = geometry_utils.make_cameras(center, camera_radius, up_direction, n_cameras, close=False)
  cameras = [general_utils.package_camera(camera, camera_parameters, 'camera_' + str(i)) for i, camera in enumerate(cameras)]

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

  import frustum_visualizer
  visualizer = frustum_visualizer.PointCloudCameraVisualizer(pcd, cameras, center)
  visualizer.visualize()

  #if plot: 
  #  for image in images:
  #    plt.imshow(image)
  #    plt.show()

  return camera_parameters, images




