
def bmw_bundle_adjustment_experiment(root_dir, input_cloud=None, output_cloud_dir=None, visualize_frustums=False):
  import geometry_utils
  import image_utils
  import general_utils
  import numpy as np
  import open3d as o3d
  import matplotlib.pyplot as plt
  import os
   
  n_cameras = 6
  
  img_dim = 600
  cx = img_dim // 2
  cy = img_dim // 2
  foc = 525
  camera_parameters = general_utils.create_pinhole(foc,foc,cx,cy)

  n_points = 500000 // 30
  camera_radius = 5


  # bundle adjustment noise parameters
  pix_noise = 0.0
  pos_noise = 1.0
  rot_noise = 0.0


  # to find the center of the pcd, use CloudCompare or something
  center = np.array([-9.62, -0.608, -1.345])
  pcd = o3d.io.read_point_cloud(input_cloud)

  # point N cameras at the center of the point cloud (all around the same point on the sphere,
  # with slight variation)  
  up_direction = "y"
  cameras = list()

  ring_params_file = "manifold_encodings/ring_params.txt"
  cameras = geometry_utils.make_cameras_on_ring(center, camera_radius, up_direction, n_cameras, ring_params_file=ring_params_file)
  cameras = [general_utils.package_camera(camera, camera_parameters, 'camera_' + str(i)) for i, camera in enumerate(cameras)]
 

  # subsample point cloud indices
  indices = np.arange(len(pcd.points))
  indices = np.random.permutation(indices)
  indices = indices[:n_points]

  # correspondences[point_idx] = [(x,y), z] where (x,y) is a pixel coord and z is depth
  correspondences = image_utils.rasterize(cameras=cameras,indices_to_project=indices,
                                                          points=np.array(pcd.points),
                                                          colors=pcd.colors)
  
  ba_encoding_file = "problem_encodings/ba_problem_input.txt"
  geometry_utils.create_bal_problem_file(correspondences, n_cameras, np.array(pcd.points), cameras, ba_encoding_file,
                                         pixel_noise_scale=pix_noise, translation_noise_scale=pos_noise, rotation_noise_scale=rot_noise)
  
  import ring_utils 
  ring_encoding_file = "problem_encodings/ring_problem_input.txt"
  ring_utils.update_bal_problem_with_ring_cameras(ring_params_file, ba_encoding_file, ring_encoding_file)
  print("residuals before ring projection")
  ring_utils.print_camera_residuals_from_file(ring_params_file, ba_encoding_file)
  print("and after")
  ring_utils.print_camera_residuals_from_file(ring_params_file, ring_encoding_file)
  
  geometry_utils.invert_camera_pose(ba_encoding_file, "problem_encodings/inverted_ba_problem.txt")
  geometry_utils.invert_camera_pose(ring_encoding_file, "problem_encodings/inverted_ring_problem.txt")

  if visualize_frustums: 
    import frustum_visualizer
    visualizer = frustum_visualizer.PointCloudCameraVisualizer(pcd, cameras, center)
    visualizer.visualize()


def bmw_retriangulation_experiment(root_dir,input_cloud=None,output_cloud_dir=None,plot_rendered_images=True, visualize_frustrums=True):
  import geometry_utils
  import image_utils
  import general_utils
  import numpy as np
  import open3d as o3d
  import matplotlib.pyplot as plt

  n_cameras = 3
  
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
  ring_params_file = "manifold_encodings/ring_params.txt"
  cameras = geometry_utils.make_cameras_on_ring(center, camera_radius, up_direction, n_cameras, ring_params_file=ring_params_file)
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

  if visualize_frustrums:
    import frustum_visualizer
    visualizer = frustum_visualizer.PointCloudCameraVisualizer(pcd, cameras, center)
    visualizer.visualize()

  if plot_rendered_images: 
    for image in images:
      plt.imshow(image)
      plt.show()

  return camera_parameters, images




