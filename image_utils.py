"""
wrapper function for getting a list of images from a correspondence 
object

correspondences[point_idx] = [ [(x,y),z], [(x,y),z]... ]

where each point_idx is the index of a unique point in the original point cloud

"""
def render_all_images(correspondences, points, colors, img_dim):
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


"""
get a list of image correspondences for every unique point
in the point cloud
"""
def rasterize(cameras,indices_to_project,points,colors):
  import numpy as np 

  pixels = list()
  correspondences = dict()

  for camera_obj in cameras:
    for point_idx in indices_to_project:
      pose = camera_obj['extrinsic']
      intrinsic_params = camera_obj['intrinsic']
      camera_center = pose[:3,3]
      
      point_hom = np.append(points[point_idx,:],1).reshape((4,1))
      point_hom = np.matmul(np.linalg.pinv(pose),point_hom)
      point_hom = point_hom[:3]

      pixel = np.matmul(intrinsic_params,point_hom)
      pixel = pixel[:2] / pixel[2]
      pixel = pixel.flatten().astype(float)

      pixels.append(pixel)

      if point_idx not in correspondences:
        correspondences[point_idx] = list()
    
      correspondences[point_idx].append((pixel,np.linalg.norm(point_hom)))
    
  return correspondences


"""
coordinates is a dictionary with both pixel information,
as well as depth (for basic rendering)

coordinates[point_idx] = [ (x,y), z ]

image_size is a tuple (dimensions)

colors is a list where colors[point_idx] = [r,g,b]

"""
def render_coordinates(coordinates, image_size, colors):
  import numpy as np

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
