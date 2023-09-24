


"""
correspondences is a dictionary with both pixel information,
as well as depth (for basic rendering)

coordinates[point_idx] = [ (x,y), z ]

image_size is a tuple (dimensions)

colors is a list where colors[point_idx] = [r,g,b]

"""
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
