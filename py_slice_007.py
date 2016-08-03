from numpy_stl import mesh
import math
import numpy as np
import collections
from matplotlib import pyplot
from mpl_toolkits import mplot3d
#from meshcut import meshcut
'''
Description:
    This set of functions is used to process meshes, from STL to a mesh. From here, transformations can be applied to the mesh - scale, transform, and rotation transformations
    are currently supported across all axis (axis'...?). Once these transformations are completed, call slice_mesh to turn the mesh into a set of profiles at each layer which can be 
    used for Gcode generation. Gcode generation is not done in this file - merely the preparation for Gcode generation.

    Mplot3d has been included to allow for viewing of the meshes. Each function calls for the mesh alone in its definition, but will call for mesh.vectors in the function
    itself. This returns each triangle in the mesh as a seperate entry in a 3x3xN array - where N is the amount of triangles. This means each "entry" contains 3 points that 
    define the vertices of a triangle. mesh.points will return an array of points that is only 2 dimensional, but this becomes MUCH harder to iterate through when dealing 
    with large meshes. mesh.properties returns useful properties like the CoG, volume, and triangle count for each mesh. Poly reduction is planned, as high-poly models are 
    actually worse for Gcode generation, but is not currently implemented. Additional documentation will be provided when this is integrated. Most likely as an intermediary 
    step between the numpy-stl library and this function set.

    External libraries used - Numpy-stl. Documentation found at http://numpy-stl.readthedocs.io/en/latest/

(core) Function list:

    mesh = meshgen(filename) - returns a mesh to handle mesh, from the numpy-stl library, processed from "filename". Give this as a string, and be sure to include the .stl ext
        Attributes of mesh:
        mesh.vectors - 3x3xN array where each 3x3 entry is the vertices of a triangle and N is the total amount of triangles in the mesh
        mesh.points - Not used here, but I believe that each row is equivalent to the vertices of one triangle and as such the amount of rows is the total triangle amount
        mesh.normals - normal vectors for the given mesh, given as a 3xN array where N is the amount of triangles
        mesh.get_mass_properties - entry 0 is the volume. entry 1 is the CoG. Entries 2-5 are the inertia matrices for each axis, but these are unused here.
        mesh.x, mesh.y, mesh.z - returns x,y,z points, respectively. Nx3 - N is amount of triangles, 3 because this is the x/y/z values for each individual triangle.
                                Imagine this as slicing the mesh.vectors bit by each coordinate axis, and that's effectively how this works.

    point_to_plant_dist(p,plane) - returns the distance from the point p to the plane. unused yet.

Transformation functions - These take the vertices of our mesh as input, and return vertices as output. You can choose to overwrite the input with the new output values, which is 
usually ideal, or can create new sets of vertices. 

    find_layers(mesh,layer_height) - given the mesh handle from a mesh created with meshgen, find the layer heights when we use the height layer_height.
        Its important to note that this is inherently not entirely accurate - its rare that our height will be a multiple of the layer_height, so we isntead round
        and go one layer higher than we need to most of the time. Results in slight approximation, but shouldn't result in anything crazy.

    rotate_mesh(mesh,axes) - REMOVED. use mesh.rotate(axis,theta,point) instead. Axis is either single or multiple axis. Theta is in RADIANS. Point is rotation anchor.
                                        ^ numpy-stl method is faster and better integrated. also calculates a rotation matrix on-the-fly for multiple axis at once

    transform_mesh(mesh,x,y,z) - linearly transforms a distance x,y,z along each axis. So to move 20 units in x, -10 units in Y, and 0 units in Z, you would call
        mesh_vertices = transform_mesh(mesh_vertices,20,-10,0) - this will, by the way, overwrite your previous mesh with new values.

    scale_mesh(mesh,x,y,z) - scales the mesh in the x,y,z directions by the scaling factor specified in the respective fields. 

    unified_scale(mesh,scale_factor) - scales the whole mesh uniformly by the factor given by scale_factor
'''

def point_to_plane_dist(p,plane):
    return np.dot((p-plane.origin),plane.n)

def meshgen(filename):
	new_mesh = mesh.Mesh.from_file(filename)
	return new_mesh

# def find_layers(mesh_vertices,layer_height):
#     mesh = meshgen(filename)
#     index = len(mesh.vectors[:,:,2])
#     heights = mesh.vectors[index-1,:,2]
#     abs_height = max(heights)
#     layers = np.arange(0,abs_height+layer_height,layer_height,float)
#     return layers     
        
def transform_mesh(mesh_in,x,y,z): # Given a transformation amount for X,Y,Z, transform the mesh in the given direction the given amount.
    leng = mesh_in.vectors.shape[0]
    i = 0
    for i in range(leng):
        j = 0
        for j in range(mesh_in.vectors.shape[1]):
            mesh_in.x += x
            mesh_in.y += y
            mesh_in.z += z

    return mesh_in
    
def scale_mesh(mesh_in,x,y,z): # Does what it says on the tin. Scales the shape in directions independently.
    leng = mesh_in.vectors.shape[0]
    i = 0
    for i in range(leng):
        j = 0
        for j in range(mesh_in.vectors.shape[1]):
            mesh_in.x = np.multiply(x,mesh_in.x)
            mesh_in.y = np.multiply(y,mesh_in.y)
            mesh_in.z = np.multiply(z,mesh_in.z)
    return mesh_in

def unified_scale(mesh_in,scale_factor): # This will uniformly scale the mesh the same amount in all directions, as specified by scale_factor
    leng = mesh_in.vectors.shape[0]
    i = 0
    for i in range(leng):
        j = 0
        for j in range(mesh_in.vectors.shape[1]):
            mesh_in.vertices[i,j,:] = np.multiply(scale_factor,mesh_in.vertices[i,j,:])
    return mesh_in

#
test = meshgen('20mmbox.stl')
normals = test.normals
vectors = test.vectors
points = test.points
#inv_normals = np.logical_not(normals)
#print inv_normals
#vertices = test.vectors
#vertices2 = vertices[inv_normals,:]
# indices = range(2)
# test.vectors = test.vectors.take(indices,0,mode='clip')
#def slice_mesh(mesh_in,layer_height):
#    vertices = mesh_in.vectors
#    normals = mesh_in.normals
#    inv_normals = np.logical_not(normals)
#    vertices_2 = vertices[inv_normals,:]
  
  
# # Create a new plot
#figure = pyplot.figure()
#axes = mplot3d.Axes3D(figure)
#
# # Render the cube
#axes.add_collection3d(mplot3d.art3d.Poly3DCollection(vertices2))
#
# # Auto scale to the mesh size
#scale = test.points.flatten(-1)
#axes.auto_scale_xyz(scale, scale, scale)
#
# # Show the plot to the screen
#pyplot.show()
