from stl import mesh
import math
import numpy as np
import numpy.linalg as la
import collections
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
#from meshcut import meshcut
'''
This set of functions is used to process meshes, from STL to a mesh. From here, transformations can be applied to the mesh - scale, transform, and rotation transformations
are currently supported across all axis (axis'...?). Once these transformations are completed, call slice_mesh to turn the mesh into a set of profiles at each layer which can be 
used for Gcode generation. Gcode generation is not done in this file - merely the preparation for Gcode generation.

Mplot3d has been included to allow for viewing of the meshes. Notice that each function calls for mesh_vertices. Mesh vertices is the optimal format, as it gives 
each triangle in the mesh as a seperate entry in a 3x3xN array - where N is the amount of triangles. This means each "entry" contains 3 points that define the vertices 
of a triangle. mesh.points will return an array of points that is only 2 dimensional, but this becomes MUCH harder to iterate through when dealing with large meshes. 
mesh.properties returns useful properties like the CoG, volume, and triangle count for each mesh. Poly reduction is planned, as high-poly models are actually worse for
Gcode generation, but is not currently implemented. Additional documentation will be provided when this is integrated. Most likely as an intermediary step between the 
numpy-stl library and this function set.

Function list:

mesh = meshgen(filename) - returns a mesh to handle mesh, from the numpy-stl library, processed from "filename". Give this as a string, and be sure to include the .stl ext
    Attributes of mesh:
    mesh.vertices - 3x3xN array where each 3x3 entry is the vertices of a triangle and N is the total amount of triangles in the mesh
    mesh.points - Not used here, but I believe that each row is equivalent to the vertices of one triangle and as such the amount of rows is the total triangle amount
    mesh.normals - normal vectors for the given mesh, given as a 3xN array where N is the amount of triangles
    mesh.get_mass_properties - entry 0 is the volume. entry 1 is the CoG. Entries 2-5 are the inertia matrices for each axis, but these are unused here.
    mesh.x, mesh.y, mesh.z - returns x,y,z points, respectively. Nx3 - N is amount of triangles, 3 because this is the x/y/z values for each individual triangle.
                                Imagine this as slicing the mesh.vertices bit by each coordinate axis, and that's effectively how this works.

point_to_plant_dist(p,plane) - returns the distance from the point p to the plane

Transformation functions - These take the vertices of our mesh as input, and return vertices as output. You can choose to overwrite the input with the new output values, which is 
usually ideal, or can create new sets of vertices. 

find_layers(mesh_vertices,layer_height) - given the mesh.vertices from a mesh created with meshgen, find the layer heights when we use the height layer_height.
    Its important to note that this is inherently not entirely accurate - its rare that our height will be a multiple of the layer_height, so we isntead round
    and go one layer higher than we need to most of the time. Results in slight approximation, but shouldn't result in anything crazy.

rotate_mesh(mesh_vertices,axes) - rotates a mesh theta RADIANS about the given axis. Only rotates about one axis at a time, for now. Specify x,y,z as a string 'x' 'y' 'z'

transform_mesh(mesh_vertices,x,y,z) - linearly transforms a distance x,y,z along each axis. So to move 20 units in x, -10 units in Y, and 0 units in Z, you would call
    mesh_vertices = transform_mesh(mesh_vertices,20,-10,0) - this will, by the way, overwrite your previous mesh with new mesh_vertices. If you want to keep the original,
    just use a new name like mesh_vertices2 or something. Since the structure was already created, it will return the same structure.
    You can also just transform one axis at a time by leaving fields as 0 if unused.

scale_mesh(mesh_vertices,x,y,z) - scales the mesh in the x,y,z directions by the scaling factor specified in the respective fields. 

unified_scale(mesh_vertices,scale_factor) - scales the whole mesh uniformly by the factor given by scale_factor
'''

class r_mesh(object): ## This class is for meshes run through the stl converter, and allows easy access to parameters we need (short for ready_mesh)
    def __init__(self,filename,vertices,normals,properties):
        self.filename = filename
        mesh = meshgen(filename)
        self.vertices = mesh.vertices
        self.normals = mesh.normals
        self.properties = mesh.get_mass_properties

class cut_plane(object): ## Class defining our plane for cutting the mesh. We want the normals as this is what we will use like a unit vector during iteration
    def __init__(self,origin,normals):
        self.origin = origin
        self.n = normals / la.norm(normals)

    def __str__(self):
        return 'plane(o=%s, n=%s)' % (self.origin,self.n)

def point_to_plane_dist(p,plane):
    return np.dot((p-plane.origin),plane.n)

# def tri_intersects_plane():
#     return 

def meshgen(filename):
	new_mesh = mesh.Mesh.from_file(filename)
	return new_mesh

def find_layers(mesh_vertices,layer_height):
    mesh = meshgen(filename)
    index = len(mesh.vectors[:,:,2])
    heights = mesh.vectors[index-1,:,2]
    abs_height = max(heights)
    layers = np.arange(0,abs_height+layer_height,layer_height,float)
    return layers
    
## Spec from MATLAB - matlab_processor(f,v,n,layers)
## F = vertices V = normals N = points layers = z coordinates for slices
## This function only returns the profiles of each layer of the file being sliced. It does not generate paths or anything of the like   
# def slice_model(filename,layer_height):
#     #   vertices2 = vertices[~normals,:] ??
#     # we generate the layers in this script as well, just pass height
#     layers = find_layers(filename,layer_height)
#     mesh = mesh.Mesh.from_file(filename)
#     vertices = mesh.vectors
#     normals = mesh.normals
#     i = 0
#     for i in range(len(layers)):
#         h = i / layer_height
        
def rotate_mesh(mesh_vertices,theta,axes):
    # Function to rotate a mesh
    # About Z: [ cos -sin 0; sin cos 0; 0 0 1]
    # About Y: [cos 0 sin; 0 1 0; -sin 0 cos]
    # About X: [ 1 0 0; 0 cos -sin; 0 sin cos]
    # The above is the transformation matrix for rotating a matrix about the specified axis.
    # Iterate in two steps: first through each triangle in vertices (to apply transform to whole mesh), and
    # within this loop further iterate through each vertice (the rows) to transform the whole triangle
    # Output is rotated by theta, in RADIANS
    if axes is 'x':
        length = mesh_vertices.shape
        leng = length[0]
        rotation_matrix = np.array([[1,0,0],[0,np.cos(theta),-1*np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
        for i in range(leng):
            j = 0
            for j in range(length[1]):
                mesh_vertices[i,j,:] = np.dot(mesh_vertices[i,j,:],rotation_matrix)
        return mesh_vertices
    elif axes is 'y':
        length = vertices.shape
        leng = length[0]
        rotation_matrix = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-1*np.sin(theta),0,np.cos(theta)]])
        for i in range(leng):
            j = 0
            for j in range(length[1]):
                mesh_vertices[i,j,:] = np.dot(mesh_vertices[i,j,:],rotation_matrix)
        return mesh_vertices
    elif axes is 'z':
        length = vertices.shape
        leng = length[0]
        rotation_matrix = np.array([[np.cos(theta),-1*np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
        for i in range(leng):
            j = 0
            for j in range(length[1]):
                mesh_vertices[i,j,:] = np.dot(mesh_vertices[i,j,:],rotation_matrix)
        return mesh_vertices
        
def transform_mesh(mesh_vertices,x,y,z): # Given a transformation amount for X,Y,Z, transform the mesh in the given direction the given amount.
    leng = mesh_vertices.shape[0]
    i = 0
    for i in range(leng):
        j = 0
        for j in range(mesh_vertices.shape[1]):
            mesh_vertices[i,j,0] += x
            mesh_vertices[i,j,1] += y
            mesh_vertices[i,j,2] += z

    return mesh_vertices
    
def scale_mesh(mesh_vertices,x,y,z): # Does what it says on the tin. Scales the shape in directions independently.
    leng = mesh_vertices.shape[0]
    i = 0
    for i in range(leng):
        j = 0
        for j in range(mesh_vertices.shape[1]):
            mesh_vertices[i,j,0] = np.multiply(x,mesh_vertices[i,j,0])
            mesh_vertices[i,j,1] = np.multiply(y,mesh_vertices[i,j,1])
            mesh_vertices[i,j,2] = np.multiply(z,mesh_vertices[i,j,2])
    return mesh_vertices

def unified_scale(mesh_vertices,scale_factor): # This will uniformly scale the mesh the same amount in all directions, as specified by scale_factor
    leng = mesh_vertices.shape[0]
    i = 0
    for i in range(leng):
        j = 0
        for j in range(mesh_vertices.shape[1]):
            mesh_vertices[i,j,:] = np.multiply(scale_factor,mesh_vertices[i,j,:])
    return mesh_vertices
## Generate test mesh
test = mesh.Mesh.from_file('Typhoon_fixed_Cut_1.stl')

normals = test.normals
points = test.points
vertices = test.vectors
faces = test.areas


# ## Plot test to make sure stl parser is actually working

# figure = plt.figure()
# axes = mplot3d.Axes3D(figure)
# axes.add_collection3d(mplot3d.art3d.Poly3DCollection(vertices))
# scale = test.points.flatten(-1)
# axes.auto_scale_xyz(scale,scale,scale)
   

# ## Testing rotation, scaling, and transforming
# vertices2 = scale_mesh(vertices,3,3,3)        
# vertices2 = rotate_mesh(vertices, 3.14, 'x')            
# figure2 = plt.figure()
# axes = mplot3d.Axes3D(figure2)
# axes.add_collection3d(mplot3d.art3d.Poly3DCollection(vertices2))
# scale = test.points.flatten(-1)
# axes.auto_scale_xyz(scale,scale,scale)
        
# plt.show(figure)



