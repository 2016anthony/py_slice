from stl import mesh
import math
import numpy as np
import numpy.linalg as la
import collections
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
#from meshcut import meshcut

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
def slice_model(filename,layer_height):
    #   vertices2 = vertices[~normals,:] ??
    # we generate the layers in this script as well, just pass height
    layers = find_layers(filename,layer_height)
    mesh = mesh.Mesh.from_file(filename)
    vertices = mesh.vectors
    normals = mesh.normals
    i = 0
    for i in range(len(layers)):
        h = i / layer_height
        
def rotate_mesh(mesh_vertices,theta,axes):
    # Function to rotate a mesh
    # About Z: [ cos -sin 0; sin cos 0; 0 0 1]
    # About Y: [cos 0 sin; 0 1 0; -sin 0 cos]
    # About X: [ 1 0 0; 0 cos -sin; 0 sin cos]
    # Iterate in two steps: first through each vertices in vertices, and
    # within this loop further iterate through each row to transform.
    # Output is rotated by theta rad
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
        
def transform_mesh(mesh_vertices,x,y,z):
    leng = mesh_vertices.shape[0]
    i = 0
    for i in range(leng):
        j = 0
        for j in range(mesh_vertices.shape[1]):
            mesh_vertices[i,j,0] += x
            mesh_vertices[i,j,1] += y
            mesh_vertices[i,j,2] += z

    return mesh_vertices
    
def scale_mesh(mesh_vertices,x,y,z):
    leng = mesh_vertices.shape[0]
    i = 0
    for i in range(leng):
        j = 0
        for j in range(mesh_vertices.shape[1]):
            mesh_vertices[i,j,0] = np.multiply(x,mesh_vertices[i,j,0])
            mesh_vertices[i,j,1] = np.multiply(y,mesh_vertices[i,j,1])
            mesh_vertices[i,j,2] = np.multiply(z,mesh_vertices[i,j,2])
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



