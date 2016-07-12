from stl import mesh
import math
import numpy as np

def meshgen(filename):
	new_mesh = mesh.Mesh.from_file(filename)
	return new_mesh


mesh1 = meshgen('20mmbox.stl')
#for i in range(len(mesh1)):
#    print mesh1[i]
mesh1.update_normals 
mesh1.update_areas

mesh_norm = mesh1.normals 
mesh_areas = mesh1.areas
mesh_points = mesh1.points
mesh_vectors = mesh1.vectors

z_points = mesh1.z
points = mesh_points.tolist()

def find_layers(filename,layer_height):
    mesh = meshgen(filename)
    index = len(mesh.vectors[:,:,2])
    heights = mesh.vectors[index-1,:,2]
    abs_height = max(heights)
    layers = np.arange(0,abs_height+layer_height,layer_height,float)
    return layers
    
height = find_layers('Typhoon_fixed_Cut_1.stl',0.25)


	





