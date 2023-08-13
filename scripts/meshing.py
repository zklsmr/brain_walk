import argparse

parser = argparse.ArgumentParser()
parser.add_argument('file')
parser.add_argument('out_dir')
args = parser.parse_args()

import numpy as np
import nibabel as nb
from stl import mesh
from skimage import measure

#load data
nii_img = nb.load(args.file)
nii_data = nii_img.get_fdata()

vertices,faces,_y,_u = measure.marching_cubes(nii_data, 0)


ThreeD_model = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
print("created ThreeD_model")

for i, j in enumerate(faces):
    ThreeD_model.vectors[i] = vertices[j]
print("populated ThreeD_model")


print(f"saving {args.file.split('.')[0].split('/')[-1]} to {args.out_dir}")
ThreeD_model.save(f"{args.out_dir}/3D_model_{args.file.split('.')[0].split('/')[-1]}.stl")


