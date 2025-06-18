import os
from urdf2mjcf import run

urdf_path = "deploy/assets/g1/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf"
mjcf_path = "resources/robots/g1_description/g1_body29_inspire.xml"

full_urdf_path = os.path.abspath(urdf_path)
full_mjcf_path = os.path.abspath(mjcf_path)

print(f"URDF Path: {full_urdf_path}")
print(f"MJCF Path: {full_mjcf_path}")

run(
    urdf_path=full_urdf_path,
    mjcf_path=full_mjcf_path,
    copy_meshes=True,
)