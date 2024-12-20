import pymeshlab
import argparse


def calculate_inertial_tag(filepath=None, mass=-1, precision=8, scale_factor=1.0):
    ms = pymeshlab.MeshSet()

    ms.load_new_mesh(filepath)

    if mass < 0:
        raise Exception("Negative mass argument")

    print('Calculating the center of mass')
    geom = ms.get_geometric_measures()
    com = geom['barycenter']

    print('Scaling the mesh')
    ms.compute_matrix_from_scaling_or_normalization(axisx=scale_factor, axisy=scale_factor, axisz=scale_factor)

    print('Generating the convex hull of the mesh')
    ms.generate_convex_hull()  # TODO only if object is not watertight

    print('Calculating intertia tensor')
    geom = ms.get_geometric_measures()
    volume = geom['mesh_volume']
    tensor = geom['inertia_tensor'] / pow(scale_factor, 2) * mass / volume

    intertial_xml = f'<inertial>\n  <origin xyz="{com[0]:.{precision}f} {com[1]:.{precision}f} {com[2]:.{precision}f}"/>\n  <mass value="{mass:.{precision}f}"/>\n  <inertia ixx="{tensor[0, 0]:.{precision}f}" ixy="{tensor[1, 0]:.{precision}f}" ixz="{tensor[2, 0]:.{precision}f}" iyy="{tensor[1, 1]:.{precision}f}" iyz="{tensor[1, 2]:.{precision}f}" izz="{tensor[2, 2]:.{precision}f}"/>\n</inertial>'
    print(intertial_xml)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calculates inertial matrix for 3D Mesh. Takes as input mass, filepath and desired scale factor and returns as a string the xml inertial tag to put in the urdf.')
    parser.add_argument("filepath", help="Path of the 3D Mesh file", action="store",type=str)
    parser.add_argument("mass", help="Mass of the 3D Mesh", action="store",type=float)
    parser.add_argument("-s", "--scale_factor", help="Scale factor used to scale the 3D Mesh. Default value is 1.", default=1.0, action="store",type=float)
    parser.add_argument("-pr", "--precision", help="Desired precision for inertia factors. Default value is 8.",default=8,  action="store",type=int)
    arguments = parser.parse_args()
    calculate_inertial_tag(**vars(arguments))
