import numpy as np
import stl
from stl import mesh
import argparse
import os
import json
from jinja2 import FileSystemLoader, Environment

"""
For a SDF file we need,
* CoG
* Mass
* Inertia
* Bounding box (for collision)

Resources:
http://gazebosim.org/tutorials?tut=inertia
http://answers.gazebosim.org/question/18701/how-do-i-get-the-inertia-tensor-of-a-link-using-its-actual-density/
https://answers.ros.org/question/237421/calculating-inertial-matrix-for-gazebo/
"""

# find the max dimensions, so we can know the bounding box, getting the height,
# width, length (because these are the step size)...
def find_mins_maxs(obj):
    # This output matches that of meshlab
    minx = maxx = miny = maxy = minz = maxz = None
    for p in obj.points:
        # p contains (x, y, z)
        if minx is None:
            minx = p[stl.Dimension.X]
            maxx = p[stl.Dimension.X]
            miny = p[stl.Dimension.Y]
            maxy = p[stl.Dimension.Y]
            minz = p[stl.Dimension.Z]
            maxz = p[stl.Dimension.Z]
        else:
            maxx = max(p[stl.Dimension.X], maxx)
            minx = min(p[stl.Dimension.X], minx)
            maxy = max(p[stl.Dimension.Y], maxy)
            miny = min(p[stl.Dimension.Y], miny)
            maxz = max(p[stl.Dimension.Z], maxz)
            minz = min(p[stl.Dimension.Z], minz)
    return minx, maxx, miny, maxy, minz, maxz

def sdf_cog(cog):
    print ("<pose>{0:f} {1:f} {2:f}</pose>".format(*cog))

def sdf_inertia(i):
    """ SDF assumes symmetric inertia matrix so only 6 above-diagonal elements
    are used """
    print ("<inertia>\n\t<ixx>{0:f}</ixx>\n\t<ixy>{1:f}</ixy>\n\t<ixz>{2:f}</ixz>\n\t<iyy>{3:f}</iyy>\n\t<iyz>{4:f}</iyz>\n\t<izz>{5:f}</izz>\n</inertia>".format(*i[0,:], i[1,1], i[1,2], i[2,2]))


def render_from_template(directory, template_name, **kwargs):
    loader = FileSystemLoader(directory)
    env = Environment(loader=loader)
    template = env.get_template(template_name)
    return template.render(**kwargs)

def update_geometry(m):

    stl_file = m["stl"]
    obj = mesh.Mesh.from_file(stl_file)
    volume, cog, inertia = obj.get_mass_properties()
    m["cog"] = " ".join(["{:f}".format(_cog) for _cog in cog * m["stl_length_scale"]])

    print ("cog=", cog)
    i = inertia * np.power(m["stl_length_scale"], 2) * m["mass"]/volume
    print ("inertia=", i)
    #m["inertia"] = list(map(lambda x: "{:f}".format(x), [i[0, 0], i[0, 1], i[0, 2], i[1, 1], i[1,2], i[2,2] ] ))
    m["inertia"] = list(map(lambda x:  "{}".format(x) if x > 1e-9 else 0, [i[0, 0], i[0, 1], i[0, 2], i[1, 1], i[1,2], i[2,2] ] ))

def dump_sdf_parameters(stl_filepath, stl_length_scale, mass):
    """

    Args: 
        stl_length_scale: If using units not meters, scale to convert to meters
    """

    obj = mesh.Mesh.from_file(stl_filepath)
    volume, cog_raw, inertia = obj.get_mass_properties()
    cog = " ".join(["{:f}".format(_cog) for _cog in cog_raw * stl_length_scale])
    print ("Pose=", cog)

    i = inertia * np.power(stl_length_scale, 2) * mass/volume
    #print ("inertia=", i)

    # Get only the positive top right diagonal
    positive_inertia = list(map(lambda x:  "{}".format(x) if x > 1e-9 else 0, [i[0, 0], i[0, 1], i[0, 2], i[1, 1], i[1,2], i[2,2] ] ))

    print ("Inertia=", positive_inertia)

def gen(template_dir, aircraft_config):
    data = None
    with open(aircraft_config) as f:
        data = json.load(f)

    print("Geometry for Frame")
    dump_sdf_parameters("/home/jarrod/Documents/gymfc-digitaltwin-tinyhawk/models/tinyhawk/stl/Frame.stl", 0.001, 0.00612)

    print("Geometry for FC")
    dump_sdf_parameters("/home/jarrod/Documents/gymfc-digitaltwin-tinyhawk/models/tinyhawk/stl/Flight Controller.stl", 0.001, 0.00611)

    print("Geometry for battery")
    dump_sdf_parameters("/home/jarrod/Documents/gymfc-digitaltwin-tinyhawk/models/tinyhawk/stl/Battery.stl", 0.001, 0.01253)

    print("Geometry for battery holder")
    dump_sdf_parameters("/home/jarrod/Documents/gymfc-digitaltwin-tinyhawk/models/tinyhawk/stl/Battery Holder.stl", 0.001, 0.00076)

    print("Geometry for camera")
    dump_sdf_parameters("/home/jarrod/Documents/gymfc-digitaltwin-tinyhawk/models/tinyhawk/stl/Camera.stl", 0.001, 0.002)

    print("Geometry for motors")
    for m in data["motors"]:
        dump_sdf_parameters("/home/jarrod/Documents/gymfc-digitaltwin-tinyhawk/models/tinyhawk/stl/Motor.stl", 0.001, 0.00253)

    print("Geometry for props")
    for p in data["props"]:
        dump_sdf_parameters("/home/jarrod/Documents/gymfc-digitaltwin-tinyhawk/models/tinyhawk/stl/Prop.stl", 0.001, 0.00062)


    template_name="model.sdf.template.xml"
    sdf = render_from_template(template_dir, template_name, **data)
    sdf_file = os.path.join(template_dir, "model.sdf")
    with open(sdf_file, "w") as f:
        f.write(sdf)




def main(stl_file, scale, mass):
    obj = mesh.Mesh.from_file(stl_file)
    volume, cog, inertia = obj.get_mass_properties()

    print ("M={}Kg".format(mass))
    density = mass/volume
    print ("Density ", density)

    i = inertia * np.power(scale, 2) * density
    #i = inertia  * mass/volume
    print("Volume                                  = {0}".format(volume))
    print("Position of the center of gravity (COG) = {0}".format(cog))
    print("Scaled: Position of the center of gravity (COG) = {0}".format(cog * scale))

    # '{0:f}'.format
    print("Inertia matrix at expressed at the COG  = {0}".format(i[0,:]))
    print("                                          {0}".format(i[1,:]))
    print("                                          {0}".format(i[2,:]))
    
    print ("Before Scaling, ")
    print("Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
    print("                                          {0}".format(inertia[1,:]))
    print("                                          {0}".format(inertia[2,:]))

    # Compute the bounding box
    minx, maxx, miny, maxy, minz, maxz = find_mins_maxs(obj)
    w = (maxx - minx) * scale
    l = (maxy - miny) * scale
    h = (maxz - minz) * scale

    print ("Bounding box")
    print ("w={}m l={}m h={}m".format(w,l,h))

    print ("----------------------")
    sdf_cog(cog*scale)
    sdf_inertia(i)



if __name__ == "__main__":
    parser = argparse.ArgumentParser("Compute inertia properties for SDF.")
    parser.add_argument('--template-dir', help="Directory where template file exists.")
    parser.add_argument('--aircraft-config', help="Configuration for aircraft")

    args = parser.parse_args()

    
#    main(args.stl, args.length_scale, args.mass)
    gen(args.template_dir, args.aircraft_config)


