import math
import random
import json
import genotype as g


def generate_generation(num_worms):
    genotypes = []
    
    for i in range(num_worms):
        genes = g.Genotype.randomize()
        xml = generate_mujoco_xml(genes)
        with open(f"models//model{i}.xml", "w") as file:
            file.write(xml)
        genotype_info = {
            "numSegments": genes.numSegments,
            "segment_dimensions": genes.segment_dimensions,
            "moves": genes.moves.tolist(),
            "protocol": genes.protocol,
            "gear": genes.gear,
            "friction": genes.friction
        }
        genotypes.append(genotype_info)
    
    with open("genotypes.json", "w") as f:
        json.dump(genotypes, f, indent=4)


def generate_mujoco_xml(genotype):
    
    bodyDims = genotype.segment_dimensions
    parts = genotype.numSegments
    

    xml_str = """<mujoco model = "worm">
    <option gravity="0 0 -20"/>
    <worldbody>
        <light diffuse=".8 .8 .8" pos="0 0 5" dir="0 0 -1"/>
        <geom type="plane" size="50 50 0.1" rgba=".9 .5 .9 1"/>"""
    
    for i in range(parts):

        if i ==0:
            sliding, torsional, rolling = genotype.friction[i]
            x0, y0, z0 = bodyDims[0]
            xml_str += f"""
        <body name="body0" euler="0 0 0">
            <joint type="free" axis="-1 0 0" pos="0 0 0"/>
            <geom type="ellipsoid" size="{x0}" rgba="{x0 *151} 0 {y0 *101} 0.9" fromto="0 0 0 0 0 0.09" mass="10" friction = "{sliding} {torsional} {rolling}"/>\n
    """
        else:
            x, y, z = bodyDims[i]
            p_x, p_y, p_z = bodyDims[i-1]
            j_x = -y
            
            sliding, torsional, rolling = genotype.friction[i]
            
            indent = "    " * (i + 2)  # Assuming you want to use spaces for indentation; replace with "\t" * i for tabs
            xml_str += indent + f"""<body pos="0 0 {x}" euler="0 0 0">\n"""
            xml_str += indent + f"""    <joint name="joint{i}" type="hinge" axis="-1 0 0" pos="0 0 0" range="-15 15"/>\n"""
            xml_str += indent + f"""    <geom type="ellipsoid" size="{x}" fromto="0 0 0 0 0 0.09" rgba="0 {x *251} {y *141} 0.9" mass="10" friction = "{sliding} {torsional} {rolling}"/>\n"""
            xml_str += "\n"

    # Assuming xml_str is your XML string that you've been building up to this point
    for i in range(parts, 0, -1):
        xml_str += "    " * (i+1) + "</body>\n"

    xml_str += "    " + "</worldbody>\n"

    xml_str += "    " + "<actuator>\n"
    
    gear = genotype.gear

    for i in range(parts-1):
        xml_str += "    " * (2) + f"""<motor name="seg{i}"  gear="{gear}"  joint="joint{i+1}"/>\n"""

    xml_str += "    " + "</actuator>\n"

    xml_str += "</mujoco>"

    return xml_str

