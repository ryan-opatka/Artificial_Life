import math
import random
import genotype as g


random_genotype = g.Genotype.randomize()

print(random_genotype)

blockDims = random_genotype.segment_dimensions

parts = random_genotype.numSegments 


def generate_mujoco_xml(parts):
    # XML Header and world setup

    xml_str = """<mujoco>
    <option gravity="0 0 -9.81"/>
    <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <geom type="plane" size="5 5 0.1" rgba=".9 .5 .9 1"/>"""
    
    for i in range(parts):

        if i ==0:
            x0, y0, z0 = blockDims[0]
            xml_str += f"""
        <body name="block0" pos="0 0 0">
            <joint type="free"/>
            <geom type="box" size="{x0} {y0} {z0}" rgba="1 0 0 1" euler="0 90 0"/>
    """

        else:
            x, y, z = blockDims[i]
            p_x, p_y, p_z = blockDims[i-1]
            j_x = -x
            xml_str += f"""
            <body name="block{i}" pos="0 {(p_y+y)*(i)} 0">
                <joint name="joint{i-1}" pos="0 {j_x} 0" type="hinge" axis="0 0 1" range="-15 0"/>
                <geom type="box" size="{x} {y} {z}" rgba="1 0 0 1" euler="0 90 0"/>
        """
            if i < parts - 1:
                xml_str += """
                </body>
                """
            
        
    xml_str += """
            </body>
        </body>
    </worldbody>
</mujoco>"""
    
    return xml_str

xml_content = generate_mujoco_xml(parts)

with open("homework_1//model.xml", "w") as file:
    file.write(xml_content)
