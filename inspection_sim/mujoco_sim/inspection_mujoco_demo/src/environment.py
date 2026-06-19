import os
import random
from PIL import Image, ImageDraw
from src import config


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))  
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)           
MODELS_DIR = os.path.join(PROJECT_ROOT, "models")         
def generate_qr_labels():
   
    os.makedirs(MODELS_DIR, exist_ok=True)
    
    for bid, data in config.bin_data.items():
       
        tex_path = os.path.join(MODELS_DIR, f"qr_{bid}.png")
        img = Image.new('RGB', (256, 256), color=(245, 245, 245))
        draw = ImageDraw.Draw(img)
        
        is_error = bid in config.ERROR_BINS
        border_color = "#ef4444" if is_error else "#34d399"
        
        draw.rectangle([64, 20, 192, 148], outline=border_color, width=4)
        for x in range(68, 188, 8):
            for y in range(24, 144, 8):
                if random.choice([True, False]):
                    draw.rectangle([x, y, x+8, y+8], fill="black")
                    
        draw.text((20, 160), f"BIN: {bid} - GR LABEL", fill="black")
        
        if is_error:
            draw.text((20, 185), f"Part No: WRONG-ITEM!", fill="red")
            draw.text((20, 210), f"Qty: UNKNOWN", fill="red")
        else:
            draw.text((20, 185), f"Part No: {data['part']}", fill="black")
            draw.text((20, 210), f"Qty: {data['qty']}", fill="black")
        
        img.save(tex_path)

def generate_warehouse_xml():
    generate_qr_labels() 
    
    xml = """<mujoco model="warehouse_inspection_demo">
    <option timestep="0.01" gravity="0 0 -9.81"/>
    <visual>
        <headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6" specular="0.1 0.1 0.1"/>
        <rgba haze="0.15 0.18 0.23 1"/>
        <quality shadowsize="4096"/>
    </visual>
    <asset>
        <mesh name="warehouse_mesh" file="warehouse.obj"/>
        <mesh name="box_mesh"       file="SM_CardBoxB_01_1051.obj"/>

        <material name="mat_rack"  rgba="0.5 0.5 0.5 1" shininess="0.3"/>
        <material name="mat_shelf" rgba="0.6 0.6 0.6 1"/>
        <material name="mat_box"   rgba="0.72 0.58 0.42 1"/> 
        <texture type="2d" name="floor_tex" builtin="checker" rgb1="0.22 0.25 0.28" rgb2="0.18 0.20 0.23" width="512" height="512"/>
        <material name="floor_mat" texture="floor_tex" texrepeat="15 15" reflectance="0.1"/>
        """
        
    for bid in config.bin_data.keys():
        xml += f'\n        <texture type="2d" name="tex_{bid}" file="qr_{bid}.png" />'
        xml += f'\n        <material name="mat_{bid}" texture="tex_{bid}"/>'

    xml += """
    </asset>
    <worldbody>
        <light name="ceiling_1" pos="1.2 -0.9 8" dir="0 0 -1" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2" castshadow="true"/>
        <light name="ceiling_2" pos="-2.0 -0.9 7" dir="0 0 -1" diffuse="0.5 0.5 0.5" castshadow="true"/>
        <geom name="floor" type="plane" size="25 25 0.1" pos="0 -0.9 0" material="floor_mat"/>
        
        <geom type="box" size="10.0 0.1 2.5" pos="1.2 5.0 2.5" rgba="0.88 0.88 0.88 1"/>
        <geom type="box" size="10.0 0.1 1.0" pos="1.2 5.0 6.0" rgba="0.92 0.62 0.08 1"/>
        <geom type="box" size="10.0 0.1 2.5" pos="1.2 -8.0 2.5" rgba="0.88 0.88 0.88 1"/>
        <geom type="box" size="10.0 0.1 1.0" pos="1.2 -8.0 6.0" rgba="0.92 0.62 0.08 1"/>
        <geom type="box" size="0.1 6.5 2.5" pos="-8.8 -1.5 2.5" rgba="0.88 0.88 0.88 1"/>
        <geom type="box" size="0.1 6.5 1.0" pos="-8.8 -1.5 6.0" rgba="0.92 0.62 0.08 1"/>
        <geom type="box" size="0.1 6.5 2.5" pos="11.2 -1.5 2.5" rgba="0.88 0.88 0.88 1"/>
        <geom type="box" size="0.1 6.5 1.0" pos="11.2 -1.5 6.0" rgba="0.92 0.62 0.08 1"/>

        <body name="warehouse_env" pos="0 0 0"></body>
"""
    columns = {'A': 0.0, 'B': 1.2, 'C': 2.4}
    levels  = [1, 2, 3, 4, 5, 6]
    Z_OFFSET = 0.1 

    COL_Z = 2.05                       
    COL_SIZE = "0.04 0.04 2.05"        
    COL_COLOR = "0.5 0.5 0.5 1"     

    xml += f"""
        <body pos="-0.4 -0.45 {COL_Z}"><geom type="box" size="{COL_SIZE}" rgba="{COL_COLOR}"/></body>
        <body pos="2.8 -0.45 {COL_Z}"><geom type="box" size="{COL_SIZE}" rgba="{COL_COLOR}"/></body>
        <body pos="-0.4 0.45 {COL_Z}"><geom type="box" size="{COL_SIZE}" rgba="{COL_COLOR}"/></body>
        <body pos="2.8 0.45 {COL_Z}"><geom type="box" size="{COL_SIZE}" rgba="{COL_COLOR}"/></body>
    """
    for lvl in levels:
        z_shelf = (lvl - 1) * 0.8 + Z_OFFSET
        xml += f'\n        <body pos="1.2 0 {z_shelf - 0.02}"><geom type="box" size="1.6 0.45 0.02" material="mat_shelf"/></body>'
        xml += f'\n        <body pos="1.2 -0.45 {z_shelf - 0.01}"><geom type="box" size="1.6 0.02 0.04" rgba="0.92 0.42 0.05 1"/></body>'

    for col_name, x in columns.items():
        for lvl in levels:
            bid = f"{col_name}{lvl}"
            z_shelf = (lvl - 1) * 0.8 + Z_OFFSET
            xml += f"""
        <body name="bin_{bid}" pos="{x} 0 {z_shelf}">
            <geom name="geom_{bid}" type="mesh" mesh="box_mesh" rgba="0.72 0.58 0.42 1"/>
            <geom type="plane" size="0.1 0.1 0.01" pos="0 -0.28 0.25" euler="90 0 0" material="mat_{bid}"/>
        </body>"""

    xml += f"""
        <body pos="-0.4 -2.25 {COL_Z}"><geom type="box" size="{COL_SIZE}" rgba="{COL_COLOR}"/></body>
        <body pos="2.8 -2.25 {COL_Z}"><geom type="box" size="{COL_SIZE}" rgba="{COL_COLOR}"/></body>
        <body pos="-0.4 -1.35 {COL_Z}"><geom type="box" size="{COL_SIZE}" rgba="{COL_COLOR}"/></body>
        <body pos="2.8 -1.35 {COL_Z}"><geom type="box" size="{COL_SIZE}" rgba="{COL_COLOR}"/></body>
    """

    for lvl in levels:
        z_shelf = (lvl - 1) * 0.8 + Z_OFFSET
        xml += f'\n        <body pos="1.2 -1.8 {z_shelf - 0.02}"><geom type="box" size="1.6 0.45 0.02" material="mat_shelf"/></body>'
        xml += f'\n        <body pos="1.2 -1.35 {z_shelf - 0.01}"><geom type="box" size="1.6 0.02 0.04" rgba="0.92 0.42 0.05 1"/></body>'

    for col_name, x in columns.items():
        for lvl in levels:
            z_shelf = (lvl - 1) * 0.8 + Z_OFFSET
            xml += f"""
        <body name="sec_bin_{col_name}{lvl}" pos="{x} -1.8 {z_shelf}">
            <geom name="geom_sec_{col_name}{lvl}" type="mesh" mesh="box_mesh" rgba="0.72 0.58 0.42 1" euler="0 0 180"/>
        </body>"""

    xml += """
        <body name="drone_body" mocap="true" pos="-3.0 -0.9 0.22">
            <geom type="cylinder" size="0.06 0.025" rgba="0.1 0.6 0.8 1"/>
            <geom type="capsule" size="0.008" fromto="0 0 0  0.15  0.15 0" rgba="0.4 0.4 0.4 1"/>
            <geom type="capsule" size="0.008" fromto="0 0 0 -0.15  0.15 0" rgba="0.4 0.4 0.4 1"/>
            <geom type="capsule" size="0.008" fromto="0 0 0 -0.15 -0.15 0" rgba="0.4 0.4 0.4 1"/>
            <geom type="capsule" size="0.008" fromto="0 0 0  0.15 -0.15 0" rgba="0.4 0.4 0.4 1"/>
            <geom type="cylinder" size="0.015 0.01" pos="0.15  0.15 0.01" rgba="0.8 0.1 0.1 1"/>
            <geom type="cylinder" size="0.015 0.01" pos="-0.15  0.15 0.01" rgba="0.8 0.1 0.1 1"/>
            <geom type="cylinder" size="0.015 0.01" pos="-0.15 -0.15 0.01" rgba="0.2 0.2 0.2 1"/>
            <geom type="cylinder" size="0.015 0.01" pos="0.15 -0.15 0.01" rgba="0.2 0.2 0.2 1"/>
            <geom type="box" size="0.09 0.008 0.001" pos="0.15  0.15 0.02" rgba="0.9 0.9 0.9 0.6"/>
            <geom type="box" size="0.09 0.008 0.001" pos="-0.15  0.15 0.02" rgba="0.9 0.9 0.9 0.6"/>
            <geom type="box" size="0.09 0.008 0.001" pos="-0.15 -0.15 0.02" rgba="0.9 0.9 0.9 0.6"/>
            <geom type="box" size="0.09 0.008 0.001" pos="0.15 -0.15 0.02" rgba="0.9 0.9 0.9 0.6"/>
            <geom type="box" size="0.015 0.015 0.015" pos="0 0.07 -0.015" rgba="0.1 0.1 0.1 1"/>
            <camera name="fpv_cam" pos="0 0.09 -0.015" xyaxes="1 0 0 0 0 1" fovy="55"/>
            <light name="drone_scanner_light" pos="0 0.1 -0.015" dir="0 1 0" diffuse="0.25 0.25 0.25" specular="0.05 0.05 0.05" active="true" castshadow="false"/>
        </body>
    </worldbody>
</mujoco>"""

    xml_path = os.path.join(MODELS_DIR, "scene.xml")
    with open(xml_path, "w", encoding="utf-8") as f:
        f.write(xml)