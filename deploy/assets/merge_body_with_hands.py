import yaml
f = open('inspire_hand_config.yaml')
try:
    config = yaml.load(f, Loader=yaml.FullLoader)
finally:
    f.close()

import xml.etree.ElementTree as ET

def merge_urdfs(g1_urdf, hand_type):
    lhand_urdf = f"inspire_hand/{hand_type}_left_hand.urdf"
    rhand_urdf = f"inspire_hand/{hand_type}_right_hand.urdf"
    G1 = ET.parse(g1_urdf)
    g1 = G1.getroot()
    lhand = ET.parse(lhand_urdf).getroot()
    rhand = ET.parse(rhand_urdf).getroot()

    # clean
    for element in g1:
        name = element.attrib.get('name', None)
        if element.tag == "link" and name in config['G1_remove_links']:
            print('[INFO] Remove link', name)
            g1.remove(element)

    for element in g1:
        name = element.attrib.get('name', None)
        if element.tag == "joint" and name in config['G1_remove_joints']:
            print('[INFO] Remove joint', name)
            g1.remove(element)

    for element in lhand:
        name = element.attrib.get('name', None)
        if element.tag == "link" and name in config['L_hand_remove_links']:
            lhand.remove(element)
    for element in rhand:
        name = element.attrib.get('name', None)
        if element.tag == "link" and name in config['R_hand_remove_links']:
            rhand.remove(element)

    # merge
    for element in lhand:
        if element.tag in ["link", "joint"]:
            g1.append(element)
    for element in rhand:
        if element.tag in ["link", "joint"]:
            g1.append(element)

    output = g1_urdf[:-5] + f"_with_inspire_hand_{hand_type}.urdf"
    G1.write(output)
    print(f"[INFO] Generate merged URDF file: {output} Done.")


merge_urdfs(g1_urdf="g1_29dof_rev_1_0.urdf", hand_type="FTP")
merge_urdfs(g1_urdf="g1_29dof_rev_1_0.urdf", hand_type="DFQ")

