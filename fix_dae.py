import xml.etree.ElementTree as ET
import os
import glob

def upgrade_dae_materials(dae_path):
    print(f"Processing {dae_path}")
    tree = ET.parse(dae_path)
    root = tree.getroot()
    ns = {'collada': 'http://www.collada.org/2005/11/COLLADASchema'}

    # 1. Update Effects: Switch Lambert to Phong and add Ambient
    effects = root.findall('.//collada:library_effects//collada:effect', ns)
    if not effects:
        print("No effects found")
    
    for effect in effects:
        profile_common = effect.find('.//collada:profile_COMMON', ns)
        if profile_common is not None:
            technique = profile_common.find('.//collada:technique', ns)
            if technique is not None:
                # Find current shading model (lambert or phong)
                lambert = technique.find('collada:lambert', ns)
                phong = technique.find('collada:phong', ns)
                
                target_node = None
                if lambert is not None:
                    print(f"  Converting lambert to phong in effect {effect.get('id')}")
                    # Change tag to phong
                    lambert.tag = '{http://www.collada.org/2005/11/COLLADASchema}phong'
                    target_node = lambert
                elif phong is not None:
                    target_node = phong

                if target_node is not None:
                    # Check/Add Ambient
                    ambient = target_node.find('collada:ambient', ns)
                    if ambient is None:
                        # print("  Adding ambient color")
                        ambient = ET.Element('{http://www.collada.org/2005/11/COLLADASchema}ambient')
                        color = ET.SubElement(ambient, '{http://www.collada.org/2005/11/COLLADASchema}color', sid="ambient")
                        color.text = "0.6 0.6 0.6 1"
                        # Insert ambient at the beginning or appropriate place (usually before diffuse)
                        target_node.insert(0, ambient) # Insert at top
                    else:
                        # Update existing ambient if present but different
                        color = ambient.find('collada:color', ns)
                        if color is not None:
                            color.text = "0.6 0.6 0.6 1"

                    # Add Specular if missing (from reference)
                    specular = target_node.find('collada:specular', ns)
                    if specular is None:
                         # Reference had specular 0.027...
                         # print("  Adding specular")
                         specular = ET.Element('{http://www.collada.org/2005/11/COLLADASchema}specular')
                         color = ET.SubElement(specular, '{http://www.collada.org/2005/11/COLLADASchema}color', sid="specular")
                         color.text = "0.027451 0.027451 0.027451 1"
                         target_node.append(specular)

                    # Add Shininess if missing
                    shininess = target_node.find('collada:shininess', ns)
                    if shininess is None:
                         # print("  Adding shininess")
                         shininess = ET.Element('{http://www.collada.org/2005/11/COLLADASchema}shininess')
                         float_node = ET.SubElement(shininess, '{http://www.collada.org/2005/11/COLLADASchema}float', sid="shininess")
                         float_node.text = "256"
                         target_node.append(shininess)

    # Save changes
    ET.register_namespace('', "http://www.collada.org/2005/11/COLLADASchema")
    tree.write(dae_path, encoding='utf-8', xml_declaration=True)
    print(f"Saved {dae_path}")

target_dir = "/home/ai-suitcase-1/nitta_workspace/cabot/cabot-navigation/LIRS-HMLG/Children/self_made"
# Find all dae files in subdirectories
dae_files = glob.glob(os.path.join(target_dir, "**", "*.dae"), recursive=True)

print(f"Found {len(dae_files)} files")
for dae in dae_files:
    upgrade_dae_materials(dae)
