"""Palette-backed PBR material helpers for Gazebo Ogre2."""
from pathlib import Path
import hashlib
from PIL import Image
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
PALETTE=HERE/'materiales'
PALETTE.mkdir(exist_ok=True)
def color_map(color):
    vals=[float(x) for x in color.split()]
    rgb=tuple(max(0,min(255,round(x*255))) for x in vals[:3])
    alpha=vals[3] if len(vals)>3 else 1.0
    key=hashlib.sha1(bytes(rgb)).hexdigest()[:10]
    path=PALETTE/f'{key}.png'
    if not path.exists(): Image.new('RGB',(4,4),rgb).save(path)
    return path,alpha

def set_pbr(material,color,roughness=.82):
    for child in list(material): material.remove(child)
    path,alpha=color_map(color)
    pbr=ET.SubElement(material,'pbr'); metal=ET.SubElement(pbr,'metal')
    ET.SubElement(metal,'albedo_map').text=path.as_uri()
    ET.SubElement(metal,'metalness').text='0.0'
    ET.SubElement(metal,'roughness').text=str(roughness)
    if alpha < .99: ET.SubElement(material,'transparency').text=str(1-alpha)

def apply_pbr_to_visual(visual,color):
    mat=visual.find('material')
    if mat is None: mat=ET.SubElement(visual,'material')
    set_pbr(mat,color)

if __name__=='__main__':
    import xml.etree.ElementTree as ET
    path=HERE/'huerta_ruda.sdf'; root=ET.parse(path).getroot(); count=0
    for visual in root.findall('.//visual'):
        mat=visual.find('material')
        if mat is None: continue
        color=mat.findtext('diffuse') or mat.findtext('ambient')
        if color:
            set_pbr(mat,color); count+=1
    ET.indent(root,space=' ')
    path.write_text("<?xml version='1.0' encoding='utf-8'?>\n"+ET.tostring(root,encoding='unicode'))
    print(f'PBR materials applied to {count} base visuals; palette={PALETTE}')
