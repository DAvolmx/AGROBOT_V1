#!/usr/bin/env python3
"""Apply photo materials and curved leaf meshes without changing collisions."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def material(visual,texture,roughness=.9):
    old=visual.find('material')
    if old is not None:visual.remove(old)
    m=ET.SubElement(visual,'material')
    ET.SubElement(m,'ambient').text='.8 .8 .8 1'
    ET.SubElement(m,'diffuse').text='1 1 1 1'
    metal=ET.SubElement(ET.SubElement(m,'pbr'),'metal')
    ET.SubElement(metal,'albedo_map').text=str(texture)
    ET.SubElement(metal,'roughness').text=str(roughness)
    ET.SubElement(metal,'metalness').text='0'


def decorate(source,out,assets):
    tree=ET.parse(source); world=tree.getroot().find('world')
    before=[ET.tostring(c) for c in world.findall('.//collision')]
    visual=world.find("model[@name='suelo']/link/visual")
    geom=visual.find('geometry'); geom.clear()
    plane=ET.SubElement(geom,'plane');ET.SubElement(plane,'normal').text='0 0 1'
    ET.SubElement(plane,'size').text='30 12'
    material(visual,assets/'ground_atlas.png')
    leaves=0
    for model in world.findall('model'):
        if model.get('name','').startswith('surco_'):
            for vis in model.findall('.//visual'):material(vis,assets/'soil_albedo.png')
        if model.get('name','').startswith('cultivo_'):
            for vis in model.findall('.//visual'):
                if not vis.get('name','').startswith('leaf_visual'):continue
                g=vis.find('geometry'); radii=g.findtext('ellipsoid/radii')
                if not radii:continue
                # Built-in UVs remain reliable in headless EGL on this Jetson.
                material(vis,assets/'leaf_albedo.png',.8)
                leaves+=1
    camera=world.find(".//sensor[@name='camara_rgb']/camera/image")
    camera.find('width').text='512'; camera.find('height').text='384'
    # Keep dynamic shadows disabled: previous EGL regression produced black RGB.
    after=[ET.tostring(c) for c in world.findall('.//collision')]
    if before!=after:raise ValueError('Collision geometry changed unexpectedly')
    out.parent.mkdir(parents=True,exist_ok=True);tree.write(out,encoding='utf-8',xml_declaration=True)
    metadata={'source_world':str(source),'output_world':str(out),'photo_textured_leaves':leaves,
              'visual_geometry':'built_in_ellipsoid_uv_and_ground_plane_atlas',
              'camera_resolution':[512,384],'collision_geometry_identical':True,
              'world_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),
              'texture_provenance':json.loads((assets/'provenance.json').read_text()),
              'photo_texture_not_photogrammetry':True,'cast_shadows':False}
    out.with_suffix('.json').write_text(json.dumps(metadata,indent=2));print(json.dumps(metadata,indent=2))


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--assets',type=Path,required=True)
    a=p.parse_args();decorate(a.source.resolve(),a.out.resolve(),a.assets.resolve())


if __name__=='__main__':main()
