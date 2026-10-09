#!/usr/bin/env python3
"""Merge OBJ object boundaries by material, preserving geometry and exported coordinates.
Writes derived assets only. No centering, rescaling, simplification or mesh welding.
"""
from pathlib import Path
from collections import OrderedDict, Counter
import hashlib
import json
import shutil

PACKAGE = Path(__file__).resolve().parents[1]
CATEGORIES = ['seabed', 'sediment', 'marine_growth', 'floating_objects', 'rocks',
              'building_ruins', 'debris', 'shipwreck', 'pipes']

def parse(path):
    records = []; groups = OrderedDict(); libraries = []; objects = 0
    material = None; smoothing = 'off'; counts = dict(v=0, vt=0, vn=0)
    for line in path.read_text().splitlines():
        parts = line.split()
        if not parts: continue
        tag = parts[0]
        if tag in counts:
            counts[tag] += 1; records.append(line)
        elif tag == 'mtllib': libraries.extend(parts[1:])
        elif tag == 'o': objects += 1
        elif tag == 'usemtl': material = line.split(None, 1)[1]
        elif tag == 's': smoothing = parts[1]
        elif tag == 'f':
            # Normalize relative indices before moving faces after vertex declarations.
            normalized = []
            for token in parts[1:]:
                refs = token.split('/')
                for i, key in enumerate(('v', 'vt', 'vn')):
                    if i >= len(refs) or not refs[i]: continue
                    index = int(refs[i])
                    if index < 0: index = counts[key] + index + 1
                    if not 1 <= index <= counts[key]:
                        raise ValueError('Invalid face index in ' + str(path))
                    refs[i] = str(index)
                normalized.append('/'.join(refs))
            groups.setdefault((material, smoothing), []).append('f ' + ' '.join(normalized))
        elif tag in ('g', '#'): pass
        else: raise ValueError('Unsupported OBJ record: ' + line)
    return records, groups, libraries, objects, counts

def face_signature(groups):
    return Counter((material, smoothing, face)
                   for (material, smoothing), faces in groups.items() for face in faces)

def main():
    source = PACKAGE / 'meshes/underwater_wreck_scene'
    destination = source / 'optimized'
    destination.mkdir(exist_ok=True)
    report = {}
    for category in CATEGORIES:
        src = source / (category + '.obj'); dst = destination / src.name
        records, groups, libraries, objects, counts = parse(src)
        lines = ['# Derived mesh: merged object boundaries; geometry and materials preserved.']
        lines += ['mtllib ' + lib for lib in libraries]
        lines += ['o ' + category] + records
        for (material, smoothing), faces in groups.items():
            if material is not None: lines.append('usemtl ' + material)
            lines.append('s ' + smoothing); lines.extend(faces)
        dst.write_text('\n'.join(lines) + '\n')
        for lib in libraries:
            libpath = (source / lib).resolve()
            if libpath.parent != source.resolve(): raise ValueError('External material resource')
            shutil.copy2(libpath, destination / lib)
            assert (destination / lib).read_bytes() == libpath.read_bytes()
        result_records, result_groups, _, result_objects, result_counts = parse(dst)
        assert records == result_records and counts == result_counts
        assert face_signature(groups) == face_signature(result_groups)
        report[category] = dict(source_objects=objects, optimized_objects=result_objects,
            material_smoothing_groups=len(groups), vertices=counts['v'],
            faces=sum(len(faces) for faces in groups.values()),
            source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),
            output_sha256=hashlib.sha256(dst.read_bytes()).hexdigest(),
            coordinates_normals_uv_faces_and_materials_preserved=True)
    validation = PACKAGE / 'validation/underwater_wreck'
    validation.mkdir(parents=True, exist_ok=True)
    (validation / 'merged_mesh_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))

if __name__ == '__main__': main()
