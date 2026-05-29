#!/usr/bin/env python3
"""
sdf_to_json.py — Convierte un fichero SDF/World de Gazebo a JSON
para que Unity pueda reconstruir la geometría dinámicamente.

Extrae todos los <model> estáticos con geometría <box>, incluyendo:
  - nombre del modelo
  - pose (x, y, z, roll, pitch, yaw)
  - tamaño de la caja (sx, sy, sz)
  - color difuso (r, g, b, a)

Uso:
  python3 sdf_to_json.py <fichero.sdf> [<salida.json>]

Si no se indica salida, se escribe a /tmp/dualsim_world.json
"""

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_pose(pose_str: str) -> dict:
    """Parsea una cadena de pose SDF '0 0 1.25 0 0 0' a un dict."""
    parts = [float(x) for x in pose_str.strip().split()]
    # SDF pose: x y z roll pitch yaw
    while len(parts) < 6:
        parts.append(0.0)
    return {
        'x': parts[0], 'y': parts[1], 'z': parts[2],
        'roll': parts[3], 'pitch': parts[4], 'yaw': parts[5],
    }


def parse_size(size_str: str) -> dict:
    """Parsea una cadena de tamaño SDF '0.15 4.15 2.5' a un dict."""
    parts = [float(x) for x in size_str.strip().split()]
    return {'x': parts[0], 'y': parts[1], 'z': parts[2]}


def parse_color(color_str: str) -> dict:
    """Parsea una cadena de color SDF '0.7 0.7 0.7 1' a un dict."""
    parts = [float(x) for x in color_str.strip().split()]
    while len(parts) < 4:
        parts.append(1.0)
    return {'r': parts[0], 'g': parts[1], 'b': parts[2], 'a': parts[3]}


def extract_models(sdf_path: str) -> list:
    """Extrae los modelos estáticos con geometría box del SDF."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    models = []

    # Buscar todos los <model> en el documento (soporta SDF y World)
    for model in root.iter('model'):
        name = model.get('name', 'unnamed')

        # Solo procesar modelos estáticos (paredes, obstáculos, etc.)
        static_el = model.find('static')
        if static_el is None or static_el.text.strip().lower() != 'true':
            continue

        # Pose del modelo
        pose_el = model.find('pose')
        pose = parse_pose(pose_el.text) if pose_el is not None else parse_pose('0 0 0 0 0 0')

        # Buscar geometría box en los links
        for link in model.iter('link'):
            # Pose del link (relativa al modelo)
            link_pose_el = link.find('pose')
            link_pose = parse_pose(link_pose_el.text) if link_pose_el is not None else None

            # Buscar visual con geometría box
            for visual in link.iter('visual'):
                box = visual.find('.//geometry/box/size')
                if box is None:
                    continue

                size = parse_size(box.text)

                # Color del material (diffuse o ambient)
                color = {'r': 0.7, 'g': 0.7, 'b': 0.7, 'a': 1.0}  # default gris
                diffuse = visual.find('.//material/diffuse')
                if diffuse is not None:
                    color = parse_color(diffuse.text)
                else:
                    ambient = visual.find('.//material/ambient')
                    if ambient is not None:
                        color = parse_color(ambient.text)

                entry = {
                    'name': name,
                    'pose': pose,
                    'size': size,
                    'color': color,
                }

                # Si el link tiene pose propia, incluirla
                if link_pose is not None:
                    entry['link_pose'] = link_pose

                models.append(entry)
                break  # Solo el primer visual con box por link

    return models


def sdf_to_json(sdf_path: str, output_path: str = '/tmp/dualsim_world.json') -> str:
    """Convierte un SDF a JSON y lo escribe a disco."""
    sdf_path = str(Path(sdf_path).resolve())
    models = extract_models(sdf_path)

    world_data = {
        'source_file': sdf_path,
        'world_name': Path(sdf_path).stem,
        'model_count': len(models),
        'models': models,
    }

    with open(output_path, 'w') as f:
        json.dump(world_data, f, indent=2)

    return output_path


def main():
    if len(sys.argv) < 2:
        print(f"Uso: {sys.argv[0]} <fichero.sdf> [<salida.json>]")
        print("  Si no se indica salida, se escribe a /tmp/dualsim_world.json")
        sys.exit(1)

    sdf_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else '/tmp/dualsim_world.json'

    if not Path(sdf_path).exists():
        print(f"Error: no se encuentra '{sdf_path}'")
        sys.exit(1)

    result = sdf_to_json(sdf_path, output_path)
    print(f"[sdf_to_json] Convertido: {sdf_path}")
    print(f"[sdf_to_json] Salida: {result}")

    # Mostrar resumen
    with open(result) as f:
        data = json.load(f)
    print(f"[sdf_to_json] Modelos estáticos encontrados: {data['model_count']}")
    for m in data['models']:
        s = m['size']
        p = m['pose']
        print(f"  - {m['name']}: pos=({p['x']}, {p['y']}, {p['z']}) "
              f"size=({s['x']}, {s['y']}, {s['z']})")


if __name__ == '__main__':
    main()
