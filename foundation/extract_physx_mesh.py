"""Extract local triangle geometry from a PhysX 3.x cooked MESH v14 stream.

Only the little-endian vertex/index prefix is decoded. Material tables,
midphase trees, collision channels, mesh-instance transforms and runtime
contacts are not interpreted. Reject unknown formats instead of guessing.
The prefix layout follows NVIDIA PhysX 3.4 TriangleMeshBuilder::save.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import struct
import numpy as np


def decode(blob):
    marker = b'NXS\x01MESH'
    offset = blob.find(marker)
    if offset < 0 or blob.find(marker, offset + 1) != -1:
        raise ValueError('expected exactly one little-endian PhysX MESH header')
    if len(blob) < offset + 28:
        raise ValueError('truncated header')
    version, midphase, flags, nv, nt = struct.unpack_from('<5I', blob, offset + 8)
    if version != 14 or midphase != 0 or flags & ~0x3f or bool(flags & 4) and bool(flags & 8):
        raise ValueError('unsupported version, midphase or flags')
    if not 3 <= nv <= 2_000_000 or not 1 <= nt <= 4_000_000:
        raise ValueError('invalid mesh dimensions')
    vertex_offset = offset + 28
    index_offset = vertex_offset + nv * 12
    dtype = np.dtype('u1' if flags & 4 else '<u2' if flags & 8 else '<u4')
    required = index_offset + nt * 3 * dtype.itemsize
    if required > len(blob):
        raise ValueError('truncated mesh')
    vertices = np.frombuffer(blob, dtype='<f4', count=nv * 3, offset=vertex_offset).reshape(nv, 3).copy()
    triangles = np.frombuffer(blob, dtype=dtype, count=nt * 3, offset=index_offset).reshape(nt, 3).astype(np.uint32)
    if not np.all(np.isfinite(vertices)) or int(triangles.max()) >= nv:
        raise ValueError('nonfinite vertices or out-of-bounds triangle index')
    if np.any((triangles[:, 0] == triangles[:, 1]) | (triangles[:, 0] == triangles[:, 2]) |
              (triangles[:, 1] == triangles[:, 2])):
        raise ValueError('degenerate triangle indices')
    return vertices, triangles, {'header_offset': offset, 'version': version, 'midphase': midphase,
                                  'flags': flags, 'vertex_count': nv, 'triangle_count': nt,
                                  'index_bytes': dtype.itemsize, 'unparsed_trailing_bytes': len(blob)-required}


def body_setups(asset):
    imports = asset.get('Imports', [])
    result = []
    for entry in asset.get('Exports', []):
        ci = entry.get('ClassIndex', 0)
        if isinstance(ci, int) and ci < 0 and -ci <= len(imports) and imports[-ci-1].get('ObjectName') == 'BodySetup':
            result.append(entry)
    return result


def extract(source, output):
    raw = source.read_bytes()
    asset = json.loads(raw)
    bodies = body_setups(asset)
    if len(bodies) != 1:
        raise ValueError('missing or ambiguous BodySetup')
    body = bodies[0]
    flag = next((p.get('EnumValue') for p in body.get('Data', []) if p.get('Name') == 'CollisionTraceFlag'), None)
    blob = base64.b64decode(body['Extras'], validate=True)
    vertices, triangles, metadata = decode(blob)
    metadata.update({'source_json_sha256': hashlib.sha256(raw).hexdigest(),
                     'body_extras_sha256': hashlib.sha256(blob).hexdigest(),
                     'collision_trace_flag': flag, 'coordinate_scope': 'local mesh coordinates, assumed UE cm',
                     'bounds_min': vertices.min(axis=0).tolist(), 'bounds_max': vertices.max(axis=0).tolist(),
                     'limitations': ['only MESH vertex/index prefix decoded',
                                     'material data, mesh tree and instance transforms not decoded',
                                     'collision channels and runtime contact not validated']})
    np.savez_compressed(output, vertices=vertices, triangles=triangles)
    output.with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    print(json.dumps(extract(args.source, args.output), indent=2))


if __name__ == '__main__':
    main()
