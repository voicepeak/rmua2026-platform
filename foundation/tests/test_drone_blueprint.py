import json
from pathlib import Path

from foundation.road_scene import value


def test_detect_sphere_blueprint_properties_and_parent():
    path = Path(__file__).resolve().parents[1]/'asset_analysis/blueprints/BP_FlyingPawn.json'
    asset = json.loads(path.read_bytes())
    exports, imports = asset['Exports'], asset['Imports']
    sphere = next(e for e in exports if e.get('ObjectName') == 'JianCeQiu_GEN_VARIABLE')
    props = {p['Name']:p.get('Value') for p in sphere['Data']}
    assert props['SphereRadius'] == 2
    assert value(props, 'RelativeLocation', 'XYZ', [0, 0, 0]).tolist() == [0, 0, 6]
    node = next(e for e in exports if e.get('ObjectName') == 'SCS_Node_7')
    fields = {p['Name']:p.get('Value') for p in node['Data']}
    assert fields['InternalVariableName'] == 'Mavic3'
    children = [exports[p['Value']-1] for p in fields['ChildNodes']]
    assert any(any(p['Name'] == 'InternalVariableName' and p.get('Value') == 'JianCeQiu'
                   for p in e.get('Data', [])) for e in children)
