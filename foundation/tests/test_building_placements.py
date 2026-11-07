import pytest
from foundation.building_placements import build


def v(x, y, z):
    return [{'Value': {'X': x, 'Y': y, 'Z': z}}]


def test_template_and_instance_transform_composition():
    inventory = {'actors': [{'actor': 'a', 'class': 'BP_Building_X_C', 'components': [
        {'name': 'Part', 'export_index': 2, 'mesh': {'name': 'mesh', 'package': '/Game/mesh'},
         'unresolved_parent': None, 'chain': [
             {'name': 'Part', 'fields': {}},
             {'name': 'Root', 'fields': {'RelativeLocation': v(10, 0, 0)}}]}]}]}
    bp = {'BP_Building_X': {'Exports': [
        {'ObjectName': 'Part_GEN_VARIABLE', 'Data': [{'Name': 'RelativeLocation', 'Value': v(2, 0, 0)}]},
        {'ObjectName': 'Root_GEN_VARIABLE', 'Data': []}]}}
    result = build(inventory, bp, {'assets': [{'asset': 'mesh', 'triangle_status': 'decoded_prefix',
                                                'collision_trace_flag': None}]})
    assert result['placements'][0]['translation_ue_cm'] == pytest.approx([12, 0, 0])
    assert not result['unresolved']
    bp['BP_Building_X']['Exports'].pop()
    assert result['placements'] and build(inventory, bp, {'assets': []})['unresolved']
