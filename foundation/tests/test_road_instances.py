from foundation.road_instances import build


def test_chain_keeps_unresolved_cycle():
    asset = {'Imports': [{'ObjectName': 'BP_Segment_Rot_C'}], 'Exports': [
        {'ObjectName': 'road', 'ClassIndex': -1, 'Data': [{'Name': 'Segment', 'Value': 2}]},
        {'ObjectName': 'mesh', 'Data': [{'Name': 'AttachParent', 'Value': 3}]},
        {'ObjectName': 'parent', 'Data': [{'Name': 'AttachParent', 'Value': 2}]}]}
    actor = build(asset, 'abc')['actors'][0]
    assert len(actor['chain']) == 2
    assert actor['unresolved_parent'] == 2
