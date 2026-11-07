from foundation.sprite_inventory import build


def test_standalone_sprite_asset_and_transform():
    imports = [{'ObjectName': 'PaperSpriteComponent'}, {'ObjectName': 'PaperSpriteActor'},
               {'ObjectName': 'Marker', 'OuterIndex': -4},
               {'ObjectName': '/Game/Markers/Marker', 'OuterIndex': 0, 'ClassName': 'Package'}]
    def vec(name, x, y, z):
        return {'Name': name, 'Value': [{'Value': {'X': x, 'Y': y, 'Z': z}}]}
    asset = {'Imports': imports, 'Exports': [
        {'ObjectName': 'marker_actor', 'ClassIndex': -2, 'OuterIndex': 0},
        {'ObjectName': 'RenderComponent', 'ClassIndex': -1, 'OuterIndex': 1,
         'Data': [{'Name': 'SourceSprite', 'Value': -3}, vec('RelativeLocation', 100, 200, 300)]}]}
    result = build(asset, {})
    assert not result['unresolved']
    assert result['placements'][0]['translation_ue_cm'] == [100, 200, 300]
    assert result['placements'][0]['sprite_package'] == '/Game/Markers/Marker'
