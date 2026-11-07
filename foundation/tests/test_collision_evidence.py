from foundation.collision_evidence import collect


def test_nested_profile_fields_do_not_imply_unspecified_defaults():
    props = [{'Name': 'BodyInstance', 'Value': [
        {'Name': 'CollisionProfileName', 'Value': 'BlockAll'},
        {'Name': 'CollisionEnabled', 'EnumValue': 'ECollisionEnabled::QueryAndPhysics'}]}]
    fields = collect(props)
    assert fields == [{'name': 'CollisionProfileName', 'Value': 'BlockAll'},
                      {'name': 'CollisionEnabled', 'EnumValue': 'ECollisionEnabled::QueryAndPhysics'}]
    assert collect([{'Name': 'MaxAngularVelocity', 'Value': 10}]) == []
