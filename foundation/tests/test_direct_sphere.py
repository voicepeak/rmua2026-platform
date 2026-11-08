from foundation.analyze_direct_sphere import analyze
from foundation.region_volumes import CandidateBoxVolumes


def test_direct_sphere_intersection_and_missing_samples():
    box={'actor':'road','component':'ChargeArea','translation_ue_cm':[0,0,0],
         'quaternion_xyzw':[0,0,0,1], 'scale':[1,1,1], 'box_half_extents_cm':[10,10,10]}
    groups={'road':CandidateBoxVolumes([box])}
    events=[{'monotonic_s':1,'predicate':1,'is_transition':True,'sphere_world_cm':[11,0,0],
             'drone_world_cm':[11,0,-6],'sphere_component':'0x1'},
            {'monotonic_s':2,'predicate':0,'is_transition':False,'sphere_read_error':'missing'}]
    rows, counts=analyze(events, groups, 2)
    assert rows[0]['predicted_overlap']==1 and rows[0]['root_to_sphere_cm']==6
    assert not rows[1]['available'] and rows[1]['reason']=='missing'
    assert counts==[{'transition':True,'observed':1,'predicted':1,'count':1}]
