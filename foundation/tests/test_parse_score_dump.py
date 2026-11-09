from foundation.parse_score_dump import parse


def test_parse_score_dump_component_and_score(tmp_path):
    log = tmp_path / 'score.log'
    log.write_text(
        'DUMP_BEGIN actor=0x100 count=2 arr=0x200\n'
        'DUMP_ITEM actor=0x100 idx=0 component=0x300 xyz_cm=1,2,3 '
        'quat=0,0,0,1 scale=1,1,1\n'
        'DUMP_ITEM actor=0x100 idx=1 component=0x301 xyz_cm=4,5,6 '
        'quat=0,0,0,1 scale=1,1,1\n'
        'DUMP_END actor=0x100 count=2\n'
        'SEG actor=0x100 idx=1 enter=1 event=0 score=0\n'
        'SEG actor=0x100 idx=1 enter=0 event=0 score=1\n'
        'SEG actor=0x100 idx=0 enter=1 event=1 score=2\n',
        encoding='utf-8')
    result = parse(log)
    assert result['actor_count'] == 1
    assert result['component_count'] == 2
    assert result['score_end'] == 2
    assert result['score_increment_count'] == 2
    assert result['observed_indices'] == [0, 1]
    assert result['actors']['0x100']['components'][1]['position_cm'] == [4.0, 5.0, 6.0]
    assert len(result['source_sha256']) == 64
