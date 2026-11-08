import pytest

from foundation.analyze_rotor_mapping import compare


def test_fixed_rotor_mapping_beats_swapped_channels():
    commands=[{'kind':'start','monotonic_ns':0},
              {'kind':'command','monotonic_ns':1_000_000,'pwm':[.1,.2,.3,.4]},
              {'kind':'command','monotonic_ns':2_000_000,'pwm':[.1,.2,.3,.4]}]
    lines=[]
    for i, value in enumerate((.1,.2,.3,.4)):
        for t in (.03,.04,.05):
            lines.append(f'KIN_ROTOR mono={t} rotor={hex(0x1000+i*8)} filtered=0 target=0 input={value} thrust=0')
    result=compare('\n'.join(lines),commands,start_s=.02,end_s=.06,delays_s=[.002])
    assert result['counts']==[3,3,3,3]
    assert result['delay_results'][0]['fixed_mapping_rmse']==pytest.approx(0)
    assert result['delay_results'][0]['best_alternative_rmse']>0
    with pytest.raises(ValueError,match='four rotor'):
        compare('\n'.join(lines[:3]),commands,start_s=.02,end_s=.06,delays_s=[.002])
