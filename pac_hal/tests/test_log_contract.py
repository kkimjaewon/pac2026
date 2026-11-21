from copy import deepcopy
import pytest
from pac_hal.log_contract import normalize,read_hal,TIMES


def sent(seq=1,session=42):
    row={k:str(10_000_000_000_000_001+i*10) for i,k in enumerate(TIMES)}
    row.update(session_id=str(session),command_seq=str(seq),source_result_id='7',result='sent',error_code='0')
    row['receiver_applied_mono_ns']=row['ack_received_mono_ns']=''
    return row


def ack(row):
    row=deepcopy(row);row.update(result='applied',receiver_applied_mono_ns=str(10_000_000_000_000_075),ack_received_mono_ns=str(10_000_000_000_000_090));return row


def test_sent_and_ack_are_one_command_and_use_integer_ns():
    s=sent();rows,stats=normalize([s,ack(s),ack(s)])
    assert len(rows)==1 and rows[0]['result']=='applied'
    assert rows[0]['b_to_c_ns']==10
    assert stats['ack_observed_commands']==1 and stats['identical_duplicate_success_rows']==1


def test_rejected_and_superseded_do_not_enter_latency_rows():
    s=sent();r=sent(2);r.update(result='rejected',error_code='5');old=sent(3);old['result']='superseded'
    rows,stats=normalize([s,r,old])
    assert len(rows)==1 and rows[0]['b_to_adoption_ns']==''
    assert stats['rejection_error_counts']=={'5':1} and stats['event_counts']['superseded']==1
    assert stats['ack_not_observed_commands']==1


def test_session_is_part_of_key_and_zero_source_is_preserved():
    a=sent(session=42);a['source_result_id']='0';b=sent(session=43)
    rows,_=normalize([a,b]);assert len(rows)==2 and rows[0]['source_result_id']=='0'


def test_transport_return_can_follow_target_adoption():
    s=sent();s['transport_end_mono_ns']=str(10_000_000_000_000_085)
    rows,stats=normalize([s,ack(s)])
    assert rows[0]['transport_to_adoption_ns']==24 and not stats['invalid_interval_counts']


def test_backward_interval_is_missing_and_reported():
    s=sent();s['validated_mono_ns']=s['b_created_mono_ns']
    rows,stats=normalize([s]);assert rows[0]['c_validation_ns']=='' and stats['invalid_interval_counts']['c_validation_ns']==1


@pytest.mark.parametrize('field',['source_result_id','b_created_mono_ns','transport_start_mono_ns'])
def test_mismatched_ack_identity_is_rejected(field):
    s=sent();a=ack(s);a[field]=str(int(a[field])+1)
    with pytest.raises(ValueError,match='identity'):normalize([s,a])


def test_incomplete_or_contradictory_ack_is_rejected():
    a=ack(sent());a['receiver_applied_mono_ns']=''
    with pytest.raises(ValueError,match='missing ACK'):normalize([a])
    a=ack(sent());b=deepcopy(a);b['ack_received_mono_ns']=str(int(b['ack_received_mono_ns'])+1)
    with pytest.raises(ValueError,match='ACK timestamps'):normalize([a,b])


def test_unknown_status_and_missing_header_fail_closed(tmp_path):
    s=sent();s['result']='arrived'
    with pytest.raises(ValueError,match='unknown'):normalize([s])
    p=tmp_path/'hal.csv';p.write_text('result,error_code\nsent,0\n')
    with pytest.raises(ValueError,match='columns'):read_hal(p)


def test_applied_before_sent_still_normalizes_once():
    s=sent();rows,stats=normalize([ack(s),s]);assert len(rows)==1 and rows[0]['ack_observed']==1 and stats['applied_without_sent_rows']==0


def test_ik_analyzer_does_not_count_replaced_targets_as_success(tmp_path):
    import importlib.util
    from pathlib import Path
    spec=importlib.util.spec_from_file_location('ik_analyzer',Path(__file__).resolve().parents[1]/'tools/analyze_ik_budget.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    p=tmp_path/'hal.csv';p.write_text('result,error_code,ik_wall_ns,ik_cpu_ns\nsent,0,10,8\napplied,0,10,8\nsuperseded,0,,\nrejected,5,700000,50000\n')
    result=module.analyze(p,500000)
    assert result['command_rows']==2 and result['groups']['0']['count']==1 and result['result_counts']['superseded']==1


def test_ai_restart_uses_actual_b_owner_and_skips_unpublished_stop():
    from pac_hal.log_contract import attach_ai_identity
    commands=[sent(1),sent(2),sent(3)]
    commands[2]['source_result_id']='0'
    b=[dict(command_seq=str(i),used_ai_seq='7',used_ai_boot_id=str(boot),has_used='1') for i,boot in [(1,10),(2,11)]]
    b.append(dict(command_seq='3',used_ai_seq='0',used_ai_boot_id='0',has_used='0'))
    a=[dict(seq='7',boot_id=str(boot),published='1') for boot in [10,11]]
    a.append(dict(seq='',boot_id='11',published='0'))
    rows,counts=attach_ai_identity(commands,b,a,42)
    assert [r['writer_boot_id'] for r in rows]==[10,11,'']
    assert counts=={'matched':2,'no_used_ai':1}


def test_ai_zero_seq_is_valid_and_missing_rows_not_guessed():
    from pac_hal.log_contract import attach_ai_identity
    c=sent();c['source_result_id']='0'
    b=[dict(command_seq='1',used_ai_seq='0',used_ai_boot_id='12',has_used='1')]
    rows,counts=attach_ai_identity([c],b,[dict(seq='0',writer_boot_id='12',published='1')],42)
    assert rows[0]['ai_join']=='matched'
    assert attach_ai_identity([c],b,[],42)[1]=={'missing_a_message':1}
    assert attach_ai_identity([c],[],[],42)[1]=={'missing_b_command':1}


def test_ai_join_rejects_conflicting_session_source_and_old_b_schema():
    from pac_hal.log_contract import attach_ai_identity
    b=[dict(command_seq='1',used_ai_seq='7',used_ai_boot_id='12',has_used='1')]
    with pytest.raises(ValueError,match='session'):attach_ai_identity([sent()],b,[],43)
    b[0]['used_ai_seq']='8'
    with pytest.raises(ValueError,match='source_result_id'):attach_ai_identity([sent()],b,[],42)
    del b[0]['used_ai_boot_id']
    with pytest.raises(ValueError,match='used_ai_boot_id'):attach_ai_identity([sent()],b,[],42)


def test_prepare_cli_emits_boot_identity_and_preserves_originals(tmp_path,monkeypatch):
    import csv,importlib.util,json
    from pathlib import Path
    spec=importlib.util.spec_from_file_location('prepare',Path(__file__).resolve().parents[1]/'tools/prepare_hal_log.py')
    tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
    hal=tmp_path/'hal.csv';rt=tmp_path/'rt.csv';ai=tmp_path/'ai.csv';out=tmp_path/'out'
    s=sent()
    with hal.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(s));w.writeheader();w.writerow(s)
    rt.write_text('command_seq,has_used,used_ai_seq,used_ai_boot_id\n1,1,7,12\n')
    ai.write_text('seq,boot_id,published\n7,12,1\n')
    original=hal.read_bytes()
    monkeypatch.setattr('sys.argv',['prepare',str(hal),'--output-dir',str(out),'--rt-csv',str(rt),'--ai-csv',str(ai),'--session-id','42'])
    tool.main()
    rows=list(csv.DictReader((out/'hal_commands.csv').open()))
    assert rows[0]['writer_boot_id']=='12' and rows[0]['ai_join']=='matched'
    assert json.loads((out/'hal_summary.json').read_text())['ai_join_counts']=={'matched':1}
    assert hal.read_bytes()==original
