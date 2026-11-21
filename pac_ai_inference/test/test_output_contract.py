"""A output contract: real ROS message type, controlled local monotonic clock."""
import csv
from types import SimpleNamespace
from unittest.mock import patch
from ai_inference.command_publisher import CommandPublisher
from ai_inference.fault_injection import FaultyPublisher
from ai_inference.detector_switch import DetectorSwitch, DetectorConfig, FrameInput
from intelion_interfaces.msg import ControlCommand
import pytest

class Node:
    def __init__(self, path):
        self.parameters=dict(log_dir=str(path),run_id='',fault_mode='none',fault_delay_ms=201)
        self.messages=[]
    def create_publisher(self,*args):return self
    def publish(self,msg):self.messages.append(msg)
    def get_parameter(self,name):return SimpleNamespace(value=self.parameters[name])
    def get_logger(self):return SimpleNamespace(info=lambda msg: None)

def publisher(tmp_path):
    node=Node(tmp_path)
    return node,CommandPublisher(node,confidence_min=.5,max_age_ns=200000000)

@pytest.mark.parametrize('age,status',[(200000000,0),(200000001,2),(-1,2)])
def test_age_boundary(tmp_path,age,status):
    node,p=publisher(tmp_path)
    with patch('ai_inference.command_publisher.time.monotonic_ns',return_value=400000000):
        msg=p.publish(0,0,[.5,0,.71,0],1.,400000000-age)
    assert msg.status==status and (msg.confidence==0 if status else msg.confidence==1)

@pytest.mark.parametrize('value',[float('nan'),float('inf'),-float('inf')])
def test_invalid_geometry_cannot_be_usable(tmp_path,value):
    node,p=publisher(tmp_path)
    msg=p.publish(0,0,[value,0,.71,0],1.,__import__('time').monotonic_ns())
    assert msg.status==2 and msg.confidence==0 and list(msg.target)==[0.]*4

def test_stop_resume_preserves_boot_seq_and_csv_identity(tmp_path):
    node,p=publisher(tmp_path);writer=FaultyPublisher(p,node)
    import time
    first=writer.publish(0,0,[.5,0,.71,0],1.,time.monotonic_ns())
    node.parameters['fault_mode']='stop'
    assert writer.publish(0,0,[.5,0,.71,0],1.,time.monotonic_ns()) is None
    node.parameters['fault_mode']='none'
    last=writer.publish(0,0,[.5,0,.71,0],1.,time.monotonic_ns());writer.close()
    assert last.seq==first.seq+1 and last.writer_boot_id==first.writer_boot_id
    rows=list(csv.DictReader(open(writer.path)))
    assert rows[1]['published']=='0' and rows[1]['seq']=='' and rows[1]['publish_mono_ns']==''
    for row,msg in [(rows[0],first),(rows[2],last)]:
        assert int(row['seq'])==msg.seq and int(row['boot_id'])==msg.writer_boot_id
        assert int(row['publish_mono_ns'])==int(row['capture_mono_ns'])+msg.age_ns

def test_both_missed_and_primary_failure():
    switch=DetectorSwitch(DetectorConfig())
    missed=switch.step(FrameInput(0))
    assert missed.xyz is None and missed.confidence==0
    fallback=switch.step(FrameInput(.1,p_xyz=None,v_xyz=(.5,0,.71),v_valid_ratio=1.))
    assert fallback.xyz==(.5,0,.71) and fallback.confidence==.6 and fallback.source=='V'
