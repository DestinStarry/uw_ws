import pytest
from uw_robot_sensors.system_monitor import read_cpu_temperature

def zone(root,name,kind,temp):
 p=root/name;p.mkdir(exist_ok=True)
 (p/"type").write_text(kind);(p/"temp").write_text(temp)
 return p

def test_cpu_selection_and_recovery(tmp_path):
 zone(tmp_path,"thermal_zone0","GPU-therm","99000")
 p=zone(tmp_path,"thermal_zone1","CPU-therm","42500")
 assert read_cpu_temperature(tmp_path)==(42.5,str(p))
 (p/"temp").unlink()
 with pytest.raises(ValueError):read_cpu_temperature(tmp_path)
 (p/"temp").write_text("45000")
 assert read_cpu_temperature(tmp_path)[0]==45

@pytest.mark.parametrize("value",["nan","inf","broken","300000","-200000"])
def test_bad_reading(tmp_path,value):
 zone(tmp_path,"thermal_zone0","CPU-therm",value)
 with pytest.raises(ValueError):read_cpu_temperature(tmp_path)

def test_no_cpu_and_bad_explicit_zone(tmp_path):
 zone(tmp_path,"thermal_zone0","GPU-therm","45000")
 for name in ["auto","../thermal_zone0","thermal_zone0"]:
  with pytest.raises(ValueError):read_cpu_temperature(tmp_path,name)
