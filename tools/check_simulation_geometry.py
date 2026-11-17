"""Cross-check C++ allocation against MuJoCo's actual rotating sites and COM."""
from pathlib import Path
import importlib.util
import json
import subprocess
import numpy as np
import mujoco

root=Path(__file__).resolve().parents[1]
path=root/'src/uw_robot_mujoco/uw_robot_mujoco/model_geometry.py'
spec=importlib.util.spec_from_file_location('plant_geometry',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
model_path=root/'src/uw_robot_mujoco/assets/scene.xml'
geometry=module.allocation_geometry_parameters(model_path)
folder=root/'validation/geometry_check'
folder.mkdir(exist_ok=True)
cpp=r'''
#include "uw_robot_control/allocation/allocator.hpp"
#include <iomanip>
#include <iostream>
int main() {
  using namespace uw;
  AllocationGeometry g;g.rotating_sites=true;
  for(int j=0;j<3;++j)std::cin>>g.com[j];
  for(auto a:{&g.sites,&g.hinges,&g.axes,&g.child_centers})
    for(auto& v:*a)for(int j=0;j<3;++j)std::cin>>v[j];
  for(int i=0;i<4;++i)std::cin>>g.mass_fractions[i];
  double a,b;std::cout<<std::setprecision(17);
  while(std::cin>>a>>b) {
    auto matrix=wrench_matrix(V2(a,b),g);
    for(int i=0;i<6;++i)for(int j=0;j<4;++j)std::cout<<matrix(i,j)<<' ';
    std::cout<<'\n';
  }
}
'''
(folder/'check.cpp').write_text(cpp)
pkg=root/'src/uw_robot_control'
subprocess.run(['g++','-std=c++17','-O2','-I/usr/include/eigen3',
                '-I'+str(pkg/'include'),'-I'+str(root/'src/uw_robot_interfaces/include'),
                str(folder/'check.cpp'),str(pkg/'src/allocation/allocator.cpp'),
                '-o',str(folder/'check')],check=True)
parameters=[]
for key in ['geometry_center_of_mass','geometry_sites','geometry_hinges','geometry_axes',
            'geometry_child_centers','geometry_mass_fractions']:
    parameters.extend(geometry[key])
rng=np.random.default_rng(20261104)
angles=np.r_[[[0,0],[np.pi/2,np.pi/2],[-np.pi/2,-np.pi/2]],rng.uniform(-np.pi/2,np.pi/2,(97,2))]
text=' '.join(map(str,parameters))+'\n'+'\n'.join(' '.join(map(str,a)) for a in angles)+'\n'
result=subprocess.run([str(folder/'check')],input=text,text=True,capture_output=True,check=True)
matrices=np.loadtxt(result.stdout.splitlines()).reshape(-1,6,4)
model=mujoco.MjModel.from_xml_path(str(model_path));data=mujoco.MjData(model)
body=model.body('base').id
error=0.
for pair,calculated in zip(angles,matrices):
    for number,value in [('1',pair[0]),('3',pair[0]),('2',pair[1]),('4',-pair[1])]:
        data.qpos[model.jnt_qposadr[model.joint(number).id]]=value
    mujoco.mj_forward(model,data)
    actual=[]
    for name in ['thrust1','thrust3','thrust2','thrust4']:
        index=model.site(name).id
        direction=data.site_xmat[index].reshape(3,3)[:,0]
        actual.append(np.r_[direction,np.cross(data.site_xpos[index]-data.subtree_com[body],direction)])
    error=max(error,float(np.max(np.abs(calculated-np.array(actual).T))))
assert error<1e-10,error
report=dict(samples=len(angles),max_matrix_error=error)
(root/'validation/translation_geometry_after.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2),flush=True)
