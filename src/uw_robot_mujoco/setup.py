from setuptools import setup
from pathlib import Path
files=[('share/ament_index/resource_index/packages',['resource/uw_robot_mujoco']),('share/uw_robot_mujoco',['package.xml'])]
for directory in sorted({p.parent for p in Path('assets').rglob('*') if p.is_file()}):
    files.append(('share/uw_robot_mujoco/'+str(directory),[str(p) for p in directory.iterdir() if p.is_file()]))
setup(name='uw_robot_mujoco',version='0.1.0',packages=['uw_robot_mujoco'],data_files=files,
      install_requires=['setuptools','mujoco>=3.3.7','numpy'],tests_require=['pytest'],
      maintainer='zy',maintainer_email='zy@example.com',license='Proprietary',
      description='MuJoCo AUV plant adapter',
      entry_points={'console_scripts':['mujoco_node = uw_robot_mujoco.mujoco_node:main']})
