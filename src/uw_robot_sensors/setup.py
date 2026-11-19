from setuptools import find_packages, setup

setup(name='uw_robot_sensors', version='0.2.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/uw_robot_sensors']),
                  ('share/uw_robot_sensors', ['package.xml'])],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      maintainer='zy', maintainer_email='zy@example.com', license='Proprietary',
      description='IMU and OpenCV stereo camera sensors',
      entry_points={'console_scripts': [
          'imu_node = uw_robot_sensors.imu.imu_node:main',
          'depth_node = uw_robot_sensors.depth.depth_node:main',
          'camera_node = uw_robot_sensors.camera.camera_node:main',
      ]})
