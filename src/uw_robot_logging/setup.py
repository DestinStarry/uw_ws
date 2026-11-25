from setuptools import find_packages, setup
setup(name="uw_robot_logging", version="0.1.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/uw_robot_logging"]),
                  ("share/uw_robot_logging", ["package.xml"])],
      install_requires=["setuptools"], tests_require=["pytest"], zip_safe=True,
      maintainer="zy", maintainer_email="zy@example.com", license="Proprietary",
      description="Independent bounded CSV flight logger with raw events and parameter snapshots",
      entry_points={"console_scripts":["flight_logger_node = uw_robot_logging.node:main",
                                        "log_summary = uw_robot_logging.summarize:main"]})
