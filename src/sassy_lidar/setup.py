from setuptools import setup
setup(name='sassy_lidar', version='0.1.0', packages=['sassy_lidar'], data_files=[('share/ament_index/resource_index/packages',['resource/sassy_lidar']),('share/sassy_lidar',['package.xml'])], entry_points={'console_scripts':['d6_scan=sassy_lidar.d6_scan:main']})
