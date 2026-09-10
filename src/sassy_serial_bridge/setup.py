from setuptools import setup
from glob import glob

setup(name='sassy_serial_bridge', version='0.1.0', packages=['sassy_serial_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/sassy_serial_bridge']), ('share/sassy_serial_bridge', ['package.xml'])],
      entry_points={'console_scripts': ['serial_bridge=sassy_serial_bridge.serial_bridge:main']})
