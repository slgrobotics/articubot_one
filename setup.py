from setuptools import setup

package_name = 'articubot_one'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.launch_utils'],
    package_dir={'': '.'},
    install_requires=['setuptools'],
    zip_safe=True,
)
