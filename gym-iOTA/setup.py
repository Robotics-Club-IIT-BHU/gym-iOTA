import setuptools
from pathlib import Path

setuptools.setup(
    name='gym_iOTA',
    version='0.0.3',
    description="A Open AI Gym Env for a Multiagent systems with our bot \'iOTA\'",
    long_description=Path("README.md").read_text(),
    long_description_content_type="text/markdown",
    packages=setuptools.find_packages(include="gym_iOTA*"),
    install_requires=['gym','opencv-python','pybullet','Pillow'],
    package_data={'':['absolute/meshes/*.STL','absolute/iota.urdf']}
)
