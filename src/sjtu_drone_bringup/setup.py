from setuptools import setup
import glob
import os

package_name = 'sjtu_drone_bringup'


# Helper to flatten files in a directory
def get_data_files(src_folder, dst_folder):
    files = []
    for path in glob.glob(f'{src_folder}/**', recursive=True):
        if os.path.isfile(path):
            rel_path = os.path.relpath(path, src_folder)
            dest_path = os.path.join(dst_folder, os.path.dirname(rel_path))
            files.append((dest_path, [path]))
    return files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob.glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, "rviz"), glob.glob('rviz/*.rviz')),
        (os.path.join('share', package_name, "config"), glob.glob('config/*.yaml')),
    ] + get_data_files('models', f'share/{package_name}/models') \
      + get_data_files('worlds', f'share/{package_name}/worlds'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='georg.novtony@aon.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_drone = sjtu_drone_bringup.spawn_drone:main',
        ],
    },
)
