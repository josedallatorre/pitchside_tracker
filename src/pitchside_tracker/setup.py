from setuptools import find_packages, setup
import os
import glob

package_name = 'pitchside_tracker'

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
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob.glob('launch/*.py')),
    ] + get_data_files('models', f'share/{package_name}/models') \
      + get_data_files('worlds', f'share/{package_name}/worlds'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='josedallatorre',
    maintainer_email='jose.dallatorre@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'spawn_sideline = pitchside_tracker.spawn_sideline:main',
            'tracking_node = pitchside_tracker.tracking_node:main',
        ],
    },
)
