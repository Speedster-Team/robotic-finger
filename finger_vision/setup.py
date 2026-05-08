from pathlib import Path

from setuptools import find_packages, setup

package_name = 'finger_vision'


def recursive_files(prefix, path):
    """
    Recurse over path returning a list of tuples suitable for use with setuptools data_files.

    :param prefix: prefix path to prepend to the path
    :param path: Path to directory to recurse. Path should not have a trailing '/'
    :return: List of tuples. First element of each tuple is destination path,
        second element is a list of files to copy to that path
    """
    return [(str(Path(prefix)/subdir),
            [str(file) for file in subdir.glob('*') if not file.is_dir()])
            for subdir in Path(path).glob('**')]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *recursive_files('share/' + package_name, 'launch'),
        *recursive_files('share/' + package_name, 'config'),

    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mjenz',
    maintainer_email='michael.jenz77@gmail.com',
    description='Identify and publish waypoints using CV',
    license=' Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
