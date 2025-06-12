import os
import sys
import sysconfig
import platform
from setuptools import setup, find_packages

PACKAGE_NAME = "autolife_robot_inspection"

# Define your base version
PACKAGE_BASE_VERSION = "1.0.0"

# Get the run number from the environment variable
run_number = os.environ.get('GITHUB_RUN_NUMBER')

# Append the run number as a build identifier if it exists
if run_number:
    version = f"{PACKAGE_BASE_VERSION}+build{run_number}"
else:
    # Fallback version if the environment variable isn't set (e.g., local build)
    version = PACKAGE_BASE_VERSION

# Determine the operating system
is_windows = platform.system() == 'Windows'
is_linux = platform.system() == 'Linux'
is_macos = platform.system() == 'Darwin'

# Set the classifiers based on the operating system
classifiers = [
    f"Programming Language :: Python :: {sys.version_info.major}.{sys.version_info.minor}",
]
if is_windows:
    classifiers.append("Operating System :: Microsoft :: Windows")
if is_linux:
    classifiers.append("Operating System :: POSIX :: Linux")
if is_macos:
    classifiers.append("Operating System :: MacOS :: MacOS X")

if 'bdist_wheel' in sys.argv:
    if not any(arg.startswith('--python-tag') for arg in sys.argv):
        sys.argv.extend(
            ['--python-tag', f'py{sys.version_info.major}{sys.version_info.minor}'])
    if not any(arg.startswith('--plat-name') for arg in sys.argv):
        sys.argv.extend(
            ['--plat-name', sysconfig.get_platform().replace('-', '_')])


def add_extra_data_dir(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            folder_prefix = ''
            if is_windows:
                folder_prefix = f'Lib/site-packages/{PACKAGE_NAME}'
            elif is_linux:
                folder_prefix = f'lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages/{PACKAGE_NAME}'
            elif is_macos:
                folder_prefix = f'lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages/{PACKAGE_NAME}'
            else:
                raise NotImplementedError(
                    f"Unsupported operating system: {platform.system()}")
            paths.append((os.path.join(folder_prefix, path),
                         [os.path.join(path, filename)]))
    return paths


setup(
    name=f"{PACKAGE_NAME}",
    version=version,  # Use the dynamically generated version
    author="Yang jianbin",
    author_email="jianbin.yang@autolife.ai",
    description="Autolife Robot Inspection",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/AutoLifeRobot/AutolifeRobotInspection",
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    package_data={
        # Include any .so, .dll or .pyd files found in any package
        '': ['*.so', '*.dll', '*.pyd', '*.example'],
    },
    install_requires=[],
    python_requires=f"~={sys.version_info.major}.{sys.version_info.minor}.0",
    classifiers=classifiers,
    include_package_data=True,
    data_files=[
        *add_extra_data_dir('model'),
        *add_extra_data_dir('configs'),
        *add_extra_data_dir('scripts'),
        *add_extra_data_dir('systemd_srv'),
    ]
)
