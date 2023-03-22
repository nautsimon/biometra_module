from setuptools import setup, find_packages
import os
from glob import glob

package_name = "biometra_driver"

setup(
    name="biometra_driver",
    version="0.0.1",
    packages=find_packages(),
    data_files=[{package_name: ["dotnet/*.dll", "dotnet/*.xml"]},
            ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
    install_requires=["pythonnet"],
    zip_safe=True,
    python_requires=">=3.8",
    maintainer="Rafael Vescovi",
    maintainer_email="ravescovi@anl.gov",
    description="",
    url="https://github.com/AD-SDL/biometra_module/biometra_driver",
    license="MIT License",
    entry_points={ 
        'console_scripts': [
            'biometra_driver = biometra_driver.biometra_driver.biometra:main',
        ]
    },
)
