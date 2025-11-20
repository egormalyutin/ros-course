from setuptools import find_packages, setup

package_name = "file_publisher"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ok",
    maintainer_email="abv1236@yandex.ru",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={},
    entry_points={
        "console_scripts": ["file_publisher = file_publisher.file_publisher:main"],
    },
)
