from setuptools import find_packages, setup

package_name = "dingo_driver_simulation"

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
    maintainer="dingo",
    maintainer_email="jsdewolde@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dingo_driver_simulation_node = dingo_driver_simulation.dingo_driver_simulation_node:main"
        ],
    },
)
