from setuptools import find_packages, setup

package_name = "dingo_pid_tuner"

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
        "console_scripts": ["dingo_pid_tuner_node = dingo_pid_tuner.dingo_pid_tuner_node:main"],
    },
)
