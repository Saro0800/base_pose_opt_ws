from setuptools import setup, find_packages

setup(
    name="reach_space_modeling",
    version="0.0.0",
    packages=find_packages(where="src"),  # Indica che i pacchetti sono in "src"
    package_dir={"": "src"},  # Dice a setuptools di guardare in "src"
    install_requires=[],
)
