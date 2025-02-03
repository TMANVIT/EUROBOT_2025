from setuptools import setup, find_packages
import pathlib

here = pathlib.Path(__file__).parent.resolve()

long_description = (here / "README.md").read_text(encoding="utf-8")

setup(
    name="cvFunctions",
    version="1.0.0",
    description="CV operations for robot pose detection",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Timur Manshin",
    author_email="TManshin@yandex.com",
    keywords="cvFunctions",
    maintainer="Timur Manshin",
    maintainer_email="TManshin@yandex.com",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    python_requires=">=3.10",
    install_requires=[
        "numpy",
        "opencv-contrib-python==4.10.0.84",
        "opencv-python==4.10.0.84"
    ],
)