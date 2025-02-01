from setuptools import setup, find_packages

setup(
    name="cvFunctions",
    version="1.0.0",
    description="CV operations for robot pose detection",
    author="Timur Manshin",
    author_email="TManshin@yandex.com",
    maintainer="Timur Manshin",
    maintainer_email="TManshin@yandex.com",
    python_requires=">=3.10",
    install_requires=[
        "numpy",
        "opencv-contrib-python==4.10.0.84",
        "opencv-python==4.10.0.84"
    ],
    packages=find_packages(),
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
)
