from setuptools import find_packages, setup
setup(
    name='JointLimitsDMP',
    packages=find_packages(include=['JointLimitsDMP']),
    version='0.1.0',
    description='Dynamic Motion primitives with joint limits',
    author='Elisabeth Menendez',
    license='MIT',
    setup_requires=['pytest-runner'],
    tests_require=['pytest==4.4.1'],
    test_suite='tests',
)