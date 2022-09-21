from setuptools import setup, find_packages

# Install with 'pip install -e .'

setup(
    name="gym_ghumanoid_pybullet",
    version="0.1.0",
    author="Alexandre Chenu, Nicolas Perrin-Gilbert",
    description="Humanoid robot locomotion environment for goal-conditioned RL (with OpenAI Gym interface)",
    url="https://github.com/AlexandreChenu/ghumanoid_dcil_pybullet",
    packages=find_packages(),
    install_requires=[
        "gym>=0.22.0",
        "torch>=1.10.0",
        "matplotlib>=3.1.3",
    ],
    license="LICENSE",
)
