from setuptools import setup, find_packages

package_name = 'joy_pwm_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@example.com',
    description='Joy controller for PWM servo control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_pwm_controller = joy_pwm_controller.joy_pwm_controller:main'
        ],
    },
)