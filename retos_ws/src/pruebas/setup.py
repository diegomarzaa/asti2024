from setuptools import setup

package_name = 'pruebas'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diego',
    maintainer_email='al426641@uji.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_vision_gazebo = pruebas.test_vision_gazebo:main',
            'detect_text_pub = pruebas.detect_text_pub:main',
            'detect_text_sub = pruebas.detect_text_sub:main',
            'detect_text_sub_easyocr = pruebas.detect_text_sub_easyocr:main',
            'distance_sensor = pruebas.distance_sensor:main',
            'nintendo_to_ros = pruebas.nintendo_to_ros:main',
            'double_distance_sensor = pruebas.double_distance_sensor:main',
            'figuras = pruebas.pruebas_de_figuras:main',
            'cam_pub = pruebas.cam_pub:main',
            'prueba_mov = pruebas.prueba_mov:main',
            'cam_sub = pruebas.camara_sub:main',
            'lector_de_archivo = pruebas.lector_de_archivo:main',
            'lector_cmd_vel = pruebas.lector_cmd_vel:main',
            'detector = pruebas.detector:main',
            'avoidance = pruebas.avoidance:main',
            'follower = pruebas.follower:main',
        ],
    },
)
