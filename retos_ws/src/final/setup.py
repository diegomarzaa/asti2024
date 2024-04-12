from setuptools import setup

package_name = 'final'

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
            'rosbag = final.rosbag:main',
            'keyboard_teleop = final.keyboard_teleop:main',
            'nintendo_teleop = final.nintendo_teleop:main',
            'nintendo_figs = final.nintendo_figs:main',
            
            'dibuja_figura = final.dibuja_figura:main',
            'siguelineas_sim = final.siguelineas_sim:main',
            'siguelineas_matriz = final.siguelineas_matriz:main',
            'bolos = final.bolos:main',
            'cuadricula = final.cuadricula:main',
            'mini_fabrica = final.mini_fabrica:main',
            'reconocer_figura_mapa = final.reconocer_figura_mapa:main',
            'laberinto = final.laberinto:main',
            'figuras = final.figuras:main',
            
            'distance_sensor = final.distance_sensor:main',
            
            'cam_pub = final.cam_pub:main',
            'camara_sub = final.camara_sub:main',
            'sensor_sub = final.Sensors:main',
            
            'Contar_lineas_cuadricula = final.contar_lineas_cuadricula:main',
            'new_siguelineas_puntos = final.new_siguelineas_puntos:main',

        ],
    },
)
