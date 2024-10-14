from setuptools import setup

package_name = 'semifinal'

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
            'laberinto = semifinal.laberinto:main',
            'rosBagManual = semifinal.rosBagManual:main',
            'siguelineas = semifinal.siguelineas_puntos:main',
            'siguelineas2 = semifinal.siguelineas_alemany:main',
            'keyboard_teleop = semifinal.keyboard_teleop:main',
             'siguelineas_matriz = semifinal.siguelineas_matriz:main',
            'rectificador = semifinal.rectificador:main',
            'camara_sub = semifinal.camara_sub:main',
            'siguelineas_sim = semifinal.siguelineas_sim:main',
            'new_siguelineas = semifinal.new_siguelineas_puntos:main',
            'detect_text_sub_easyocr = pruebas.detect_text_sub_easyocr:main'
        ],
    },
)
