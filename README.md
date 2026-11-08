# Control de impedancia cartesiana para Kinova Gen3

Controlador de impedancia cartesiana en C++ para el brazo **Kinova Gen3 de 7 GDL** con
pinza **Robotiq 2F-85**, y una secuencia de tarea que ejecuta de forma automática un
ensamble: tomar una placa, colocarla en un *fixture* giratorio, tomar un tornillo y
soltarlo en un hoyo de la placa. Las posiciones de las piezas las entrega un módulo de
visión que corre como proceso aparte.

El programa no usa ROS: se comunica directamente con el robot mediante la API Kortex en
**servocontrol de bajo nivel** y envía la corriente de cada motor en un ciclo de
aproximadamente **1 kHz**.

## Características

- **Ley de impedancia cartesiana**: el efector se comporta como un resorte-amortiguador
  hacia la pose deseada (`τ = g(q) + Jᵀ(−KΔξ − D J q̇) + τ_null + τ_fric`), con error de
  orientación por cuaterniones y amortiguamiento `D = 2ζ√K`.
- **Modelo analítico propio** del Gen3: cinemática directa, jacobiano y vector de gravedad
  en forma cerrada.
- **Espacio nulo** con pesos por articulación y postura por pose para controlar el codo
  sin perturbar la tarea.
- **Compensación de fricción** con un observador LuGre (RK4), dirigida por el par del
  resorte y desactivada en el objetivo para evitar vibración.
- **Rigidez variable cerca del objetivo** y zona de solo gravedad al llegar.
- **Secuencia de tarea** como máquina de estados probada sin robot: movimientos, pinza,
  *fixture*, esperas y pasos de visión, con modo automático o paso a paso, pausas y
  validación completa antes de conectar el robot.
- **Protecciones**: saturación de par, barrera de corriente, limitador de variación del
  par, comprobación de valores finitos en cada ciclo y transiciones suaves con solo
  gravedad.
- **Interfaz gráfica** (PyQt5 + VTK) como capa encima del controlador: robot en 3D,
  estado de la secuencia, análisis de corridas y control desde el celular.
- **Simulación** en MATLAB/Simulink del mismo controlador con el URDF del Gen3.

## Arquitectura

```
 Interfaz gráfica (PyQt5) ──UDP 5011──►  impedance_gen3  ◄──UDP 5006──►  Módulo de visión
  · celular (HTTP 8420)                  ├─ hilo GTK (slider, botones)    (vision_server/)
                                         ├─ hilo de control ~1 kHz
                                         │   modelo → impedancia → fricción
                                         │   → secuencia → corriente
                                         └─ hilo de comandos de la GUI
                                                │ API Kortex (TCP/UDP)     │ USB serie
                                                ▼                          ▼
                                     Kinova Gen3 + Robotiq 2F-85    ESP32 del fixture
```

## Estructura del repositorio

| Ruta | Contenido |
|---|---|
| `src/main_impedance.cpp` | Programa principal: conexión, ciclo de control, fricción, secuencia, tablero y registro. |
| `src/cartesian_impedance_controller.cpp`, `include/cartesian_impedance_controller.hpp` | Clase del controlador (ley de impedancia, espacio nulo, filtros). |
| `include/controller_config.hpp` | **Configuración**: ganancias, poses, caja de trabajo y pasos de la tarea. |
| `include/task_sequence.hpp` | Máquina de estados de la secuencia (`task::Sequencer`). |
| `include/fixture_client.hpp`, `include/screw_vision_client.hpp` | Clientes del *fixture* (serie) y del módulo de visión (UDP). |
| `Gen3_model.h` | Modelo analítico del robot y constantes de los motores. |
| `KinovaRobot/`, `Matrix/`, `DataLogging/` | Envoltura de Kortex, álgebra matricial y registro CSV. |
| `tests/` | Pruebas sin robot (modelo, jacobiano, espacio nulo, secuencia). |
| `tools/` | Utilidades: `task_info`, `gripper_test`, `fixture_test`, análisis de CSV. |
| `gui/` | Interfaz gráfica y servidor para el celular (`gui/movil`). |
| `matlab/` | Simulaciones en MATLAB y Simulink. |
| `vision_lib/` | Biblioteca de visión: calibración de la cámara, nube de puntos, segmentación por color, GPD y selección del agarre (incluye GPD en `vision_lib/third_party/gpd`). |
| `vision_server/` | Servidor de visión (`screw_server`) que atiende al controlador; también detecta el hoyo. |
| `fixture/firmware/` | Firmware del ESP32 del *fixture*: identificación, PI de corriente y control adaptable (`control_adaptable.ino`, el que usa la tarea). |
| `fixture/matlab/` | Scripts de adquisición, identificación, sintonía y análisis de los ensayos del *fixture*. |

## Requisitos

- Linux x86-64 (probado en Ubuntu 24.04), g++ con C++20 y CMake ≥ 3.16.
- Eigen 3 y GTK 3:
  ```bash
  sudo apt install build-essential cmake libeigen3-dev libgtk-3-dev
  ```
- La **API Kortex 2.7.0** y **Boost 1.87** se descargan automáticamente con CMake la
  primera vez (se necesita internet).
- Interfaz gráfica (opcional): `python3-pyqt5`, `python3-pyqt5.qtsvg`, `python3-vtk9` y
  `pyqtgraph`.

## Obtener el código

```bash
git clone https://github.com/beetozpAG/kinova-gen3-impedance-control.git
cd kinova-gen3-impedance-control
```

También puede descargarse como ZIP desde el botón **Code → Download ZIP** de GitHub.

## Compilación

Desde la carpeta del repositorio:

```bash
cmake -S . -B build
cmake --build build -j8
```

Los ejecutables quedan en `build/`: `impedance_gen3` (el controlador), las pruebas y las
herramientas.

## Pruebas sin robot

No se conectan al robot; conviene correrlas después de cada cambio en la configuración.

```bash
cd build
./model_smoke_test
./jacobian_validation_test
./nullspace_test
./task_sequence_test
./task_info          # imprime en JSON la secuencia compilada y valida la configuración
```

## Ejecución

> ⚠️ **Seguridad.** `impedance_gen3` envía corriente a los motores del robot real. Tenga
> el paro de emergencia al alcance y el área de trabajo libre. El control arranca
> apagado (solo compensación de gravedad) y se activa con el *slider* de la ventana.

1. Conecte la PC al robot por Ethernet (por defecto el robot está en `192.168.1.10`;
   se cambia en `ROBOT_IP`, en `src/main_impedance.cpp`).
2. Configure el TCP de la herramienta a 0.12 m en la aplicación web de Kinova.
3. Si la tarea usa visión, arranque antes el servidor de visión.
4. Ejecute `./build/impedance_gen3`, active el *slider* y pulse **Siguiente paso**.

También puede usarse la interfaz gráfica:

```bash
cd gui
python3 -m venv --system-site-packages .venv
.venv/bin/pip install --no-deps "pyqtgraph==0.13.7"
./run_gui.sh
```

## Adaptar a otra celda de trabajo

Casi todo lo que depende del laboratorio está en `include/controller_config.hpp`:

- **Poses**: se copian de la aplicación web de Kinova (posición del TCP, ángulos
  `Rx Ry Rz` y las siete articulaciones).
- **Caja de trabajo**, distancia máxima por movimiento y límites de las poses calculadas.
- **Pasos de la tarea**: una línea por paso (`moveTo`, `gripper`, `fixtureTo`, `waitFor`,
  pasos de visión).
- **Ganancias**: rigidez base y cerca del objetivo, amortiguamiento, espacio nulo y
  fricción.

Después de cualquier cambio: recompilar y correr `task_info` y `task_sequence_test`.

## Módulo de visión

`vision_lib/` contiene la biblioteca de visión (C++ con PCL, OpenCV y GPD) y
`vision_server/` el servidor que atiende al controlador. Requiere además
`libpcl-dev`, `libopencv-dev`, `libjsoncpp-dev` y `libx11-dev`. El servidor se compila
dentro de `vision_lib/`, porque las rutas de sus archivos `cfg/` empiezan en `../`:

```bash
cmake -S vision_server -B vision_lib/build_modular -DKG_DIR=$PWD/vision_lib \
      -DKORTEX_DIR=$PWD/build/_deps/kortex_api_rel-src
cmake --build vision_lib/build_modular -j8
cd vision_lib/build_modular && ./screw_server ../cfg/gpd_params.cfg
```

Los colores de las piezas (rangos HSV) están en `vision_lib/cfg/plate_colors.cfg` y se
leen sin recompilar. El controlador solo se conecta al servidor si la tarea tiene pasos
de visión.

## *Fixture*

`fixture/firmware/control_adaptable.ino` es el firmware del ESP32 (motor de CD de 12 V
con encoder, puente H L298N y sensor INA226): lazo de corriente PI a 4 kHz con
linealización por mapa estático (`mapa_corriente.h`) y lazo de posición PD adaptable a
1 kHz con zona muerta. Se comunica por USB serie a 921 600 bauds con comandos de texto
(`ZERO`, `START`, `POS <grados>`, `STOP`, `STATUS`). Los otros dos firmwares y los
scripts de `fixture/matlab/` sirven para identificar la planta y sintonizar el PI con
otro motor.

## Simulación

`matlab/` contiene la simulación del controlador con el URDF del Gen3 (MATLAB R2026a y
Robotics System Toolbox): contacto con una pared elástica, liberación de la pared y el
mismo escenario en Simulink (`matlab/simulink/gen3_pared_retira_2s.slx`).

## Créditos

Proyecto Modular, Centro Universitario de Ciencias Exactas e Ingenierías (CUCEI),
Universidad de Guadalajara.

- Alberto Carlos Aguirre Gonzalez
- Antony Hernan Lopez Chuan
- Carlos Omar Rodriguez Vazquez
- Asesor: Carlos Ivan Aldana Lopez

La ley de control toma como referencia la implementación de M. Mayr y J. M. Salt-Ducaju,
*A C++ implementation of a Cartesian impedance controller for robotic manipulators*
(arXiv:2212.11215, 2022).

## Licencia

Código bajo licencia [MIT](LICENSE). Los componentes de terceros conservan su propia
licencia: el modelo URDF del Gen3 (Kinova, BSD-3, `matlab/URDFFiles`), `low_latency_list`
(MIT, `DataLogging/low_latency_list`), GPD (BSD-2, `vision_lib/third_party/gpd`) y las
fuentes IBM Plex (SIL OFL 1.1, `gui/fonts`).
La API Kortex y Boost se descargan durante la compilación bajo sus propias licencias.
