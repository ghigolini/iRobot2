# ros_control_ws

Workspace dedicato ai componenti di controllo (controller, interfacce hardware) usati dal progetto.

## Scopo
Contiene package relativi al controllo, manipolazione del movimento ed esecuzione degli algoritmi.

## Struttura
- `src/` — sorgenti dei pacchetti di controllo.
- `build/` — output di `colcon build`.
- `install/` — artefatti installati.

## Uso rapido
0. Buildare ed eseguire prima il container create3_ws con il simulatore del robot.

1. Compila o manualmente o attraverso l'estensione il container Docker attraverso i file .devcontainer e DockerFile
   
2. Entrare nell'ambiente ROS (container o macchina con ROS):

```bash
source /opt/ros/jazzy/setup.bash
cd home/ws
colcon build
source install/local_setup.bash
```

3. Avviare nodi necessari:

```bash
ros2 launch bringup bringup.launch.py
```

4. Per eseguire la simulazione utilizzare i comandi presenti nel file comandi.

## Configurazione dei controller
- I file di configurazione dei controller (YAML) si trovano all'interno dei package sotto `src/bringup/config`.
