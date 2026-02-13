# IROBOT2

Workspace e codice dedicato allimplementazione fisica del progetto su Arduino Uno Q.

## Scopo
Contiene il progetto riadattato per l'utilizzo su ROS2 GALACTIC all'interno di un Arduino Uno Q mediante container Docker.  
Contiene inoltre cartelle e settaggi necessari per poterlo utilizzare ed analizzare anche localmente all'interno della propria macchina
attraverso l'utilizzo dell'estensione DevContainers di VSC e di Docker.

## Struttura
- `src/` — sorgenti dei pacchetti di controllo.
- `build/` — output di `colcon build`.
- `install/` — artefatti installati.

## Differenze con progetto di Simulazione
L'adattamento per galactic ha richiesto alcune modifiche al codice dovute alle versioni troppo vecchie di python e g++ disponibili su Galactic.  
L'assenza di un sesore LIDAR disponibile ci ha portati inoltre a modificare il comportamento della mappatura andando ad utilizzare gli IR presenti all'interno dell' iRobot Create 3 con una precisione nettamente inferiore rispetto al LIDAR.  
Sono stati inoltre cambiati i parametri di configurazione in modo da adattarsi al robot fisico e all'ambiente disponibile per i test.

## Uso in locale
0. Assicurarsi di aver installato Docker e l'estensione DevContainer di VSC.

1. Estrarre i file della cartella compressa VSC_Workspace_Config.zip (.devcontainer e .vscode) all'interno della root del progetto e modificare nella cartella .devcontainer il file devcontainer.json in modo che si adatti alla propria macchina in utilizzo.

2. Attraverso la barra superiore di VSC aprire il container mediante i comando
   ```bash
    >Dev Containers: Reopen in Container
    ```
   
3. A fine della configurazione effettuare il primo source per ROS2
   ```bash
    source /opt/ros/galactic/setup.bash
    ```

4. Buildare il progetto
   ```bash
    colcon build
    ```

5. Effettuare source per i nodi appena buildati
   ```bash
    source install/local_setup.bash
    ```

6. Avviare i nodi necessari
   ```bash
    ros2 launch bringup bringup.launch.py
    ```

## Uso su Arduino Uno Q
0. Assicurarsi che sia presente docker e docker-compose all'interno dell'Arduino Uno Q

1. All'interno dell'Uno Q creare una cartella e clonare all'interno il progetto.
   
2. Creare una cartella con nome ros_ws e spostare all'interno tutto il progetto ad eccezione della cartella compressa Docker_Uno_Q.zip.
   
3. Estrarre la cartella compressa Docker_Uno_Q e se necessario modificare il file docker-compose.yml con i dati specifici del proprio ambiente di lavoro.
   
4. Buildare il container mediante il comando
   ```bash
    docker compose build
    ```

5. Lanciare in container 
   ```bash
    docker compose up -d
    ```

6. Entrare nel container
   ```bash
    docker exec -it ros_create3 bash
    ```

7. Effettuare source per ros2
   ```bash
    source /opt/ros/galactic/setup.bash
    ```

8. Buildare il progetto
   ```bash
    cd ros_ws
    colcon build
    ```

9.  Effettuare source per i nodi appena buildati
    ```bash
    source install/local_setup.bash
    ```

10. Avviare i nodi necessari
    ```bash
    ros2 launch bringup bringup.launch.py
    ```
