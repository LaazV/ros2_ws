# ros_alvw_caregiver

[![ROS2](https://img.shields.io/badge/ROS2-Humble-green)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/Python-v3.7-blue)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-v22.04.4-red)](https://ubuntu.com/download)
[![Gazebo](https://img.shields.io/badge/Gazebo-v11.10-orange)](https://gazebosim.org/docs)

## Sobre
o objetivo deste trabalho é simular um robô acompanhante de hospital. A terefa simulada será a busca por alguma pessoa em um ambiente não mapeado, em seguida o retorno para a recepção do hospital.



## Procedimentos para a instalação do pacote

* No terminal, abra a area de trabalho ROS (ros2_ws), e em seguida o diretório src:
```
cd ~/ros2_ws/src
```
* Faça o clone do pacote do GitHub:
```
git clone https://github.com/viniciusbaltoe/ros_alvw_caregiver.git
```

NÃO ESTÁ FUNCIONANDO DIREITO:
```
cd ros_alvw_caregiver
chmod +x setup.sh
./setup.sh
export GAZEBO_MODEL_PATH=`pwd`/models:`pwd`/fuel_models
cd ..
```
* Faça o clone do pacote de exploração utilizado.
```
git clone https://github.com/robo-friends/m-explore-ros2.git
```
* Retorne ao diretório do WorkSpace (ros2_ws) e faça a atualização do colcon:
```
cd ~/ros2_ws && colcon build && source install/setup.bash
```

## Tutoriais de utilização do pacote

* Configuração inicial necessária para rodar o experimento:
```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```
* Launch do Ambiente Gazebo:
```
ros2 launch ros_alvw_caregiver hospital_launch.py headless:=False slam=True
```
Este launch é responsável por abrir o ambiente de simulação do hospital no Gazebo e acrescentar o robô nas coordenadas (x = 0.0, y = 0.0).

* Launch da tarefa:
```
ros2 launch ros_alvw_caregiver caregiver_launch.py
```
