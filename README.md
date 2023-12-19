# ros_alvw_caregiver

[![ROS2](https://img.shields.io/badge/ROS2-Humble-green)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/Python-v3.7-blue)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-v22.04.4-red)](https://ubuntu.com/download)
[![Gazebo](https://img.shields.io/badge/Gazebo-v11.10-orange)](https://gazebosim.org/docs)

## Sobre
o objetivo deste trabalho é simular um robô acompanhante de hospital. A terefa simulada será a busca por alguma pessoa em um ambiente não mapeado, em seguida o retorno para a recepção do hospital.



## Procedimentos para a instalação do pacote

* No terminal, abra a pasta $HOME
```
cd ~/
```
* Clone este repositório
```
git clone https://github.com/LaazV/ros2_ws.git
```

- Baixe o arquivo `yolo_detector.zip`, e extraia na pasta ros2_ws

```
https://drive.google.com/file/d/1WOc4_xbdNo7XR8pc3GYMumKJ3j6VDPEN/view?usp=sharing
```

- A estrutura do projeto deverá ficar:

```
- ros2_ws/
-- src/
-- yolo_detector/
-- start
-- README.md
```

- Certifique-se que o arquivo `start` seja executável

```
chmod +x start
```

Instale o pacote `tf-transformations` no ubuntu

```
sudo apt update && sudo apt install ros-humble-tf-transformations 
```

- E o pacote `transforms3d` do python

```
pip install transforms3d
```

## Tutoriais de utilização do pacote

Para rodar o pacote, será necessário 4 terminais. 

Abra 4 terminais. Em cada terminal, entre no workspace do ros, e execute o script `start` 

```
cd $HOME/ros2_ws 
./start
```

No primeiro terminal, inicie o gazebo digitando:

```
term1
```

Aguarde o gazebo iniciar, e depois aguarde mais alguns segundos para o robô se orientar.

Em seguida, no segundo terminal, inicie a navegação autonoma digitando:

```
term2
```

No terceiro terminal, inicie os procedimentos de navegação ao detectar um humano com o comando

```
term3
```

E no quarto terminal, inicie a detecção com o comando

```
yolo
```
