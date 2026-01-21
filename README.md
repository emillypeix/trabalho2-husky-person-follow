# Trabalho 2 – Husky Person Follow (YOLO)
ROS 2 Humble + Gazebo (world office)

## Visão geral
Este projeto implementa um comportamento 100% autônomo para o robô Clearpath Husky em ambiente simulado, utilizando percepção baseada em visão computacional com YOLO.

O robô é capaz de:

- ser inicializado em uma posição aleatória do ambiente simulado;
- detectar e identificar uma pessoa posicionada aleatoriamente no mundo;
- se aproximar da pessoa detectada;
- se posicionar ao lado da pessoa (direita ou esquerda);
- sinalizar quando a pessoa é localizada;
- finalizar a missão de forma autônoma;
- executar toda a tarefa sem teleoperação ou comandos externos.

A detecção de pessoas é realizada por meio de redes neurais convolucionais utilizando YOLO, integradas ao ROS 2 através do pacote `yolo_ros`.

## Arquitetura do Sistema

O sistema é composto pelos seguintes blocos principais:

### 1. Simulação
- Gazebo / Gazebo Sim
- Mundo `office`
- Robô Clearpath Husky

### 2. Navegação
- Nav2
- Localização e mapeamento
- Planejamento de trajetória

### 3. Percepção
- YOLO para detecção de pessoas
- Pacote `yolo_ros`

## Requisitos do ambiente
Para executar o projeto, o sistema deve possuir:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo / Gazebo Sim compatível com ROS 2 Humble
- Clearpath Gazebo packages
- Nav2
- Dependências ROS Gazebo:
  - `ros_gz_sim`
  - `ros_gz_bridge`

## Instalação e configuração do ambiente
Esta seção descreve os passos necessários para preparar o ambiente de execução do projeto, conforme a documentação oficial da Clearpath Robotics e do Gazebo.

### Instalação dos pacotes Clearpath (Offboard PC)
Comandos baseados na documentação oficial:
https://docs.clearpathrobotics.com/docs/ros2humble/ros/installation/offboard_pc

```bash
wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -

sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'

sudo apt-get update
```

Adicionar a lista do rosdep da Clearpath:

```bash
sudo wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list \
  -O /etc/ros/rosdep/sources.list.d/50-clearpath.list

rosdep update
```

### Instalação do simulador Gazebo (Ignition Fortress)
Comandos baseados na documentação oficial:
https://docs.clearpathrobotics.com/docs/ros2humble/ros/tutorials/simulator/install

```bash
sudo apt-get update && sudo apt-get install wget

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install ignition-fortress
```

### Ferramentas adicionais 
```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-vcstool
```

### Pacote de navegação (Nav2)
O sistema utiliza o Nav2 como algoritmo de navegação do ROS 2, incluindo localização, planejamento e controle.
O pacote Nav2 deve ser instalado conforme a documentação oficial do ROS 2 Humble.

### Compilação do workspace do trabalho (TR_ws)
O workspace do trabalho (TR_ws) consiste nos seguintes pacotes:

- pacotes Clearpath;
- pacotes de navegação (Nav2);
- pacotes de exploração;
- pacotes de percepção (YOLO);
- pacotes personalizados do trabalho.

## Estrutura do repositório
O repositório contém um workspace ROS completo:
```text
src/
├── clearpath_common
├── clearpath_config
├── clearpath_msgs
├── clearpath_nav2_demos
├── clearpath_simulator
├── clearpath_viz
├── person_goal_navigator
├── yolo_ros
└── Autonomous-Explorer-and-Mapper-ros2-nav2
```

## Compilação do workspace
Após clonar o repositório, executar:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

# Execução do Comportamento Autônomo

Em um terminal Ubuntu, execute o sistema completo (simulação, navegação e comportamento) com um único arquivo launch:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch person_goal_navigator bringup_all.launch.py
```

Este launch inicializa automaticamente:

- o robô Husky no ambiente simulado;
- os sensores necessários (câmera, laser e IMU);
- o sistema de navegação (Nav2);
- o sistema de percepção baseado em YOLO;
- o comportamento autônomo de aproximação e posicionamento ao lado da pessoa.

# Funcionamento do Sistema

1. **O robô é inicializado no ambiente simulado**
2. **O YOLO detecta a pessoa**
3. **A posição da pessoa é estimada**
4. **O objetivo é enviado ao sistema de navegação**
5. **O robô navega até a pessoa**
6. **O robô se posiciona ao lado configurado**
7. **A missão é concluída automaticamente**

# Considerações Finais

Este projeto demonstra a integração entre:

- **Percepção por visão computacional (YOLO)**
- **Navegação autônoma com Nav2**
- **Simulação no Gazebo**
- **Robô móvel Clearpath Husky**

## Característica Principal

**Todo o sistema opera de forma totalmente autônoma.**
