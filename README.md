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

## Inicialização da simulação

Em um terminal, iniciar o ambiente simulado com o Husky no mundo office:

```bash
source /opt/ros/humble/setup.bash
ros2 launch clearpath_gz simulation.launch.py world:=office
```

O robô é inicializado automaticamente em uma posição válida do ambiente.

## Execução do sistema de percepção (YOLO)

Em outro terminal, iniciar o detector de pessoas baseado em YOLO:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch yolo_ros yolo.launch.py
```

# Execução do Comportamento Autônomo

Em um terminal Ubuntu, execute o comportamento principal com o comando:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch person_goal_navigator person_follow.launch.py side:=left
```

# Parâmetros Disponíveis
- **`side:=left`** → Robô posiciona-se à **esquerda** da pessoa
- **`side:=right`** → Robô posiciona-se à **direita** da pessoa

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
