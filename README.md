# Trabalho 2 – Husky Person Follow  
ROS 2 Humble + Gazebo (world office) 

## Visão geral
Este projeto implementa um comportamento totalmente autônomo para o robô Clearpath Husky, no qual o robô:

- é inicializado em um ambiente simulado no Gazebo (world office);
- encontra e identifica uma pessoa posicionada aleatoriamente no ambiente;
- se aproxima da pessoa;
- se posiciona ao lado dela (direita ou esquerda);
- sinaliza que a pessoa foi localizada;
- finaliza a missão;
- gera automaticamente um arquivo de relatório em texto contendo a localização aproximada da pessoa e o tempo total de execução.

O sistema não utiliza teleoperação nem comandos manuais durante a missão.

---

## Arquitetura do sistema
O projeto é dividido em três componentes principais:

1. Simulação (externa ao repositório)
   - Gazebo / Gazebo Sim
   - Robô Clearpath Husky
   - Mundo office

2. Pacote ROS desenvolvido neste trabalho
   - husky_person_follow
   - Contém:
     - detector de pessoa (simulado);
     - máquina de estados (FSM);
     - geração do relatório final;
     - arquivos de launch.

3. Bridge Gazebo para ROS
   - Responsável por encaminhar a pose da pessoa do Gazebo para o ROS 2.

---

## Requisitos do ambiente
O computador que irá rodar a simulação precisa ter:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo / Gazebo Sim compatível com Humble
- Pacotes instalados:
  - clearpath_gz
  - ros_gz_sim
  - ros_gz_bridge

Verificação rápida:
```bash
ros2 pkg list | grep clearpath_gz
ros2 pkg list | grep ros_gz
