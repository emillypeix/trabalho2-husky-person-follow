# Trabalho 2 – Husky Person Follow
ROS 2 Humble + Gazebo (world office)

## Visão geral
Este projeto implementa um comportamento 100% autônomo para o robô Clearpath Husky, no qual o robô:

- é inicializado em um ambiente simulado no Gazebo (world office);
- encontra e identifica uma pessoa posicionada aleatoriamente no ambiente;
- se aproxima da pessoa;
- se posiciona ao lado dela (direita ou esquerda);
- sinaliza que a pessoa foi localizada;
- finaliza a missão;
- gera automaticamente um arquivo de relatório em texto contendo:
  - localização aproximada da pessoa;
  - tempo total de execução da tarefa.

O sistema não utiliza teleoperação nem comandos manuais durante a missão.

---

## Arquitetura do sistema
O projeto é dividido em três componentes principais:

1. Simulação (externa ao repositório)
   - Gazebo / Gazebo Sim
   - Robô Clearpath Husky
   - Mundo `office`

2. Pacote ROS desenvolvido neste trabalho
   - `husky_person_follow`
   - Contém:
     - detector de pessoa (simulado);
     - máquina de estados (FSM);
     - geração do relatório final;
     - arquivos de launch.

3. Bridge Gazebo ↔ ROS
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
```

---

## Clonando o repositório
O pacote desenvolvido neste trabalho deve ser clonado dentro de um workspace ROS 2 padrão:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/emillypeix/trabalho2-husky-person-follow.git
```

---

## Compilação
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## Inicialização do robô no ambiente
O robô Husky é inicializado automaticamente por meio do launch do Clearpath Gazebo no mundo office.

A configuração padrão da simulação permite variação na pose inicial do robô, garantindo que a execução da tarefa não dependa de uma posição fixa no ambiente.

---

## Iniciando a simulação do Husky no world office
Em um terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 launch clearpath_gz simulation.launch.py world:=office
```

---

## Criando a pessoa no ambiente (posição aleatória)

Criar o modelo da pessoa (executar apenas uma vez):

```bash
cat > ~/person.sdf << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="person">
    <static>false</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.2</radius>
            <length>1.7</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.2</radius>
            <length>1.7</length>
          </cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
EOF
```

Spawn da pessoa em posição aleatória:

```bash
X=$(python3 - << 'PY'
import random
print(round(random.uniform(-3.0, 3.0), 2))
PY
)

Y=$(python3 - << 'PY'
import random
print(round(random.uniform(-3.0, 3.0), 2))
PY
)

ros2 run ros_gz_sim create \
  -world office \
  -name person \
  -file ~/person.sdf \
  -x $X -y $Y -z 0.0

echo "Pessoa criada em x=$X y=$Y"
```

---

## Bridge Gazebo para ROS (pose da pessoa)

Verificar o tópico da pose da pessoa:

```bash
gz topic -l | grep "/world/office" | grep person | grep pose
```

Normalmente o tópico será:

```
/world/office/model/person/pose
```

Rodar o bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/office/model/person/pose@geometry_msgs/msg/Pose[gz.msgs.Pose
```

---

## Executando o comportamento autônomo

Em outro terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch husky_person_follow demo_gz.launch.py side:=left
```

Parâmetros disponíveis:

- `side:=left` → robô para à esquerda da pessoa
- `side:=right` → robô para à direita da pessoa

---

## Sinalização de pessoa localizada

Quando a pessoa é encontrada, o sistema:

- imprime no log a mensagem “Pessoa localizada”;
- publica no tópico ROS:

```
/person_found
```

Pode ser monitorado com:

```bash
ros2 topic echo /person_found
```

---

## Relatório final

Ao final da missão, é gerado automaticamente o arquivo:

```
~/mission_report.txt
```

Conteúdo do relatório:

- posição aproximada da pessoa (x, y);
- tempo total de execução da tarefa.

Visualizar o relatório:

```bash
cat ~/mission_report.txt
```

---

## Observações importantes

- O repositório contém apenas o pacote desenvolvido neste trabalho.
- Gazebo, Husky e o mundo `office` são dependências externas e devem estar instaladas no sistema.
- O tópico da pose da pessoa pode ser ajustado via parâmetro de launch, caso seja diferente do padrão.

Exemplo de ajuste do tópico da pose:

```bash
ros2 launch husky_person_follow demo_gz.launch.py \
  side:=left \
  gz_pose_topic:=/TOPICO_CORRETO
```

Não é necessário editar o código para realizar essa alteração.
