<a id="top"></a>

Clique na imagem abaixo para assistir o vídeo da demonstração.

<p align="center">
<a href="https://youtu.be/aJ39MruDdLo" target="_blank">
<img src="https://user-images.githubusercontent.com/28100951/121440615-98a52280-c95e-11eb-9177-1bf9fa9ab69c.png" width="500">
</p>
</a>

------------
## Sumário

1. [Mantenedores](#1.0)
2. [Compatibilidade](#2.0)
3. [Instalação](#3.0)
4. [Como usar o pacote](#4.0) 
5. [Informações sobre pacote](#4.0) 


---
<a name="1.0"></a>
## 1. Mantenedores
 [Voltar ao topo da página](#top)

Pesquisadores:
- M.Sc. Caio Viturino* - [[Lattes](http://lattes.cnpq.br/4355017524299952)] [[Linkedin](https://www.linkedin.com/in/engcaiobarros/)] - engcaiobarros@gmail.com
- M.Sc. Daniel M. de Oliveira* - [[Linkedin](https://www.linkedin.com/in/daniel-moura-de-oliveira-9b6754120/)] - danielmoura@ufba.br 

Supervisor:
- Prof. Dr. André Gustavo Scolari Conceição* - [[Lattes](http://lattes.cnpq.br/6840685961007897)] - andre.gustavo@ufba.br

*LaR - Laboratório de Robótica, Departamento de Engenharia Elétrica e de Computação, Universidade Federal da Bahia, Salvador, Brasil

---
<a name="2.0"></a>
## 2. Compatibilidade
[Voltar ao topo da página](#top)

**Sistema:** Ubuntu 16.04 or Ubuntu 18.04

**Versão do ROS:** Kinetic ou Melodic.

---
<a name="3.0"></a>
## 3. Instalação
[Voltar ao topo da página](#top)

- Instale o ROS disponível em 

  http://wiki.ros.org/ROS/Installation

- Instale o(s) pacote(s):

  ```shell
  sudo apt-get install ros-kinetic-trac-ik
  ```

- Siga o procedimento de instalação presente em:

  https://cyberbotics.com/doc/guide/installing-webots

Depois de instalar o webot e o ROS para o seu sistema operacional, siga os passos adiante:

- Clone este pacote para a sua pasta src do seu espaço de trabalho. 
  Ex: `/path/to/catkin_ws/src`

- Instale as dependências, trocando o $ROS_DISTRO pela versão do seu ROS e realize o build do espaço de trabalho
     ```shell
     rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
     catkin build
     ```

---
<a name="4.0"></a>
## 4. Como usar o pacote
[Voltar ao topo da página](#top)

- É extremamente importante que você siga os tutoriais do Webots para familiarizar-se com o software. Link:

  https://cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots

- No software webots, abra o mundo presente na pasta 

  `worlds/ur5_robotiq_gripper2f140.wbt`

- Antes de iniciar a simulação no webots, verifique se todos os controladores estão no modo `<extern>`. Aperte o play na simulação e faça o launch do seguinte arquivo:

     ```
     roslaunch ur5_webots_lar ur5e_joint_limited.launch
     ```
     Após realizar o launch do ur5e_joint_limited.launch, a simulação irá iniciar automaticamente (lembre de deixar o play pressionado). Sempre que finalizar esse launch, você deve reiniciar a simulação no webots.

- Rodar a simulação no RVIZ pode ser vantajoso para visualizar a nuvem de pontos, eixos de coordenadas, etc. Para abrir o rviz, realize o launch do seguinte arquivo:
     ```
     roslaunch ur5_webots_lar view_ur5e.launch
     ```

- Para controlar o robô, rode o seguinte *node*:
     ```
     rosrun ur5_webots_lar test_controller.py
     ```
  Este node utiliza um planejamento de trajetória conhecido como polinômios quínticos. Além disso, foi implementada a cinemática inversa utilizando o pacote [Trac IK](http://wiki.ros.org/trac_ik).

---
<a name="5.0"></a>
## 5. Informações sobre o pacote
[Voltar ao topo da página](#top)

Existem dois controladores que fazem conexão com o Webots para obter as imagens de cor e profundidade e controlar o efetuador final e o robô. Estes controladores estão nos arquivos:
- webots_camera_controller.py
- webots_ur5_controller.py

Estes controladores utilizam os ActionServers presentes nos arquivos seguintes para enviar comandos ao robô e ao efetuador final no Webot:
- trajectory_follower_gripper.py
- trajectory_follower.py

Para criar uma nuvem de pontos que possa ser visualizada no RVIZ, foi implementado o nodelet [depth_image_proc/point_cloud_xyz](depth_image_proc/point_cloud_xyz) que utiliza os parâmetros intrínsecos da câmera para gerar a nuvem de pontos. Este nodelet está implementado no launch seguinte:
- depth_to_pc.launch (inicializado automaticamente com o ur5e_joint_limited.launch)

Os parâmetros intrínsecos da câmera estão localizados na pasta `config`, nos arquivos:
- color_camera_info.yaml
- depth_camera_info.yaml
