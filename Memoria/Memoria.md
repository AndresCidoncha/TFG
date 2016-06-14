#Memoria TFG   

**COMANDOS:**

* rostopic list
* rostopic echo *topic*
* rostopic pub *value* *topic*
* rosservice list
* rosservice call *command*

-----------------

##Primeras semanas

**1.** Jonay me ha dejado el [Odroid][odroid] y he estado trasteando con el por ssh. Para conectarse el user es *odroid* y la contraseña la misma. Se alimenta a 5V-2A.

**2.** Me he traido el [AmigoBot][amigobot] y he estado trasteando con el por Serial-USB. He encontrado su [guia de usuario][amigobotuserguide] y un [sitio][amigobotsoftware] para descargar software pero el dominio del amigosounds ya no existe. Hay que cruzar los cables 2-3 porque el genderchanger no viene precruzado. He instalado el [ros Aria][rosaria] en mi portatil y tras muchas peleas logro que eche a andar, me conecto con el robot y realizo varias pruebas. En el Odroid no logro instalarlo ni siguiendo el [tutorial][tutorialrosaria].

**3.** Tengo un regulador de corriente que usare para conectar el Odroid a la bateria del robot y hacerlo independiente. Se controlara por ssh usando un pincho wifi USB y editando las variables *ROS_IP* y *ROS_MASTER_URI*.

**4.** He instalado el paquete [openNI][openni] para usar la kinect, aunque al no detectarla tuve que seguir los pasos de este hilo: http://answers.ros.org/question/60562/ubuntu-12042-and-openni_launch-not-detecting-kinect-after-update/ Y he realizado pruebas usando el [tutorial][tutorialopenni]

**5.** Montaje de la Kinect y el Odroid en el AmigoBot. (Sin conexiones)

**6.** Soldadura y montaje del regulador en el robot, ademas de conexionado de la kinect y el Odroid.

**7.** Configuración del módulo wifi, de las variables de ROS y pruebas de la conexion. El portatil recibe correctamente el mapa de puntos de la Kinect. Para esto:

    export ROS_IP=<ip del dispositivo>
En odroid ademas:

    export ROS_MASTER_URI=<ip del portatil>

**8.** Debido a un formateo, he reinstalado tanto ROSARIA como LibAria. He conseguido una version de AmigoSounds, pero no logro que conecte al robot debido a que se conecta SOLO por serial, no le vale un Serial-USB, por lo que debería intentarlo en otro sistema que tenga puerto serial directo (el sobremesa tampoco tiene). El codigo para conectarse y obtener informacion del robot es simpleConnect y el de ejemplo de sonido es soundQueue. La cuestion es lograr que el robot reproduzca lo que yo quiero, que debe ser posible, pero aun no se como acceder a la salida del altavoz. He encontrado los metodos de la [Clase ArRobot][documentacionArRobot].

**9.** Mirar [tutoriales][turtlebotTutorials] turtlebot mapeo, teleoperacion, etc, depthimage_to_laserscan.

**10.** Intentando el tutorial del turtlebot me saltan errores de no tener configuradas variables de entorno como TURTLEBOT_BASE o TURTLEBOT_BATTERY. Esto cuando intento ejecutar roslaunch turtlebot_bringup minimal.launch --screen desde el odroid. Si le defino las variables como la base, me da error al no poder conectarse a una de las dos bases (kobuki o create).

    export TURTLEBOT_BASE=create
    export TURTLEBOT_STACKS=circles
    export TURTLEBOT_3D_SENSOR=kinect
    export TURTLEBOT_SERIAL_PORT=/dev/ttyUSB0


  1. Iniciamos el roscore en el portatil
  2. Definimos variables de entorno
  3. Lanzamos roslaunch turtlebot_bringup minimal.launch --screen

**11.** He encontrado [como teleoperar el pioneer3][pioneer3teleop] pero no funciona.

**12.** rosrun rqt_graph rqt_graph muestra la conexion de los nodos de ROS. Jonay me ha enseñado como teleoperar, hay dos opciones (habiendo arrancado RosAria antes):

  * rosrun turtlebot_teleop turtlebot_teleop_key /turtlebot_teleop/cmd_vel:=/RosAria/cmd_vel
  * rosrun turtlesim_teleop turtlesim_teleop_key /turtlesim_teleop/cmd_vel:=/RosAria/cmd_vel

**13.** Bueno no todo iba a ser tan positivo. Cuando parecía que ya iba a poder teleoperar, la electrónica del robot ha muerto.

![DramaticMusic](https://media.giphy.com/media/8nhgZZMKUicpi/giphy.gif)

**14.** Se ha sustituido por un conjunto de Arduino+MóduloPuenteH (me falta conectar los [encoders al arduino][encodersarduino]) y debo mirar como usar el nodo de ros [rosserial_arduino][rosserialarduino]. Me apena perder el sonido y la información de los sonares pero Jonay dice que iba a intentar arreglar esto último.

**15.** Encontrado codigo para robot diferencial con arduino [ROS Arduino Bridge][rosarduinobridge]. Instaladas librerias en la IDE. Tengo que mirarme [como publicar][arduinoserialpublish] y [como suscribirse][arduinoserialsub] a los topicos de ros.

**16.** El encoder del M2 parece tener problemas comparado con el del M1, tengo que consultarlo con Jonay, además del uso del Arduino Bridge.

**17.** El problema es de cosa de las interrupciones, usaré mi Leonardo/Mega que me deje Jonay. Deberia modificar rosserial_python node_serial.py para dejarlo a mi gusto. Los ultrasonidos se obtienen de la electronica mediante los pines y usando señales de selección:
    * 1 -> SEL1
    * 2 -> SEL2
    * 3 -> SEL3
    * 4 -> BINH
    * 5 -> INIT
    * 6,7 -> VCC
    * 8,9 -> GND
    * 10 -> ECHO

**18.** Aprovechare los leds originales del amigo bot, ya que quedan bien integrados y para que poner nuevos teniendo estos. Funcionan a 1,5V y puedo usar la faja de datos original para conectarlos. He realizado pruebas del rosserial y logro crear un topico, pero tengo que ver bien como suscribirme a uno y procesar el mensaje de entrada, ademas de mirar los distintos tipos de mensaje.

**19.** Tengo hecho un pequeño codigo de prueba con varios publishers (Encoder1,Encoder2) y varios subscribers (toggle_led,cmd_vel) para probar las capacidades del robot de cara al video de este lunes. En cuanto esté pulido le haré el video.

**20.** Volvemos al ruedo, utilizando de base el sketch de ArduinoBridge, puedo obtener la potencia para cada motor. Solo he de integrar los encoders propios y cambiar el codigo para utilizar el puente H.

**21.** Conexiones del modulo de leds + switch:
            |----|
|D2-/D3-|D1+|SW1o|---------|---|
|D3+|SW2o|D2+|D1-/SW2i/SW1i|---|

Uso SW1 como reinicio del Arduino.


[pioneer3teleop]:http://answers.ros.org/question/92151/how-to-teleop-pioneer3/
[odroid]: http://www.hardkernel.com/main/products/prdt_info.php?g_code=G138745696275
[amigobot]: http://www.mobilerobots.com/ResearchRobots/AmigoBot.aspx
[amigobotuserguide]: http://robots.mobilerobots.com/amigobot/amigofree/AmigoGuide.pdf
[amigobotsoftware]: http://robots.mobilerobots.com/amigobot/originalAmigos.html
[rosaria]: http://wiki.ros.org/ROSARIA
[tutorialrosaria]: http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA
[openni]: http://wiki.ros.org/openni_camera
[tutorialopenni]: http://wiki.ros.org/openni_launch/Tutorials/QuickStart
[documentacionArRobot]: http://www.eecs.yorku.ca/course_archive/2009-10/W/4421/doc/pioneer/aria/classArRobot.html
[turtlebotTutorials]: http://wiki.ros.org/turtlebot/Tutorials
[encodersarduino]: http://playground.arduino.cc/Main/RotaryEncoders
[rosserialarduino]: http://wiki.ros.org/rosserial_arduino
[rosarduinobridge]: https://github.com/hbrobotics/ros_arduino_bridge/tree/indigo-devel
[arduinoserialpublish]: http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
[arduinoserialsub]: http://wiki.ros.org/rosserial_arduino/Tutorials/Blink
