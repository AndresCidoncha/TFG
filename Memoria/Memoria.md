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

**4.** He instalado el paquete [openNI][openni] para usar la kinect, aunque al no detectarla tuve que seguir los pasos de [este hilo][KinectNotDetected] http://answers.ros.org/question/60562/ubuntu-12042-and-openni_launch-not-detecting-kinect-after-update/ Y he realizado pruebas usando el [tutorial][tutorialopenni]

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

<center>![DramaticMusic](https://media.giphy.com/media/8nhgZZMKUicpi/giphy.gif)

**14.** Se ha sustituido por un conjunto de Arduino+MóduloPuenteH (me falta conectar los [encoders al arduino][encodersarduino]) y debo mirar como usar el nodo de ros [rosserial_arduino][rosserialarduino]. Me apena perder el sonido y la información de los sonares pero Jonay dice que iba a intentar arreglar esto último.

**15.** Encontrado codigo para robot diferencial con arduino [ROS Arduino Bridge][rosarduinobridge]. Instaladas librerias en la IDE. Tengo que mirarme [como publicar][arduinoserialpublish] y [como suscribirse][arduinoserialsub] a los topicos de ros.

**16.** El encoder del M2 parece tener problemas comparado con el del M1, tengo que consultarlo con Jonay, además del uso del Arduino Bridge.

**17.** El problema es de cosa de las interrupciones, usaré mi Leonardo/Mega que me deje Jonay. Deberia modificar rosserial_python node_serial.py para dejarlo a mi gusto. Los ultrasonidos se obtienen de la electronica mediante los pines y usando señales de selección:

    PIN -> FUNC -> ARDUINO
    ----------------------
      1   -> SEL1 -> 40
      2   -> SEL2 -> 41
      3   -> SEL3 -> 42
      4   -> BINH -> 44
      5   -> INIT -> 45
      6,7 -> VCC  -> 5V
      8,9 -> GND  -> GND
      10  -> ECHO -> 43

**18.** Aprovechare los leds originales del amigo bot, ya que quedan bien integrados y para que poner nuevos teniendo estos. Funcionan a 1,5V y puedo usar la faja de datos original para conectarlos. He realizado pruebas del rosserial y logro crear un topico, pero tengo que ver bien como suscribirme a uno y procesar el mensaje de entrada, ademas de mirar los distintos tipos de mensaje.

**19.** Tengo hecho un pequeño codigo de prueba con varios publishers (Encoder1,Encoder2) y varios subscribers (toggle_led,cmd_vel) para probar las capacidades del robot de cara al video de este lunes. En cuanto esté pulido le haré el video.

**20.** Volvemos al ruedo, utilizando de base el sketch de ArduinoBridge, puedo obtener la potencia para cada motor. Solo he de integrar los encoders propios y cambiar el codigo para utilizar el puente H.

**21.** Conexiones del modulo de leds + switch:

    PIN ->   FUNCTION   -> ARDUINO
    ------------------------------
      1  -> D2-/D3-       -> GND
      2  -> D1+           -> 12
      3  -> SW1o          -> RST
      4  -> ?
      5  -> ?
      6  -> D3+           -> 11
      7  -> SW2o          -> 10
      8  -> D2+           -> 13
      9  -> D1-/SW2i/SW1i -> GND
      10 -> ?

Uso SW1 como reinicio del Arduino.

**22.** He estado trasteando, no logro que me devuelva info los sonares. Me faltan librerias y demas cosas para poder usar el arduino_diff_drive. Tendré que acudir a Jonay para despejar ese par de dudas. Mientras, he encontrado un enlace de un tipo que dicen que es bastante bueno, trata sobre [SLAM 2D con ROS y Kinect][2dSlamKinect]. Me quedan 6 pines libres (4,5,12,A3,A4,A5), si al final llegara a usar los sonares, los usaria todos, por lo que en caso de necesitar algo adicional, deberia tirar de un MEGA.

**23.** Bueno ya esta el codigo, solo me queda arreglar las librerias. Tengo que pasar los motores a una clase, quedara mucho mas limpio. Aparte, los sonares tengo que dejar INIT encendido hasta recoger el ECHO, por lo que no me vale la libreria.

**24.** Codigo listo, lo que he hecho ha sido quitar el OdomLite y tirar directamente del paquete de odometria de ROS. Basicamente lo que he hecho es lo que hace el con el codigo [OdomliteToOdom.cpp](https://github.com/SimonBirrell/ros_arduino_diff_drive/blob/master/src/odomlite_to_odom.cpp). He creado una clase para el conjunto motor+encoder (*MotorEncoder*) y otra para la electronica en general (*AmigoBot*). He intentado limpiar y simplificar un poco,
tengo pendiente comprobar que rule, que esa es otra, ahora el IDE me decia que no habia
memoria suficiente en el Arduino...

**25.** Bueno, el codigo del Arduino Diff Drive no cabe en el Leonardo, pero si en el MEGA, Jonay dice que me conseguirá uno. Yo por mi parte, he adaptado el del Arduino Bridge, pero no logro que funcione, creo que porque es kinetic y no indigo, buscaré la manera de ajustarlo, aunque tenga que formatear de nuevo el portatil... (que pereza).

**26.** A por el MEGA! Jonay me ha dado uno modificado que me viene bastante bien, ahora intentare dejar el circuito lo mas bonito posible. He modificado un poco el orden de los pines, debo revisar el codigo. Ademas, he empezado a modificar la electronica original para usarla de base para alimentar los elementos nuevos de la electronica, lo que lo deja mucho mas limpio (y ya que voy a dejar el cacharro ese ahi, pues aprovecharlo para algo...). Por lo pronto, ya tengo los encoders preparados y su "puerto" a 5V. Tengo otro que me permite conectar otros 2 dispositivos a 5 V (odroid y arduino?). Me faltaria por conectar la Kinect (12V) y el modulo de L298N (12V).

**27.** LISTO!! La electronica esta montada totaaaaaalmente (he intentado que quede lo mas limpio posible). He intentado que funcionen los sonares pero no lo he conseguido, tengo que documentarme mas porque con lo que viene en el [datasheet][TL851-datasheet] no logro que tire. De resto, me queda formatear el portatil para poder poner indigo de nuevo, probar que todo funciona correctamente y unir nodos. Y escribir memoria como si no hubiera mañana.

**28.** Kinect funcionando de nuevo (habia que actualizar los paquetes). Pruebas realizadas, modo distribuido funcionando. He encontrado un tutorial pequeño de unir SLAM y kinect [aqui][ArdRos].

**29.** Para usar SLAM necesito convertir el PointCloud a laser, usando [este nodo][PointCloudToLaserScan]. Portatil de mierda que me jode la particion, tengo que formatearlo. El rosserial_arduino se compueba asi:

    rosrun rosserial_python serial_node.py /dev/ttyUSB0

Me da error a la hora de publicar en /odom parece, pero los topics los crea bien. Tendre que revisar eso. Para la transformacion de PointCloud a laser_scan, hay que ejecutar:

    rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node



[pioneer3teleop]:http://answers.ros.org/question/92151/how-to-teleop-pioneer3/
[odroid]: http://www.hardkernel.com/main/products/prdt_info.php?g_code=G138745696275
[amigobot]: http://www.mobilerobots.com/ResearchRobots/AmigoBot.aspx
[amigobotuserguide]: http://robots.mobilerobots.com/amigobot/amigofree/AmigoGuide.pdf
[amigobotsoftware]: http://robots.mobilerobots.com/amigobot/originalAmigos.html
[rosaria]: http://wiki.ros.org/ROSARIA
[tutorialrosaria]: http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA
[openni]: http://wiki.ros.org/openni_camera
[KinectNotDetected]: http://answers.ros.org/question/60562/ubuntu-12042-and-openni_launch-not-detecting-kinect-after-update/
[tutorialopenni]: http://wiki.ros.org/openni_launch/Tutorials/QuickStart
[documentacionArRobot]: http://www.eecs.yorku.ca/course_archive/2009-10/W/4421/doc/pioneer/aria/classArRobot.html
[turtlebotTutorials]: http://wiki.ros.org/turtlebot/Tutorials
[moduloPuenteH]: http://electronilab.co/tutoriales/tutorial-de-uso-driver-dual-l298n-para-motores-dc-y-paso-a-paso-con-arduino/
[encodersarduino]: http://playground.arduino.cc/Main/RotaryEncoders
[rosserialarduino]: http://wiki.ros.org/rosserial_arduino
[rosarduinobridge]: https://github.com/hbrobotics/ros_arduino_bridge/tree/indigo-devel
[arduinoserialpublish]: http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
[arduinoserialsub]: http://wiki.ros.org/rosserial_arduino/Tutorials/Blink
[2dSlamKinect]: http://www.hessmer.org/blog/2011/04/10/2d-slam-with-ros-and-kinect/
[TL851-datasheet]: http://www.ti.com/lit/ds/symlink/tl851.pdf
[ArdRos]: http://www.hessmer.org/blog/2011/04/10/2d-slam-with-ros-and-kinect/
[PointCloudToLaserScan]: http://wiki.ros.org/pointcloud_to_laserscan
