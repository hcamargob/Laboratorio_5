# Laboratorio 5 - Cinemática Inversa - Robot Phantom X - ROS
<p align="center">
ROBÓTICA

<p align="center">
Hugo Alejandro Camargo Barrera
<p align="center">
email: hcamargob@unal.edu.co

<p align="center">
Santiago Hernández Lamprea
<p align="center">
email: shernandezl@unal.edu.co


<p align="center">
INGENIERÍA MECATRÓNICA
<p align="center">
Facultad de Ingeniería
<p align="center">
Universidad Nacional de Colombia Sede Bogotá

Para cumplir satisfactoriamente los requerimientos y tareas propuestas, se siguió el siguiente proceso:

  ### 1. Cinemática Inversa
  
  Teniendo en cuenta la cinemática directa del robot Phantom, se realizó la cinemática inversa de este por medio del método geométrico. Se supuso la última articulación como muñeca, siempre horizontal para simplificar el problema, de esta forma no se necesita pedir una orientación específica, sino que siempre sea igual, para que solo sea necesario dar las coordenadas en (x,y,z). De esta forma, la articulación 1 se puede hallar solo con las coordenadas (x,y)  y las articulaciones 2 y 3 se hallan con cinemática inversa de un robot 2GDL, la articulación 4 depende de las anteriores para quedar horizontal y la articulación 5 trabaja como una herramienta y no tiene relación alguna con las demás, solo se varía cuando se quiere agarrar o soltar el marcador. Las ecuaciones están descritas así:

![cinematica_inv](https://user-images.githubusercontent.com/42346345/200101798-f9666ec0-92f0-4d7d-8d84-bad10b423dad.png)


  ### 2. Identificación del espacio de trabajo
Se realizó la identificacion del espacio de trabajo con Dinamixel, observando la mayor elongación del brazo y realizando el movimiento de la primera articulación, despues se realiza lo mismo con la elongación mínima. Así pues, se obtiene el espacio alcanzable por el Robot Phantom X.
  
  Este fue el espacio de trabajo:
  
  ![image](https://user-images.githubusercontent.com/112737454/200098852-cb014428-6780-4895-9ee4-5217d8b0eaf0.png)
  
  [![image](https://user-images.githubusercontent.com/112737454/200102225-f98399ba-9005-421c-9a72-0596df9db4bd.png)](https://www.youtube.com/watch?v=aqmeTjIOSUI&list=PLZO_90ooNtkvjKJZ4ga3Eclyv0Wju51Rj&index=1)

  
Se parametrizó en Desmos, y este software permitió la ubicación de las trayectorias en el espacio de trabajo, los calculos se hicieron respecto a lo obtenido y dispuesto en Desmos.
  
   ![image](https://user-images.githubusercontent.com/112737454/200098953-a57eb504-1ea2-4062-883c-fee8a54d1af6.png)


### 3. Construcción de Trayectorias 
  Para cada trayectoria se realizó una parametrización en MatLab, las cuales fueron:
  1) Letras
  Se calcularon las trayectorias de las letras S y H a través de rectas y arcos, de la siguiente manera:
  ```
  %Letra H
  x1=-189:8:-155;
  x2=-158:8:-124;
  x3=-172:8:-139;

  m1=(156-197)/(-155+189);
  b1=197-m1*(-189);
  y1=m1.*x1+b1;
  b2=226-m1*(-158);
  y2=m1.*x2+b2;
  m2=-1/m1;
  b3=176-m2*(-172);
  y3=m2.*x3+b3;

  XH=[-189 x1 -157 -172 x3 -140 -158 x2 -126];
  YH=[197 y1 158 176 y3 203 226 y2 187];
  ZH=[30 -10*ones(1,length(x1)) 10 10 -10*ones(1,length(x3)) 10 10 -10*ones(1,length(x2)) 30];
  PH=[XH' YH' ZH'];
  ```
  
  ```
  %Letra S 
  ts1=3*pi/4:0.8:7*pi/4;
  ts2=-pi/4:0.8:3*pi/4;
  r1=sqrt((-214+236)^2+(131-152)^2)/2;

  xs1=-236:5:(r1*sin(pi/4)-236);
  xs1in=(r1*sin(pi/4)-236*2)-xs1-1.46+sin(pi/4);

  ys1=152:5:(r1*sin(pi/4)+152);
  ys1in=(r1*sin(pi/4)+2*152)-ys1-1.46+sin(pi/4);

  xs2=r1*cos(ts1)-(214+236)/2;
  ys2=r1*sin(ts1)+(131+152)/2;
  xs3=r1*cos(pi/2-ts2)-(214+236)/2+(236-214);
  ys3=r1*sin(pi/2-ts2)+(131+152)/2-(-131+152);

  xs4=-191.8093-r1*sin(pi/4):5:-191.8093;
  xs4in=-2*191.8093-xs4-r1*sin(pi/4)-1.46+sin(pi/4);

  ys4=110.2035-r1*sin(pi/4):5:110.2035;
  ys4in=2*110.2035-ys4-r1*sin(pi/4)-1.46+sin(pi/4);

  XS=[-226 xs1in xs2 xs3 xs4in -202.5622];
  YS=[162 ys1in ys2 ys3 ys4in 99.4506];
  ZS=[30 -10*ones(1,length(XS)-2) 30];
  PS=[XS' YS' ZS'];
  ```
  Se combinaron en una sola rutina y asi se obtuvo el vector de puntos de las trayectorias.
  ```
  XLet=[XS XH -250];
  YLet=[YS YH 0];
  ZLet=[ZS ZH 30];
  PLet=[XLet' YLet' ZLet'];
  ```
Este es el resultado esperado
  
  ![image](https://user-images.githubusercontent.com/112737454/200099349-dedbee58-878f-4336-9b99-a76551194447.png)

 2) Figuras
  Parametrizando rectas y curvas, entendiendo el orden de los puntos, se obtuvo:
  ```
  %Triángulo

  xt1=-276:4:-256;
  xt2=-256:4:-236;
  xt3=-276:4:-236;

  xt3in=-236-xt3-276;
  yt1=tan(pi/3)*(xt1+276)+28;
  yt2=-tan(pi/3)*(xt2+236)+28;
  yt3=ones(1,length(xt3))*28;

  XT=[-276 xt1 xt2 xt3in -276];
  YT=[28 yt1 yt2 yt3 28];
  ZT=[30 -10*ones(1,length(XT)-2) 30];
  PT=[XT' YT' ZT'];
  ```
  ```
  %Círculo

  tc=0:0.2:2*pi;
  XC=[-210 20*cos(tc)-230 -210];
  YC=[0 20*sin(tc) 0];
  ZC=[30 -10*ones(1,length(XC)-2) 30];
  PC=[XC' YC' ZC'];
  ```
  ```
  %Paralelas

  xp1=-260:10:-230;
  yp1=tan(pi/6)*(xp1+260)-47;
  yp2=tan(pi/6)*(xp1+260)-67;
  yp3=tan(pi/6)*(xp1+260)-87;

  XP=[-260 xp1 -230 -260 xp1 -230 -260 xp1 -230];
  YP=[yp1(1) yp1 yp1(length(yp1)) yp2(1) yp2 yp2(length(yp3)) yp3(1) yp3 yp3(length(yp3))];
  ZP=[30 -10*ones(1,length(xp1)) 30 30 -10*ones(1,length(xp1)) 30 30 -10*ones(1,length(xp1)) 30];

  PP=[XP' YP' ZP'];
  ```
  Se unieron todas las figuras en una sola trayectoria, se obtuvo:
  ```
  XFIG=[XT XC XP -250];
  YFIG=[YT YC YP 0];
  ZFIG=[ZT ZC ZP 30];

  PFIG=[XFIG' YFIG' ZFIG'];
  ```
  Este es el resultado esperado:
  
  ![image](https://user-images.githubusercontent.com/112737454/200099746-12f32a59-f3e7-4618-95e8-f641a49e5510.png)

  3) Trayectoria propia
  Se eligió realizar una estrella suavizada de 5 puntas. Esta se parametrizó a través de funciones polares:
  ```
  %Trebol

  ttr=0:0.2:2*pi;
  XTR=[-136 cos(ttr).*7.*(cos(5*ttr)+5)-178 -136 ];
  YTR=[-171 sin(ttr).*7.*(cos(5*ttr)+5)-171 -171 ];
  ZTR=[30 -11*ones(1,length(ttr)) 30 ];

  PTR=[XTR' YTR' ZTR'];
  ```
  Se espera el siguiente resultado:
  
  ![image](https://user-images.githubusercontent.com/112737454/200100112-461ddff3-e927-474d-8fcd-8dd159c899cc.png)

  4) Puntos Equidistantes
  
  Para estos puntos, se siguió una estructura pentagonal y se obtuvo
  
  ```
  %Puntos

  tp=0:2*pi/5:2*pi;
  xpui=-230+30*cos(tp);
  ypui=-110+30*sin(tp);

  XPui=[xpui(1) xpui(1) xpui(1) xpui(2) xpui(2) xpui(2) xpui(3) xpui(3) xpui(3) xpui(4) xpui(4) xpui(4) xpui(5) xpui(5) xpui(5) -250];
  YPui=[ypui(1) ypui(1) ypui(1) ypui(2) ypui(2) ypui(2) ypui(3) ypui(3) ypui(3) ypui(4) ypui(4) ypui(4) ypui(5) ypui(5) ypui(5) 0];
  ZPui=[30 -10 30 30 -10 30 30 -10 30 30 -10 30 30 -10 30 30];

  PPui=[XPui' YPui' ZPui'];
  ```
  Se espera:
  
  ![image](https://user-images.githubusercontent.com/112737454/200100356-7054c005-6884-4cd4-8034-6050dcb91494.png)

### 4. Python
  
  Se creó un código en Python para que el robot realice las rutinas creadas previamente por medio del ingreso de comandos por teclado, solo se requiere tener la cinemática inversa realizada para cada trayectoria y poner la secuencia de ángulos de las articulaciones en el programa. Dependiendo de la tecla ingresada el robot realizará una rutina diferente, el código queda así:
```
def joint_publisher():
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)
    
    while not rospy.is_shutdown():
        key='0'
        key=input()
        if key=='1':
            hacer_trayectoria(home,pub)
        elif key=='2':
            hacer_trayectoria(tomar_marcador,pub)
        elif key=='3':
            hacer_trayectoria(espacio_trabajo,pub)
        elif key=='4':
            hacer_trayectoria(letras,pub)
        elif key=='5':
            hacer_trayectoria(figuras,pub)
        elif key=='6':
            hacer_trayectoria(puntos_eq,pub)
        elif key=='7':
            hacer_trayectoria(dibujo,pub)
        elif key=='8':
            hacer_trayectoria(dejar_marcador,pub)
```
Aquí se usan las funciones hacer_trayectoria() e ir_a_punto() que están definidas así:
```
def ir_a_punto(q, pub):	
    state = JointTrajectory()
    state.header.stamp = rospy.Time.now()
    state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
    point = JointTrajectoryPoint()
    point.positions = [q[0], q[1], q[2], q[3], q[4]]    
    point.time_from_start = rospy.Duration(0.05)
    state.points.append(point)
    pub.publish(state)
    #print('published command')
    rospy.sleep(0.7)
    
def hacer_trayectoria(puntos,pub):
    for i in puntos:
        ir_a_punto(i,pub)
```
Para que el robot realice las rutinas se debe conectar al puerto USB, se debe abrir un terminal y escribir el comando:
```
roscore
```
En otro terminal se debe ir a la ubicación del catkin workspace e ingresar los comandos:
```
source devel/setup.bash
roslaunch dynamixel_one_motor one_controller.launch
```
Por último, en otro terminal se debe ir a la carpeta de scripts del proyecto y correr el archivo del código creado:
```
python3 jointPub.py
```
Luego de esto el programa está listo para recibir instrucciones.
### 5. Resultados
  Finalmente, observamos los resultados
  
  1) Letras
  
  ![image](https://user-images.githubusercontent.com/112737454/200101064-0efab978-44b3-40bf-a37e-bc562f99ed91.png)
![image](https://user-images.githubusercontent.com/112737454/200101160-92d7897b-fad5-45b0-8894-0138ab7350d2.png)

[![image](https://user-images.githubusercontent.com/112737454/200101276-01a4a3f4-aaa3-4d7d-b096-9365fac396f3.png)](https://www.youtube.com/watch?v=DCI5xsrFhLw&list=PLZO_90ooNtkvjKJZ4ga3Eclyv0Wju51Rj&index=2)
  
  2) Figuras
  
  ![image](https://user-images.githubusercontent.com/112737454/200101400-0989fb15-a0b9-4ebd-bf00-dbfe82abc1b8.png)
  ![image](https://user-images.githubusercontent.com/112737454/200101436-8e2423fe-1e8e-4756-8cef-fabda1a6dae1.png)

  [![image](https://user-images.githubusercontent.com/112737454/200101487-7f2f0b3d-8692-49ba-be62-8d0d7b6553a7.png)](https://www.youtube.com/watch?v=_FyVG7ztp2E&list=PLZO_90ooNtkvjKJZ4ga3Eclyv0Wju51Rj&index=3)

  3) Trébol
  
  ![image](https://user-images.githubusercontent.com/112737454/200101946-7e9d5d88-9fbc-4b4f-9134-646f6e8f283c.png)
![image](https://user-images.githubusercontent.com/112737454/200101976-35968ce1-7a20-444a-b92d-ea99055a9f90.png)

 [![image](https://user-images.githubusercontent.com/112737454/200102001-9db976d2-9ebf-442d-bc63-5db6c183cd36.png)](https://www.youtube.com/watch?v=-36fzap_axY&list=PLZO_90ooNtkvjKJZ4ga3Eclyv0Wju51Rj&index=4)
  
  4) Puntos Equidistantes
  
  ![image](https://user-images.githubusercontent.com/112737454/200102101-5202b3a8-ef09-48ac-8686-ae3b986e3a3b.png)
  ![image](https://user-images.githubusercontent.com/112737454/200102134-a1ee591d-b43c-4b24-b0f0-ef838be0b91b.png)
  
  [![image](https://user-images.githubusercontent.com/112737454/200102165-fc58a3ba-2547-4045-bba1-ca9f0f1bf7e7.png)](https://www.youtube.com/watch?v=T0nJiAJR6Ag&list=PLZO_90ooNtkvjKJZ4ga3Eclyv0Wju51Rj&index=5)

 5) Comparación de trayectorias
  
  ![image](https://user-images.githubusercontent.com/112737454/200155074-3b43c07f-d453-4802-9c38-3743463b6ee7.png)




