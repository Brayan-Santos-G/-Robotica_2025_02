# Laboratorio_01
* Sergio Avellaneda Piñeros
* David Santiago Cuellar Lopez
* Brayan Yesid Santos Gonzalez

## Descripción detallada de la solución planteada
La práctica se desarrolló con el objetivo de simular un proceso de decoración de torta virtual mediante el robot industrial ABB IRB 140, programado en el entorno de RobotStudio y utilizando el lenguaje de programación RAPID. El proyecto consistió en programar trayectorias que permitieran escribir los nombres de los tres integrantes del grupo: David, Brayan y Sergio, además de una figura decorativa en forma de estrella de 5 puntas. Adicionalmente, se incorporó el control de una banda transportadora, que se activa automáticamente al finalizar la decoración para trasladar la torta al siguiente proceso. Para llevar a cabo esta solución se realizaron los siguientes pasos:

### 1.	Parámetros aplicados:
* Velocidades entre 100 y 1000 mm/s.
* Precisión de aproximación en z10.
* Movimientos continuos para un trazo fluido y legible.

### 2.	Configuración inicial del sistema
Se configuró el proyecto en RobotStudio con el controlador IRC5 y el manipulador IRB 140, definiendo una posición de referencia o posición HOME a la cual el robot retorna al inicio y al final de todas las operaciones, garantizando seguridad y repetibilidad en los movimientos.
  
Se creó un Workobject con dimensiones de 20.5 cm de ancho, 17 cm de largo y 12.8 cm de alto, ubicado sobre una mesa de trabajo frente al robot. Este objeto permitió definir un sistema de coordenadas local desde el cual se programaron las trayectorias de escritura y la figura decorativa.

### 3.	Herramienta utilizada
Se diseñó una herramienta de sujeción de marcador para acoplarse al flange del robot con algunas características principales; soporte cilíndrico para alojar un marcador tipo plumón, tapa roscada que asegura el marcador en su posición, evitando desplazamientos durante los movimientos, base adaptadora con el patrón de agujeros estándar del flange del IRB 140. Cabe resaltar en este diseño se tuvo en cuenta la tolerancia de la herramienta para que no se fuera a dañar, por lo cual se le coloco un resorte interno.

En RobotStudio se definió el correspondiente Tooldata, calibrando el TCP en la punta del marcador, lo que permitió realizar trazos precisos sobre la superficie de la torta virtual.

### 4.	Programación de trayectorias
La lógica del programa se estructuró de la siguiente forma:
  1.	El robot parte desde HOME y se desplaza a un punto inicial cercano al Workobject (ABHome).
  2.	Se escribe el nombre David sobre la superficie de la torta.
  3.	El robot regresa al ABHome y escribe el nombre Brayan.
  4.	Se repite el procedimiento para el nombre Sergio.
  5.	El robot dibuja una estrella de 5 puntas como figura decorativa final.
  6.	Una vez finalizada la decoración, el robot retorna a la posición HOME.
  7.	Se activa una salida digital (DO2) que enciende la banda transportadora, trasladando la torta hasta el extremo de salida.
  8.	Se activa una salida digital (DO3) que traslada al robot a una posición de mantenimiento para poder retirar la herramienta.
  9.	Al concluir el movimiento de la banda, la señal se desactiva automáticamente, quedando el sistema listo para un nuevo ciclo.
    
Los movimientos se programaron en RAPID utilizando MoveJ, MoveL, MoveC para los trazos de los nombres y figuras, y SetDO y ResetDO para el control de la señal digital que activa la banda transportadora.

### 5.	Validación y simulación
El proceso de validación se llevó a cabo en RobotStudio con el fin de comprobar la correcta ejecución de todas las rutinas programadas. Durante la simulación se verificó que las trayectorias generadas para los nombres y la figura decorativa coincidieran con las dimensiones establecidas en el Workobject y que el trazo realizado por el TCP del marcador fuese preciso y continuo. Se confirmó también que la ubicación del Workobject y la calibración de la herramienta permitieran que el marcador se desplazara sobre la superficie sin generar interferencias ni desviaciones.

Adicionalmente, se validó la integración del control de la banda transportadora mediante una salida digital, asegurando que esta se activara automáticamente al finalizar la decoración y se desactivara al completar el traslado de la torta. Por último, se comprobó que el robot regresara de manera segura a la posición HOME al finalizar el ciclo, garantizando que no existieran colisiones con otros elementos del entorno y dejando el sistema en condiciones de reiniciar un nuevo proceso de forma ordenada y confiable.

---

## Diagrama de flujo de acciones del robot
<img width="500" height="700" alt="image" src="https://github.com/user-attachments/assets/5f34e396-ecf9-49e8-816c-abafc9d4057e" />


---

## Plano de planta de la ubicación de cada uno de los elementos

En la vista de planta se presenta la estación de trabajo utilizada para la simulación. En la parte central se encuentra el robot ABB IRB 140, encargado de realizar la operación de decoración. Frente al robot se dispone una banda transportadora, que será la utilizada durante la práctica y cuya activación se controla mediante señales digitales de entrada y salida. A un costado se ubica una segunda banda inactiva, utilizada únicamente como referencia espacial dentro del entorno de trabajo. El objeto de operación  la torta a decorar  se posiciona sobre la banda activa, dentro del área de alcance del efector final. Esta disposición garantiza una interacción eficiente entre el sistema de transporte y el robot, optimizando el espacio operativo y evitando interferencias durante el proceso automatizado.
<img width="560" height="666" alt="image" src="https://github.com/user-attachments/assets/75e88c61-64f0-4e50-8690-df761a125d19" />




---

## Descripción de las funciones utilizadas
En el archivo principal del código, donde se escribe todo el programa a ejecutarse, contamos con un bucle `WHILE TRUE DO`, el cual permite que la simulación se mantenga en ejecución continuamente, aunque físicamente no tiene relevancia en el funcionamiento real del robot.

A continuación, se utiliza una estructura condicional `IF <EXP> THEN`, que sirve para ejecutar un bloque de código cuando se detecta una entrada digital positiva, en este caso, la pulsación de un botón.

Dentro de esta condición, se emplean las funciones `SET` y `RESET` para asignar valores a las salidas digitales: `SET` establece el valor `1` (activando elementos como la banda transportadora o los LED), mientras que `RESET` asigna el valor `0` (desactivándolos). También se usa la función `WaitTime`, que permite generar un retardo durante un tiempo determinado.

Posteriormente, se llaman las diferentes trayectorias definidas por los puntos declarados más abajo en el código. En nuestro caso, decidimos crear una trayectoria por cada letra o figura, además de trayectorias específicas para enviar al robot a la posición de `Home` o al punto de mantenimiento, las cuales se ejecutan mediante los dos botones restantes.

Para la definición de trayectorias se utilizan las instrucciones `MoveJ` para movimientos articulares del robot y `MoveL` para movimientos lineales. Asimismo, se emplea `MoveC` para describir trayectorias curvas mediante dos puntos intermedios adicionales al punto final, siempre que los ángulos involucrados no superen los `180°`.

---

## Diseño de la herramienta detallado
Para la ejecución del laboratorio fue necesario diseñar una herramienta personalizada que pudiera ser acoplada al robot ABB IRB 140. En el esquema se muestra el despiece del conjunto, donde cada elemento cumple una función específica dentro del sistema.

El componente (3) corresponde al marcador, el cual se inserta en la pieza (1.2). Esta parte forma el cuerpo principal de la herramienta y fue fabricada mediante impresión 3D, junto con las piezas (1.1) y (2). La pieza (2) corresponde a una tapa roscada, diseñada para sujetar el marcador y alojar un resorte interno que proporciona tolerancia y amortiguación a la punta, permitiendo mantener un contacto constante con la superficie sin ejercer presión excesiva.

En la parte inferior del conjunto se encuentra el flanche (4), que constituye la base de acoplamiento al extremo del robot. Esta pieza también fue impresa en 3D y cuenta con orificios de fijación para tornillos M6 de media pulgada (elementos 5), asegurando una unión firme y precisa entre el flanche y el cuerpo de la herramienta.

Para un análisis más detallado de las dimensiones y configuraciones de montaje, se puede consultar la información disponible en la carpeta de [planos](https://github.com/Brayan-Santos-G/-Robotica_2025_02/blob/main/Laboratorio_01/Planos/Planos_Herramienta.pdf), donde se incluyen los dibujos técnicos y vistas complementarias del diseño.

Se uso una caja de 205*170*128 (mm). como workobject.

<img width="551" height="459" alt="image" src="https://github.com/user-attachments/assets/5f770075-69b3-4c4e-a561-6af109882d74" />




---

## Código en RAPID del módulo utilizado para el desarrollo de la práctica
Para acceder al código fuente desarrollado en RAPID, se puede consultar la información disponible en la carpeta de [códigos](https://github.com/Brayan-Santos-G/-Robotica_2025_02/blob/main/Laboratorio_01/Codigos/C%C3%B3digo%20RAPID%20LAB%201)




## Video de simulación e implementación
Para un mayor detalle de lo realizado, se puede ver el video de la implementación tanto simulada como física [aquí](https://youtu.be/l9g8OmUdVok).

