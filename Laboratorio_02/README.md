# Laboratorio_02
* Sergio Avellaneda Piñeros
* David Santiago Cuellar Lopez
* Brayan Yesid Santos Gonzalez

# RoboDK - Descripción técnica y comunicación con Motoman

RoboDK es un software de simulación y programación offline/online para robots industriales. Permite crear, validar y transferir programas de robot sin depender directamente del control manual del manipulador. :contentReference[oaicite:0]{index=0}

---

## 1. Programación Offline
- Permite importar geometría CAD (por ejemplo STEP, IGES) y construir trayectorias de robot sobre esa geometría sin detener la producción real. :contentReference[oaicite:1]{index=1}  
- Se pueden definir targets, velocidades, cambios de herramienta, etc., directamente en la estación virtual y luego generar el programa del robot. :contentReference[oaicite:2]{index=2}  
- Esta programación offline (OLP, Off-Line Programming) usa un post-procesador para generar el código en el lenguaje nativo de cada control industrial. :contentReference[oaicite:3]{index=3}  

---

## 2. Simulación 3D y validación
- RoboDK simula el movimiento del robot en un entorno 3D completo (celda de trabajo, herramienta, piezas, útiles). :contentReference[oaicite:4]{index=4}  
- Permite revisar alcance, límites articulares, velocidades y posibles singularidades antes de ejecutar físicamente. :contentReference[oaicite:5]{index=5}  
- Incluye detección de colisiones entre robot, herramienta y entorno durante la simulación. :contentReference[oaicite:6]{index=6}  

Esto reduce el riesgo de choques y paradas de emergencia en planta, y evita programar “a mano” sobre el robot. :contentReference[oaicite:7]{index=7}  

---

## 3. Post-procesadores por fabricante
- RoboDK trae post-procesadores que traducen las trayectorias simuladas al lenguaje específico de cada controlador (ABB RAPID, Yaskawa/Motoman INFORM/JBI, Fanuc TP, KUKA KRL, URScript, etc.). :contentReference[oaicite:8]{index=8}  
- El post-procesador aplica las reglas sintácticas y limitaciones de cada marca (por ejemplo, formato de movimientos, declaración de herramientas, marcos de usuario). :contentReference[oaicite:9]{index=9}  
- Es posible elegir y personalizar el post-procesador por robot/celda. :contentReference[oaicite:10]{index=10}  

En el caso de Yaskawa/Motoman, RoboDK genera archivos `.JBI` en lenguaje INFORM/INFORM II-III listos para el controlador. :contentReference[oaicite:11]{index=11}  

---

## 4. Calibración y alineación (herramienta, marco y robot)
- RoboDK incluye utilidades para calibrar el TCP (Tool Center Point) de la herramienta, estimando con precisión la posición/orientación del extremo activo respecto a la brida del robot. Esto se hace tomando múltiples orientaciones del robot tocando un punto o un plano de referencia. :contentReference[oaicite:12]{index=12}  
- Permite calibrar marcos de referencia / marcos de trabajo (user frame / workobject) a partir de puntos medidos físicamente, para alinear el sistema de coordenadas real con el sistema virtual. :contentReference[oaicite:13]{index=13}  
- Ofrece módulos de calibración geométrica del robot completo (por ejemplo con láser tracker u otros sistemas de medición) para mejorar la exactitud absoluta del robot, típicamente llevando el error absoluto a décimas de milímetro dependiendo del modelo. :contentReference[oaicite:14]{index=14}  

Esta calibración mejora mucho la precisión de la programación offline, porque la celda virtual queda alineada con la celda real. :contentReference[oaicite:15]{index=15}  

---

## 5. API y Automatización
- RoboDK expone una API que se puede usar desde Python, C#, C++, MATLAB y otros lenguajes. La API controla robots, crea trayectorias, modifica frames, lee poses, ejecuta macros, etc. :contentReference[oaicite:16]{index=16}  
- La API usa un canal TCP/IP: RoboDK actúa como servidor y recibe comandos, lo que permite automatizar tareas, generar programas, o incluso mover el robot en tiempo real (online) si hay un driver conectado. :contentReference[oaicite:17]{index=17}  
- Con scripting (por ejemplo en Python) se pueden implementar bucles lógicos, condiciones, lectura de sensores/cámaras y comunicación con software externo (por ejemplo PLC o SCADA) usando sockets u otros protocolos personalizados. :contentReference[oaicite:18]{index=18}  

Esto habilita celdas robotizadas más “vivas”: inspección, reacondicionamiento de trayectorias, integración con visión artificial, etc. :contentReference[oaicite:19]{index=19}  

---

## 6. Integración CAM/CAD y aplicaciones de proceso
- RoboDK permite importar trayectorias CAM (mecanizado, pulido, corte láser, chorro de agua, impresión 3D, dispensado, soldadura, paletizado, etc.) y mapearlas a la cinemática del robot. :contentReference[oaicite:20]{index=20}  
- El software genera movimientos continuos de herramienta siguiendo superficies 3D complejas, aplicando velocidades y orientaciones definidas en el CAM. :contentReference[oaicite:21]{index=21}  

Esto convierte operaciones que normalmente haría una CNC fija en trayectorias ejecutables por un robot de múltiples ejes. :contentReference[oaicite:22]{index=22}  

---

## 7. Programación Online y drivers de robot
- Además de la programación offline tradicional (generar archivo y cargarlo al robot), RoboDK también puede controlar el robot en vivo mediante “drivers” específicos por marca. :contentReference[oaicite:23]{index=23}  
- Con un driver activo, RoboDK puede mandar movimientos en tiempo real, leer la posición actual del robot y verificar el estado durante la ejecución física (modo “Run on robot”). :contentReference[oaicite:24]{index=24}  
- Esto es útil para depurar trayectorias, ajustar puntos finos y sincronizar la celda virtual con la celda real sin tener que regenerar y cargar archivos en cada iteración. :contentReference[oaicite:25]{index=25}  

---

# 2. Comunicación entre RoboDK y manipuladores Yaskawa/Motoman

Esta sección describe cómo RoboDK se comunica específicamente con robots Motoman/Yaskawa (por ejemplo, con controladores DX100, DX200, FS100, NX100, YRC1000 / YRC1000micro).

## 2.1 Protocolos y métodos de comunicación soportados

### High-Speed Ethernet Server (HSE) / MotomanHSE
- RoboDK incluye un driver llamado `MotomanHSE` que usa el protocolo High-Speed Ethernet Server (también llamado “Remote mode” / “High Speed Ethernet Server Function”) disponible en controladores Yaskawa recientes. :contentReference[oaicite:26]{index=26}  
- Esta conexión se hace por Ethernet estándar (TCP/IP y UDP) directamente al IP del controlador. No requiere la opción MotoCom en muchos casos modernos. :contentReference[oaicite:27]{index=27}  
- Permite mover y monitorear el robot desde RoboDK con una tasa de refresco típica del orden de decenas de Hz (~70 Hz reportado). :contentReference[oaicite:28]{index=28}  
- Para habilitarlo, el robot debe estar en modo “REMOTE” en el teach pendant del controlador. :contentReference[oaicite:29]{index=29}  

### MotoCom / apimotoman
- En controladores que no soportan HSE de fábrica, es posible usar la librería MotoCom (SDK de Yaskawa) junto con el driver `apimotoman` de RoboDK para enviar comandos al controlador vía red. :contentReference[oaicite:30]{index=30}  
- MotoCom es una interfaz oficial de Yaskawa para desarrollar aplicaciones PC ↔ robot. :contentReference[oaicite:31]{index=31}  

### RS232 (controladores antiguos)
- Para generaciones más viejas (por ejemplo XRC), la conexión puede hacerse por puerto serie RS232 en lugar de Ethernet. :contentReference[oaicite:32]{index=32}  

### Transferencia de programas vía red (FTP)
- RoboDK puede enviar el programa generado (por ejemplo un `.JBI`) directamente al controlador Motoman a través de FTP, usando la IP y credenciales configuradas. :contentReference[oaicite:33]{index=33}  
- Esto automatiza la carga del archivo en el controlador sin tener que usar memoria USB o tarjetas externas. :contentReference[oaicite:34]{index=34}  

(En versiones estándar de RoboDK no se documenta Modbus TCP ni Ethernet/IP CIP como canal nativo específico para Motoman. Esos protocolos industriales pueden existir en ciertas celdas, pero no hacen parte del flujo típico de RoboDK ↔ Motoman descrito oficialmente, por lo que no se listan aquí.) :contentReference[oaicite:35]{index=35}  

---

## 2.2 Flujo típico de ejecución de movimientos

1. **Generación de trayectoria en RoboDK**  
   Se define la secuencia de movimientos (MoveJ, MoveL, velocidades, cambios de herramienta, marcos de trabajo) respetando las limitaciones cinemáticas del robot. :contentReference[oaicite:36]{index=36}  

2. **Post-procesado a código Motoman**  
   RoboDK usa el post-procesador de Motoman para convertir la trayectoria simulada en un programa `.JBI` en lenguaje INFORM/INFORM II-III, que es el formato nativo de los controladores Yaskawa/Motoman. :contentReference[oaicite:37]{index=37}  

3. **Validación previa en simulación**  
   Antes de mandar nada al robot real se revisan colisiones, límites articulares, singularidades y accesibilidad en el entorno virtual. :contentReference[oaicite:38]{index=38}  

4. **Transferencia al controlador**  
   El archivo `.JBI` se transfiere al controlador (por ejemplo DX100, DX200, FS100, NX100, YRC1000) vía red usando FTP o la conexión directa configurada desde RoboDK. :contentReference[oaicite:39]{index=39}  

5. **Ejecución física / “Run on robot”**  
   Existen dos opciones:
   - Ejecutar el programa cargado directamente desde el controlador como un job INFORM normal.  
   - Usar el driver en vivo (MotomanHSE) para que RoboDK mande los movimientos en tiempo real y supervise la posición actual, el job en ejecución y el estado del controlador. :contentReference[oaicite:40]{index=40}  

6. **Retroalimentación / sincronización**  
   Durante la ejecución online, RoboDK lee la pose articular y cartesiana real del robot y la refleja en la simulación. Esto mantiene la celda virtual alineada con la celda física y facilita el ajuste fino de trayectorias. :contentReference[oaicite:41]{index=41}  

---

## 2.3 Funciones específicas disponibles para Motoman en RoboDK

- **Driver MotomanHSE**  
  Driver dedicado para controladores Yaskawa/Motoman con función High-Speed Ethernet Server. Permite mover el robot directamente desde la estación simulada de RoboDK, sin reexportar cada vez. :contentReference[oaicite:42]{index=42}  

- **Sincronización simulación ↔ robot real**  
  Al activar “Run on robot”, los movimientos que se ven en la simulación se ejecutan en el manipulador físico, y el estado real se refleja en la interfaz de RoboDK casi en tiempo real. :contentReference[oaicite:43]{index=43}  

- **Gestión de programas y archivos**  
  RoboDK puede generar, organizar y enviar múltiples jobs `.JBI` al controlador por FTP. Esto soporta ciclos de trabajo repetibles: edición en PC → transferencia → ejecución en el robot. :contentReference[oaicite:44]{index=44}  

- **Parámetros específicos de Yaskawa (pulsos/grado)**  
  Para que el programa generado coincida exactamente con la cinemática del robot, RoboDK puede leer el archivo de parámetros del controlador (por ejemplo `ALL.PRM`) para extraer la relación pulsos/grado por eje y así generar trayectorias correctas en unidades internas del robot. :contentReference[oaicite:45]{index=45}  

---

## 2.4 Controladores soportados

El driver de alta velocidad documentado por RoboDK funciona (según disponibilidad de la opción High-Speed Ethernet Server / Remote mode en cada equipo) con las familias de controladores Yaskawa/Motoman como DX100, DX200, FS100, NX100 y YRC1000 / YRC1000micro. :contentReference[oaicite:46]{index=46}  

En controladores más antiguos (por ejemplo XRC) la integración suele requerir conexiones serie (RS232) y no siempre permite control en tiempo real desde RoboDK. :contentReference[oaicite:47]{index=47}  

---

## Referencias principales
- RoboDK Documentation (Programming, Offline Programming, Robot Drivers, Post Processors, API). :contentReference[oaicite:48]{index=48}  
- RoboDK Driver for Yaskawa/Motoman and Yaskawa controllers (DX100, DX200, FS100, NX100, YRC1000, YRC1000micro). :contentReference[oaicite:49]{index=49}  
- FTP transfer and INFORM/JBI program handling on Motoman controllers. :contentReference[oaicite:50]{index=50}  
- Calibration utilities (TCP, marcos de referencia, calibración geométrica con láser tracker). :contentReference[oaicite:51]{index=51}  
- Aplicaciones típicas (mecanizado robotizado, corte láser, pulido, soldadura, paletizado). :contentReference[oaicite:52]{index=52}  


Para un mayor detalle de lo realizado, se puede ver el video de la simulación [aquí](https://www.youtube.com/watch?v=jPIcGKx4hjY) y el de la implementación física [aquí](https://www.youtube.com/watch?v=a09duI1kMos).

