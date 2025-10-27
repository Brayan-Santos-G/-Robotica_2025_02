# Laboratorio_02
* Sergio Avellaneda Piñeros
* David Santiago Cuellar Lopez
* Brayan Yesid Santos Gonzalez

# Descripción de las principales funcionalidades de RoboDK

RoboDK es un software de simulación y programación offline/online para robots industriales. Permite crear, validar y transferir trayectorias de robot sin depender exclusivamente de la enseñanza punto a punto en el controlador físico.

## Programación offline y simulación 3D
- Permite crear programas de robot de manera offline (Off-Line Programming), es decir, sin detener la producción real.
- Admite la importación de geometría CAD en formatos industriales comunes como STEP e IGES, para definir piezas, herramientas, útiles y estaciones de trabajo.
- El entorno 3D permite ubicar el robot, la herramienta (TCP), las referencias de trabajo y los objetos de la celda como un gemelo digital básico.
- Se pueden predefinir targets cartesianos y articulares, velocidades, cambios de herramienta y marcos de usuario antes de mandarlos al robot físico.

## Detección de colisiones y validación de alcance
- El simulador incorpora chequeo de colisiones entre robot, herramienta y entorno. Si la detección está activa, la simulación puede detener trayectorias que entrarían en contacto.
- También se validan zonas de singularidad, límites articulares y accesibilidad de la herramienta a la pieza antes de liberar un programa.
- Esta verificación previa reduce choques en planta y evita tener que probar trayectorias peligrosas directamente sobre el manipulador.

## Post-procesadores y generación de código
- RoboDK incluye post-procesadores específicos por marca. Estos post-procesadores traducen las trayectorias simuladas al lenguaje nativo de cada fabricante (por ejemplo: RAPID para ABB, INFORM/JBI para Yaskawa Motoman, TP para Fanuc, KRL para KUKA, URScript para UR, etc.).
- En el caso de Yaskawa/Motoman, el post-procesador genera archivos `.JBI` en lenguaje INFORM/INFORM II-III listos para cargarse en el controlador.
- El post-procesador aplica las reglas sintácticas, nombres de herramientas, marcos de usuario y formato de movimientos que requiere cada controlador industrial.

## API y automatización
- RoboDK expone una API que se puede usar desde Python, C#, C++, MATLAB y otros lenguajes.
- Mediante la API se pueden crear trayectorias, modificar marcos de referencia, leer la pose actual del robot, automatizar rutinas repetitivas y generar programas desde scripts.
- La misma API también permite controlar robots en vivo (“Run on Robot”) cuando hay un driver conectado: el mismo código que se ejecuta en simulación puede mandar movimientos al robot físico en tiempo real.
- Esto habilita integración con visión, PLCs y lógica de célula sin necesidad de programar todo desde el teach pendant.

## Calibración y alineación físico-virtual
- El software incluye herramientas para definir y calibrar el TCP (Tool Center Point) de la herramienta instalada en la brida del robot.
- La calibración del TCP se puede hacer tocando un punto fijo desde varias orientaciones o tocando un plano de referencia; con suficientes muestras (típicamente ≥8 configuraciones) se obtiene la posición precisa del extremo de trabajo.
- También se pueden alinear y calibrar marcos de trabajo (workobjects / user frames) para que las coordenadas del mundo físico coincidan con las del modelo virtual.
- RoboDK ofrece módulos avanzados de calibración geométrica (por ejemplo con láser tracker u otros sistemas de medición) para mejorar la exactitud absoluta del robot y reducir el desfase entre la celda simulada y la celda real.

## Integración CAD/CAM y trayectorias de proceso
- RoboDK puede importar trayectorias CAM (fresado, corte láser, pulido, dispensado, desbaste, soldadura, etc.) y proyectarlas sobre la cinemática del robot.
- A partir de superficies 3D complejas se generan movimientos continuos de herramienta con orientación, avance y velocidad definidas para el proceso.
- Esto permite usar robots industriales para operaciones típicamente asociadas a máquinas CNC o a celdas de proceso especializadas.

## Programación online (“Run on Robot”)
- Además de generar el archivo para cargarlo en el controlador, RoboDK puede conectarse al robot y enviarle movimientos en vivo mediante drivers específicos por marca.
- En este modo el robot real sigue la trayectoria que se ve en la simulación, y la posición/articulación real del robot se refleja de vuelta en RoboDK. Esto sirve para depurar puntos finos, corregir offsets y ajustar trayectorias sin regenerar el programa completo cada vez.


# Comunicación con el Manipulador Motoman

Esta sección resume cómo RoboDK se integra específicamente con robots Yaskawa/Motoman (controladores DX100, DX200, FS100, NX100, YRC1000, YRC1000micro, etc.).

## Métodos de comunicación soportados
- Driver MotomanHSE (High-Speed Ethernet Server / “Remote”): RoboDK incluye un driver dedicado (MotomanHSE) que usa el protocolo Ethernet de alta velocidad soportado por controladores Motoman/Yaskawa recientes. No requiere la opción MotoCom en muchos casos modernos.
- Conexión Ethernet estándar (TCP/UDP): Se configura la IP del controlador y se pone el robot en modo REMOTE desde el teach pendant. Una vez conectado, RoboDK puede mover y monitorear el robot con una frecuencia típica de actualización en el orden de decenas de Hz (~70 Hz reportado).
- RS232 en controladores antiguos: En controladores que no soportan High-Speed Ethernet Server (por ejemplo familias XRC más viejas), la conexión puede hacerse por puerto serie especificando un puerto COM.
- MotoCom / apimotoman: Para ciertos controladores, es posible usar MotoCom (SDK oficial de Yaskawa) junto con el driver apimotoman, lo que permite control desde un PC cuando HSE no está disponible.

## Flujo típico de ejecución de trayectorias
1. Generación de trayectorias en RoboDK  
   Se define la secuencia de movimientos (MoveJ, MoveL, velocidades, herramientas, marcos de usuario) respetando límites articulares y cinemática del manipulador.
2. Post-proceso a código Motoman  
   RoboDK convierte la trayectoria simulada en un archivo `.JBI` en lenguaje INFORM/INFORM II-III compatible con el controlador Motoman.
3. Validación previa en simulación  
   Antes de mandar nada al robot se revisan colisiones, singularidades y accesibilidad en el entorno virtual.
4. Transferencia al controlador  
   El archivo `.JBI` se envía al controlador (DX100/DX200/FS100/YRC1000/etc.) a través de la red. RoboDK soporta transferencia directa de programas mediante FTP y también ofrece una acción “Send program to robot”.
5. Ejecución física  
   Existen dos modos:
   - Modo offline clásico: ejecutar el `.JBI` cargado como un job normal en el controlador.
   - Modo online (“Run on Robot”): el driver MotomanHSE mantiene la conexión y envía los movimientos punto a punto desde RoboDK mientras se monitorea el estado del robot.
6. Retroalimentación  
   Durante la ejecución online, RoboDK lee la pose real (articular y cartesiana) y refleja esa información en la simulación, manteniendo sincronizada la celda virtual con la celda física.

## Funciones específicas disponibles para Motoman
- Control directo desde RoboDK  
  Con el driver MotomanHSE, el robot puede ser movido directamente desde la estación virtual de RoboDK sin tener que regenerar y recargar archivos manualmente en cada iteración.
- Sincronización simulación ↔ robot real  
  “Run on Robot” hace que lo que se visualiza en RoboDK sea lo que el manipulador ejecuta físicamente, útil para depuración fina y ajuste de puntos.
- Gestión de programas  
  RoboDK puede generar múltiples jobs `.JBI`, organizarlos y enviarlos vía FTP al controlador. Esto soporta ciclos PC → controlador → prueba → ajuste.
- Variables / I/O  
  En controladores Motoman modernos, el canal HSE también se usa para consultar estado, job activo y, según la configuración del controlador, leer/escribir ciertas señales e I/O expuestas por red.

## Controladores soportados
- El protocolo High-Speed Ethernet Server y el driver MotomanHSE están documentados para controladores Motoman/Yaskawa como DX100, DX200, FS100, NX100, YRC1000 y YRC1000micro.
- En estos controladores, basta con configurar la IP del robot y colocarlo en modo REMOTE para habilitar la comunicación con RoboDK.
- Si el controlador no soporta HSE (por ejemplo generaciones XRC), aún es posible usar conexiones serie RS232 o soluciones basadas en MotoCom para tener acceso remoto.
- Además, estos mismos controladores Yaskawa suelen ofrecer servidor FTP integrado. Esto permite enviar/recibir archivos de programa (.JBI) y hacer respaldo de trabajos directamente por red sin usar memoria USB.


Para un mayor detalle de lo realizado, se puede ver el video de la simulación [aquí](https://www.youtube.com/watch?v=jPIcGKx4hjY) y el de la implementación física [aquí](https://www.youtube.com/watch?v=a09duI1kMos).

