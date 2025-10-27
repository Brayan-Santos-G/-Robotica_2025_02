# Laboratorio_02
* Sergio Avellaneda Piñeros
* David Santiago Cuellar Lopez
* Brayan Yesid Santos Gonzalez

## Cuadro comparativo


## Configuraciones (home1, home2)


## Movimientos manuales

El manipulador Motoman MH6 puede operarse manualmente mediante el teach pendant (programador de mano). Este dispositivo permite mover el robot de forma controlada para posicionarlo, programar trayectorias o verificar zonas de trabajo.
Los movimientos manuales pueden realizarse en dos modos principales: modo articular (Joint) y modo cartesiano (Base o Tool).

### 1. Activación del modo manual

1. 	En el teach pendant, seleccionar el modo “Teach” o “Manual”. Esto se realiza girando el selector de modo en el panel de control del robot.
2.	Confirmar que el interruptor de habilitación (Enable) esté presionado para permitir el movimiento.
3.	En la pantalla del teach pendant debe mostrarse el icono o texto “TEACH”, indicando que el robot está en modo manual.

### 2. Cambio entre modos de operación

El robot puede operarse de dos formas principales: modo articular (Joint) y modo cartesiano (Base o Tool).
En el modo articular, cada articulación del robot se controla de forma independiente. El operador puede mover los ejes J1 a J8 utilizando las teclas correspondientes en el teach pendant, realizando giros positivos o negativos en cada uno. Este modo resulta útil para ubicar al robot en posiciones generales, ajustar su postura o verificar la libertad de movimiento sin preocuparse por la orientación del efector final.
Por otro lado, el modo cartesiano permite desplazar el efector final del robot con respecto a los ejes del sistema de coordenadas. Este modo se selecciona accediendo al menú “COORD” del teach pendant, donde se puede elegir entre el sistema Base (WORLD), que utiliza el marco de referencia fijo del robot, o el sistema Tool (TCP), que toma como referencia el punto central de la herramienta instalada. En pantalla se indica el sistema de coordenadas activo, lo que facilita la verificación antes de ejecutar un movimiento.

### 3. Realización de traslaciones y rotaciones

Una vez seleccionado el modo cartesiano, el operador puede realizar traslaciones y rotaciones en los distintos ejes del espacio. Las traslaciones corresponden a desplazamientos lineales del efector final en las direcciones X, Y y Z, mientras que las rotaciones modifican la orientación de la herramienta alrededor de los ejes Rx, Ry y Rz. Estas operaciones se realizan mediante las teclas direccionales del teach pendant, siempre manteniendo presionado el interruptor de habilitación para garantizar la seguridad del movimiento.
Durante este proceso, es recomendable realizar movimientos lentos y controlados, especialmente al trabajar en zonas de proximidad o calibración. Si en algún momento se suelta o se presiona completamente el interruptor de habilitación, el robot se detendrá de inmediato, evitando colisiones o daños.


## Niveles de velocidad

En el teach pendant del Motoman MH6 los niveles de velocidad manual son mediante tres botones físicos claramente etiquetados: HIGH SPEED, FAST y SLOW. Cada uno corresponde a una categoría predefinida de velocidad que se aplica inmediatamente al movimiento manual cuando se pulsa. Para cambiar la velocidad durante la operación solo es necesario seleccionar el botón correspondiente; el cambio entra en efecto de forma instantánea y regula la rapidez de los desplazamientos realizados con el enable switch.

En la pantalla del teach pendant aparece el indicativo del nivel activo, por lo general mostrando el nombre del nivel (por ejemplo “HIGH SPEED” o “SLOW”) o un icono/etiqueta cercana al campo SPEED. Esto permite verificar visualmente qué nivel está seleccionado antes de ejecutar movimientos. Por seguridad y precisión, durante tareas de enseñanza, aproximaciones o trabajo en zonas con obstáculos se recomienda utilizar SLOW; FAST es apropiado para desplazamientos de ajuste y verificación; HIGH SPEED solo debe emplearse cuando el entorno es seguro y se requiere un desplazamiento rápido y amplio.


## Principales funcionalidades de RoboDK

RoboDK es un software de simulación y programación offline/online para robots industriales. Permite crear, validar y transferir trayectorias de robot sin depender exclusivamente de la enseñanza punto a punto en el controlador físico.

### Programación offline y simulación 3D
- Permite crear programas de robot de manera offline (Off-Line Programming), es decir, sin detener la producción real.
- Admite importación de geometría CAD en formatos STEP e IGES para definir piezas, herramientas, útiles y estaciones de trabajo.
- El entorno 3D permite ubicar el robot, la herramienta (TCP), referencias de trabajo y objetos de la celda como un gemelo digital básico.
- Se pueden predefinir targets cartesianos y articulares, velocidades, cambios de herramienta y marcos de usuario antes de mandarlos al robot físico.

### Detección de colisiones y validación de alcance
- El simulador incorpora chequeo de colisiones entre robot, herramienta y entorno; la simulación puede detener trayectorias que entrarían en contacto.
- Se validan singularidades, límites articulares, velocidades y accesibilidad antes de liberar un programa.
- Esta verificación previa reduce riesgos en planta y evita probar trayectorias peligrosas directamente sobre el manipulador.

### Post-procesadores y generación de código
- RoboDK incluye post-procesadores por marca que traducen las trayectorias simuladas al lenguaje nativo del controlador (p. ej., ABB RAPID, Yaskawa/Motoman INFORM/JBI, Fanuc TP/LS, KUKA KRL, URScript, etc.).
- Para Yaskawa/Motoman, el post genera archivos `.JBI` (INFORM) listos para ser cargados en el controlador.
- El post aplica reglas sintácticas, definición de herramientas y marcos, y el formato de movimientos requerido por cada fabricante.

### API y automatización
- API disponible para Python, C#, C/C++, Visual Basic (.NET) y MATLAB.
- Desde la API se pueden crear trayectorias, modificar marcos, leer poses, automatizar rutinas y generar programas por script.
- Con un driver activo, “Run on Robot” permite que el mismo script de simulación envíe movimientos al robot físico en tiempo real.

### Calibración y alineación físico-virtual
- Herramientas para calibrar el TCP (Tool Center Point) por contacto a punto o a plano, con múltiples orientaciones.
- Alineación y calibración de marcos de trabajo (workobjects / user frames) para que el sistema real coincida con el virtual.
- Módulos de calibración geométrica (p. ej., con láser tracker u ópticos) para mejorar la exactitud absoluta del robot y reducir el desfase entre celda simulada y real.

### Integración CAD/CAM y trayectorias de proceso
- Importa trayectorias CAM (fresado, corte láser/chorro, pulido, dispensado, soldadura, etc.) y las proyecta sobre la cinemática del robot.
- Genera movimientos continuos de herramienta sobre superficies 3D complejas con orientación, avance y velocidad de proceso.

### Programación online (“Run on Robot”)
- Además de generar y cargar archivos, RoboDK puede conectarse al robot y enviar movimientos en vivo mediante drivers por marca.
- En este modo, el robot real sigue la trayectoria de la simulación y su pose se refleja de vuelta en RoboDK para depuración y ajustes finos.


## Comunicación con el Manipulador Motoman

### Métodos de comunicación
- **Driver MotomanHSE (High-Speed Ethernet Server / “Remote”)**  
  Driver de RoboDK que utiliza el protocolo HSE soportado por controladores Yaskawa recientes. Permite mover y monitorear el robot desde RoboDK por Ethernet estándar (TCP/IP-UDP). Requiere poner el teach pendant en modo **REMOTE**. (RoboDK documenta un refresco de ~70 Hz).
- **RS232 (controladores antiguos)**  
  En controladores sin HSE (p. ej., XRC), la conexión puede realizarse por puerto serie especificando el COM en lugar de IP.
- **MotoCom + `apimotoman`**  
  Alternativa basada en la opción oficial MotoCom de Yaskawa cuando HSE no está disponible.

> Nota: RoboDK también puede **transferir programas por red** (típicamente FTP), además del control online. La transferencia no requiere activar el driver si solo se desea “enviar programa”.

### Flujo típico de ejecución de trayectorias
1. **Generación en RoboDK**: definir MoveJ/MoveL, velocidades, herramientas y marcos, respetando límites y cinemática.
2. **Post-proceso a Motoman**: generar `.JBI` en INFORM/INFORM II-III.
3. **Validación previa**: colisiones, singularidades, límites, accesibilidad.
4. **Transferencia al controlador**: enviar `.JBI` al robot (p. ej., vía FTP o “Send program to robot”).
5. **Ejecución física**:  
   - Modo offline clásico: ejecutar el `.JBI` cargado como job normal.  
   - Modo online (“Run on Robot”): con MotomanHSE, RoboDK envía movimientos y monitorea el estado en vivo.
6. **Retroalimentación**: durante la ejecución online se refleja la pose real (articular/cartesiana) en la simulación para sincronización y ajuste.

### Funciones específicas útiles para Motoman
- **Control directo con MotomanHSE** sin regenerar archivos en cada iteración.
- **Sincronización simulación ↔ robot real** en “Run on Robot” para depuración fina.
- **Gestión de programas**: generación de múltiples `.JBI` y envío por red.
- **Pulsos por grado (ALL.PRM)**: actualización de la relación pulsos/grado por eje a partir del respaldo del controlador para que la generación de programa coincida con la configuración del robot.

### Controladores soportados (según documentación de RoboDK)
- HSE documentado para **DX100, DX200, FS100, NX100 y YR/YRC1000**.  
- Si el controlador no soporta HSE (p. ej., XRC), usar **RS232** o **MotoCom**.
- Varios controladores Yaskawa incluyen **servidor FTP** para transferencia de archivos de programa (`.JBI`) y respaldos por red.


## RoboDK vs RobotStudio


## Diagrama de flujo

## Plano de planta de la ubicación de cada uno de los elementos
En la planta de trabajo tenemos el robot Motoman MH6 montado en su base, que es la referencia fija desde la cual el robot se mueve. Frente al robot está el workobject, que corresponde a la superficie donde se hacen las trayectorias del laboratorio. En el extremo del brazo del robot está instalada una herramienta con ventosas. Esa herramienta es la misma que tiene el robot físico del laboratorio y se incluye también en la simulación para que el modelo virtual coincida con el montaje real. Sin embargo, las ventosas no cumplen ninguna función dentro del desarrollo del laboratorio (no se usan para agarrar ni mover piezas); están ahí únicamente para mantener la misma configuración física y virtual.

<img width="721" height="559" alt="image" src="https://github.com/user-attachments/assets/68e4c01e-c3c6-4a51-a8f7-8a9e9be17299" />

## Código en Python
### Descripción del script de dibujo (cardioide y nombres)

El script en Python se conecta a RoboDK mediante la API, permite seleccionar el robot que se va a utilizar y trabaja dentro de un frame de referencia ya definido en la estación virtual. Una vez configurado el frame, la velocidad y una altura segura de trabajo, el programa genera dos trayectorias:

1. Dibujo de una cardioide:  
   - Se calcula una curva cardioide usando la ecuación polar r = 1 + cos(θ).  
   - Esa curva se convierte a coordenadas cartesianas (x, y) y se recorre punto a punto con movimientos lineales (`MoveL`) en un plano constante (z_surface).  
   - Esto simula que el TCP del robot “dibuja” la figura en la superficie.

2. Escritura de los nombres del equipo:  
   - Se usa `matplotlib` para convertir el texto (por ejemplo "DAVID BRAYAN SERGIO") en una nube de vértices 2D.  
   - El robot sigue esos vértices secuencialmente, trazando el contorno de las letras también mediante movimientos lineales en el mismo plano de trabajo.

Al finalizar cada trazo, el robot se eleva a una altura segura antes de desplazarse, para evitar colisiones con la mesa.  
El mismo script puede ejecutarse en simulación o, si se habilita la conexión remota, directamente sobre el robot físico.

Para este apartado se puede encontrar el código utilizado en la carpeta *"Códigos"*.

## Video de simulación e implementación
Para un mayor detalle de lo realizado, se puede ver el video de la simulación [aquí](https://www.youtube.com/watch?v=jPIcGKx4hjY) y el de la implementación física [aquí](https://www.youtube.com/watch?v=a09duI1kMos).

