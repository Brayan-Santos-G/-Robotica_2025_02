# Laboratorio_05
* Sergio Avellaneda Piñeros  
* David Santiago Cuellar Lopez  
* Brayan Yesid Santos Gonzalez  

## Descripción de la solución planteada

## Diagrama de flujo de acciones del robot 
```mermaid
graph TD
    %% --- ESTILOS VISUALES ---
    classDef ui fill:#E1F5FE,stroke:#0277BD,stroke-width:2px,rx:5,ry:5;
    classDef logic fill:#FFF9C4,stroke:#FBC02d,stroke-width:2px,rx:5,ry:5;
    classDef hardware fill:#E8F5E9,stroke:#2E7D32,stroke-width:2px,stroke-dasharray: 5 5;
    classDef start fill:#F3E5F5,stroke:#7B1FA2,stroke-width:3px,shape:circle;
    classDef error fill:#FFEBEE,stroke:#C62828,stroke-width:2px;

    %% --- BLOQUE 1: INICIO ---
    Start((INICIO)):::start --> Init[Inicializar Nodo ROS 2 Init Puerto Dynamixel]:::logic
    Init --> LaunchUI[Lanzar Interfaz PyQt5]:::ui

    %% --- BLOQUE 2: INTERFAZ DE USUARIO (GUI) ---
    subgraph GUI [" CAPA DE INTERFAZ (PyQt5) "]
        direction TB
        LaunchUI --> Stack{"Selección de Pestaña \n QStackedWidget"}:::ui
        
        Stack -- Pág 1 --> TabXYZ[Control XYZ \n Sliders X,Y,Z]:::ui
        Stack -- Pág 2 --> TabMan[Control Manual \n Sliders Articulares]:::ui
        Stack -- Pág 3/4 --> TabFix[Posiciones Fijas \n Botones Home, Sleep...]:::ui
    end

    %% --- BLOQUE 3: LÓGICA DE CONTROL ---
    subgraph LOGIC [" LÓGICA DE CONTROL (Python / Robotics Toolbox) "]
        direction TB
        
        %% Rama XYZ
        TabXYZ --> ReadCoords[Leer X, Y, Z]:::logic
        ReadCoords --> CalcIK[Calcular Cinemática Inversa \n robot.ikine_LM]:::logic
        CalcIK --> CheckSol{"¿Solución Válida?"}:::logic
        
        CheckSol -- No --> ShowErr[Mostrar Error / Ignorar]:::error
        CheckSol -- Sí --> ExtractQ[Extraer q1, q2, q3, q4]:::logic

        %% Rama Manual
        TabMan --> ReadSliders[Leer Valor Sliders directos]:::logic
        
        %% Rama Fijos
        TabFix --> LookupDict[Buscar en Diccionario \n Posiciones Predefinidas]:::logic
        
        %% Convergencia
        ReadSliders --> Merge((Unión)):::logic
        ExtractQ --> Merge
        LookupDict --> Merge
        
        Merge --> Validate[Validar Límites Articulares]:::logic
    end

    %% --- BLOQUE 4: HARDWARE ---
    subgraph HW [" CAPA DE HARDWARE (Dynamixel SDK) "]
        Validate --> SetAngles[Función: set_joint_angles]:::hardware
        SetAngles --> Packet[Empaquetar Datos \n write4ByteTxRx]:::hardware
        Packet --> Motor1[Motor 1: Base]:::hardware
        Packet --> Motor2[Motor 2: Hombro]:::hardware
        Packet --> Motor3[Motor 3: Codo]:::hardware
        Packet --> Motor4[Motor 4: Muñeca]:::hardware
    end

    %% --- RETROALIMENTACIÓN ---
    Motor1 -.-> Feedback[Estado Visual Robot]:::ui

```
## Plano de planta 

## Descripción de las funciones utilizadas

## Código del script utilizado para el desarrollo de la práctica

## Videos

- **Demostración de uso de la interfaz de usuario.**  
  Ver video [aquí](https://youtu.be/2warnZ_yuoM).

- **Uso de la interfaz de usuario + Gráfica digital de las poses vs la fotografía del brazo real.**  
  Ver video [aquí](https://youtu.be/8Tcm-70TWkw).

