# README_NAV_SUPERVISOR

## Objetivo del documento

Este documento resume el estado actual de la navegación portable del autito micromouse STM32 + HMI Qt, con foco en el modo `FIND_CELLS`, el supervisor portable, el mapa lógico y la detección de celdas especiales.

La intención es dejar una referencia rápida para continuar el desarrollo sin tener que reconstruir el contexto desde el código o desde chats anteriores.

---

## Arquitectura actual

La navegación está separada en capas:

```text
app_core.c
    Adaptador STM32:
    - lee sensores reales;
    - arma AppNavInput;
    - ejecuta el supervisor o primitive test;
    - aplica AppNavOutput a motores;
    - atiende comandos UNERBUS/HMI;
    - gestiona botón físico y OLED.

app_nav.c
    Capa portable de percepción, controladores y primitivas:
    - percepción de paredes/suelo;
    - control yaw-hold;
    - wall-follow;
    - advance action;
    - smooth action;
    - pivot action;
    - approach-front-wall action.

app_nav_supervisor.c
    Supervisor portable de misión:
    - decide qué primitiva iniciar;
    - secuencia FIND_CELLS;
    - actualiza mapa lógico;
    - detecta y cuenta celdas especiales;
    - finaliza misión al encontrar 3 especiales.

    Importante: la secuencia y las primitivas existen, pero la política
    actual de FIND_CELLS todavía no es una exploración completa del laberinto.

app_maze.c
    Mapa lógico portable:
    - celda actual x/y;
    - heading lógico;
    - paredes conocidas;
    - celdas visitadas;
    - celdas especiales detectadas.
```

Regla general:

```text
app_core adapta hardware.
app_nav ejecuta primitivas.
app_nav_supervisor decide misión.
app_maze guarda mapa lógico.
```

La lógica de misión no debe agregarse en `app_core.c` salvo como adaptación estrictamente necesaria.

---

## Convención lógica del mapa

El mapa lógico portable usa esta convención:

```text
X positivo: Este
Y positivo: Norte
```

Los headings son:

```c
HEADING_NORTH = 0
HEADING_EAST  = 1
HEADING_SOUTH = 2
HEADING_WEST  = 3
```

Por lo tanto:

```text
HEADING_NORTH -> y++
HEADING_EAST  -> x++
HEADING_SOUTH -> y--
HEADING_WEST  -> x--
```

La HMI o el simulador pueden usar otra convención visual, por ejemplo Y positivo hacia abajo. Esa conversión corresponde a la capa de UI/adaptador, no a `app_maze`.

En la HMI Qt real, la regla actual es:

```text
robot_maze_map[x][y] = coordenadas lógicas STM32.
current_x/current_y  = coordenadas lógicas STM32.
La conversión visual de Y se hace solo al dibujar.
```

Por lo tanto, las rutas de sincronización y actualización de mapa no deben guardar Y invertida. La inversión `scene_row = MAZE_HEIGHT - 1 - logical_y` pertenece exclusivamente al dibujo.

---

## Mapa lógico y bits de celda

Cada celda del mapa se guarda como un `uint8_t`.

Bits actuales:

```c
WALL_NORTH   = 0x01
WALL_SOUTH   = 0x02
WALL_EAST    = 0x04
WALL_WEST    = 0x08
CELL_VISITED = 0x10
CELL_SPECIAL = 0x20
```

`CELL_SPECIAL` se guarda en el mismo byte de celda que paredes y visitado.

Cuando se sincroniza el mapa hacia la HMI, se envía el byte completo de cada celda. Por eso la HMI puede visualizar celdas visitadas, paredes y celdas especiales detectadas.

---

## Modo FIND_CELLS

`FIND_CELLS` es la misión actualmente implementada a nivel de supervisor, primitivas, mapa lógico y conteo de especiales.

Objetivo nominal:

```text
Encontrar exactamente 3 celdas especiales únicas.
```

Estado real actual:

```text
FIND_CELLS todavía no está terminada como estrategia completa de exploración.
```

El supervisor puede iniciar la misión, mapear paredes/celdas visitadas, detectar celdas especiales y detenerse al llegar al contador objetivo. Sin embargo, la decisión de navegación todavía usa una política local simple. Por eso el autito puede encontrar las 3 celdas en mapas compatibles con esa heurística, pero no está garantizado que las encuentre en todos los mapas.

Cuando se detectan 3 celdas especiales:

```text
- se detienen las acciones portables;
- se ponen motores en 0;
- el supervisor vuelve a IDLE;
- active = 0;
- last_result = APP_NAV_SUPERVISOR_RESULT_FIND_CELLS_COMPLETE.
```

`GO_A_TO_B` existe como modo reservado, pero no está implementado y no debe mover el robot.

La mejora pendiente principal es reemplazar la política local actual por una navegación más inteligente basada en mapa, celdas visitadas/no visitadas y retorno/exploración planificada.

---

## Inicio de FIND_CELLS

El supervisor no exige arrancar con el sensor trasero sobre cinta.

Flujo inicial:

```text
START_INITIAL_ADVANCE
-> RUN_INITIAL_ADVANCE
-> DONE_REAR_TAPE
-> App_Maze_AdvanceRobotPosition()
-> DECIDE
```

La celda inicial no puede ser especial.

La primera acción de `FIND_CELLS` es avanzar desde la celda inicial hasta confirmar la primera línea con el sensor trasero.

---

## Punto de decisión

Las decisiones de navegación de alto nivel se toman cuando el robot está en un punto lógico válido, normalmente confirmado por el sensor trasero sobre una cinta de frontera de celda.

Regla conceptual:

```text
Cuando el sensor trasero pisa cinta, el robot ya ingresó/confirmó la celda nueva.
Desde esa celda lógica actual se decide la próxima acción.
```

La decisión no debe interpretarse como “completar la entrada” a esa celda, sino como “salir desde la celda actual hacia la próxima”.

---

## Política de navegación actual en FIND_CELLS

La política actual no usa todavía un planner ni flood fill. Tampoco prioriza explícitamente celdas no visitadas.

El supervisor llama a:

```c
App_Nav_RecommendAction(0U, &recommended_action)
```

`App_Nav_RecommendAction()` arma las opciones disponibles según las paredes percibidas en este orden:

```text
1. frente, si no hay pared frontal;
2. derecha, si no hay pared derecha;
3. izquierda, si no hay pared izquierda.
```

Como el valor usado para elegir es `random_value = 0`, se elige la primera opción válida de esa lista. La prioridad efectiva actual es:

```text
frente -> derecha -> izquierda -> volver/pivot 180
```

Si no hay salida al frente, derecha ni izquierda, la acción recomendada es `APP_NAV_ACTION_GO_BACK`, que el supervisor convierte en la secuencia de dead-end:

```text
approach front wall
-> pivot 180
-> advance hasta confirmar cinta trasera
-> DECIDE
```

Consecuencia práctica:

```text
Esta política puede recorrer mapas simples, pero puede ciclar, dejar ramas sin explorar o no llegar a las 3 celdas especiales en mapas más complejos.
```

La siguiente etapa del proyecto debería diseñar una política de exploración real para `FIND_CELLS`, por ejemplo usando memoria de celdas visitadas/no visitadas, fronteras, flood fill o una estrategia equivalente apta para STM32.

---

## Primitivas principales

### AdvanceAction

Avanza hasta que el sensor trasero confirma la próxima cinta de frontera.

Puede usar distintos perfiles de cinta trasera:

```c
APP_NAV_REAR_TAPE_PROFILE_NORMAL_CELL
APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL
APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL_AFTER_PIVOT
```

Estos perfiles existen porque en una celda especial el sensor trasero puede ver:

```text
cinta de entrada
-> blanco
-> cartulina especial
-> blanco
-> cinta de salida
```

En una celda normal ve:

```text
cinta de entrada
-> blanco
-> cinta de salida
```

### SmoothAction

Ejecuta un giro suave izquierda/derecha.

La acción smooth no se considera completa hasta que el sensor trasero confirma la cinta de frontera de la celda destino.

Puede tener fases:

```text
TURNING
-> POST_YAW_SEEK_REAR_TAPE
-> DONE_POST_YAW_REAR_TAPE
```

o terminar directamente por:

```text
DONE_REAR_TAPE
```

si el sensor trasero detecta la cinta durante la fase de giro.

### PivotAction

Pivote en celda.

Actualiza orientación lógica, pero no avanza celda.

### ApproachFrontWallForPivotAction

Avance controlado hacia pared frontal antes de un pivot 180 en dead-end.

No actualiza mapa por sí solo.

---

## Smooth: yaw, diagonal y post-yaw

Decisión importante tomada:

```text
La detección por sensor diagonal/pared durante un smooth NO termina la acción smooth.
Solo termina la fase curva del giro.
```

Antes, cuando el diagonal detectaba pared, el smooth podía terminar como:

```text
APP_NAV_SMOOTH_ACTION_DONE_WALL
```

Eso hacía que el supervisor:

```text
- actualizara heading;
- no actualizara celda;
- iniciara un ADVANCE separado.
```

Ese flujo podía atrasar el mapa lógico una celda, especialmente al salir de una celda especial.

Ahora la detección por diagonal/pared entra a:

```text
APP_NAV_SMOOTH_ACTION_POST_YAW_SEEK_REAR_TAPE
```

igual que cuando el smooth termina la fase curva por yaw target.

Flujo actual correcto:

```text
diagonal/wall detectado
-> POST_YAW_SEEK_REAR_TAPE
-> avanzar recto con yaw-hold
-> esperar cinta trasera
-> DONE_POST_YAW_REAR_TAPE
-> supervisor actualiza heading + celda
```

`APP_NAV_SMOOTH_ACTION_DONE_WALL` queda como estado legacy/defensivo. Si llega al supervisor, se trata como error de primitiva.

---

## Detección de celdas especiales

Una celda especial se detecta al confirmar la entrada a una celda.

Condición base:

```text
floor_front_black == true
floor_rear_black  == true
```

Pero no se evalúa en cualquier momento. Se evalúa justo después de actualizar la posición lógica de celda.

Ejemplo para avance:

```text
AdvanceAction termina por DONE_REAR_TAPE
-> App_Maze_AdvanceRobotPosition()
-> si front_black && rear_black:
       App_Maze_MarkCurrentCellSpecial()
```

Ejemplo para smooth:

```text
SmoothAction termina por DONE_REAR_TAPE o DONE_POST_YAW_REAR_TAPE
-> App_Maze_UpdateRobotHeading(turn)
-> App_Maze_AdvanceRobotPosition()
-> si front_black && rear_black:
       App_Maze_MarkCurrentCellSpecial()
```

Esto evita marcar la celda anterior por error.

`App_Maze_MarkCurrentCellSpecial()` devuelve:

```text
true  si CELL_SPECIAL se marcó por primera vez;
false si la celda ya estaba marcada como especial.
```

El supervisor incrementa el contador solo cuando la función devuelve `true`.

---

## Salida desde celda especial

Si la celda actual está marcada con `CELL_SPECIAL`, el supervisor inicia `AdvanceAction` o `SmoothAction` con perfil especial.

Caso normal:

```text
APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL
```

Caso especial después de pivot 180 en dead-end:

```text
APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL_AFTER_PIVOT
```

Ese tercer perfil existe porque después de:

```text
approach front wall
-> pivot 180
-> advance de salida
```

el sensor trasero puede arrancar ya sobre la cartulina especial, no sobre la cinta de entrada.

---

## Dead-end

Cuando el supervisor decide volver en un callejón sin salida:

```text
GO_BACK
-> RUN_APPROACH_FRONT_WALL_FOR_PIVOT
-> RUN_PIVOT_180
-> RUN_ADVANCE
-> DECIDE
```

Después del pivot 180, el robot no decide inmediatamente. Primero avanza hasta confirmar cinta trasera, para salir correctamente de la celda.

---

## Supervisor result codes

Valores públicos de `last_result`:

```c
APP_NAV_SUPERVISOR_RESULT_OK                  = 0
APP_NAV_SUPERVISOR_RESULT_INVALID_ARGUMENT    = 1
APP_NAV_SUPERVISOR_RESULT_START_FAILED        = 2
APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR     = 3
APP_NAV_SUPERVISOR_RESULT_UNSUPPORTED_ACTION  = 4
APP_NAV_SUPERVISOR_RESULT_FIND_CELLS_COMPLETE = 5
```

Cuando `FIND_CELLS` termina correctamente:

```text
state       = APP_NAV_SUPERVISOR_IDLE
active      = 0
last_result = APP_NAV_SUPERVISOR_RESULT_FIND_CELLS_COMPLETE
```

---

## Flujo STM32 app_core

El loop de control prioriza:

```text
1. primitive_test.active
2. supervisor_run_active
3. motores en 0 si no hay flujo activo
```

Esquema conceptual:

```text
Run_Control_Step()
    ADC_Filter_Task()
    Update sensor snapshot
    Run_Portable_Nav_Tick()           // percepción/debug
    Apply perception to snapshot

    if primitive_test.active:
        PrimitiveTest_Tick()
        return

    if supervisor_run_active:
        Tick_Supervisor_Run()
        return

    Set_Motor_Speeds(0, 0)
```

`App_Nav_Tick()` se conserva como shell de percepción/debug. No ejecuta misión.

La misión real se ejecuta mediante:

```text
Tick_Supervisor_Run()
-> App_NavSupervisor_Tick()
-> AppNavOutput
-> motores
```

---

## Comandos HMI relevantes

Comandos nuevos/importantes:

```c
CMD_SET_SUPERVISOR_INITIAL_POSE  = 0x98
CMD_GET_SUPERVISOR_INITIAL_POSE  = 0x99
CMD_START_SUPERVISOR_RUN         = 0x9A
CMD_STOP_SUPERVISOR_RUN          = 0x9B
CMD_GET_SUPERVISOR_DEBUG_STATUS  = 0x9C
```

También existen comandos de configuración para parámetros de navegación, primitive test y sincronización de mapa.

La pose inicial se configura desde la HMI. El start del supervisor usa la pose/configuración ya cargada en STM32. No debe reconstruir defaults ni enviar configuración automáticamente desde la HMI al iniciar.

`CMD_GET_SUPERVISOR_DEBUG_STATUS` devuelve un payload de 9 bytes:

```text
[0] state
[1] current_action
[2] active
[3] last_result
[4] maze_x
[5] maze_y
[6] maze_heading
[7] maze_cell
[8] special_found_count
```

`maze_cell` es el byte completo de celda, con paredes, `CELL_VISITED` y `CELL_SPECIAL`. `special_found_count` es el contador de celdas especiales únicas detectadas por `FIND_CELLS`.

---

## HMI y mapa

La página principal de operación real es `pageLaberinth`. Debe entenderse como un panel operativo STM32, no como simulador local.

Estado actual de esa página:

```text
- Reset vista local: limpia solo la representación local de la HMI, no la memoria STM32.
- Sincronizar mapa con STM32: solicita columnas del mapa real mantenido por firmware.
- Pose inicial: envía/lee celda X/Y y heading inicial del supervisor.
- Run del laberinto: permite iniciar/detener FIND_CELLS; GO_A_TO_B aparece como modo reservado.
- Estado supervisor: muestra active, state, action, result, pose, cell byte y special_count.
```

La HMI puede sincronizar manualmente el mapa desde STM32 por columnas.

La sincronización envía el byte completo de cada celda, incluyendo:

```text
paredes
CELL_VISITED
CELL_SPECIAL
```

Las celdas especiales detectadas por firmware deben visualizarse a partir del bit `CELL_SPECIAL`.

---

## Simulador

El simulador usa una copia sincronizada del firmware portable bajo `firmware_core`.

Flujo de trabajo:

```cmd
tools\sync_firmware_core_from_stm32.cmd "<RUTA_AL_REPO_STM32>"

Ejemplos en uso:

tools\sync_firmware_core_from_stm32.cmd "C:\Users\GAMING\Desktop\MICROCONTROLADORES\MICROCONTROLADORES-STM32" en computadora desktop
tools\sync_firmware_core_from_stm32.cmd "C:\Users\ivanl\OneDrive\Escritorio\MICROS_ACTUALIZADO\MICROCONTROLADORES\MICROCONTROLADORES-STM32" en notebook
```

El simulador representa celdas especiales físicas desde JSON mediante `special_cells`.

Importante:

```text
La marca física del JSON es ground truth.
CELL_SPECIAL en el overlay lógico representa lo que el firmware cree haber detectado.
```

Ambas cosas deben distinguirse visualmente.

---

## Pruebas validadas

Estado validado hasta este punto:

```text
- pasillo recto;
- mapa L;
- mapa U;
- dead-end con approach + pivot 180 + advance;
- detección de celdas especiales en mapas/escenarios de prueba compatibles con la política actual;
- conteo de celdas especiales únicas en autito real;
- salida desde celda especial normal;
- salida desde celda especial con smooth;
- salida desde celda especial en dead-end;
- smooth con diagonal/wall entrando a post-yaw sin atrasar el mapa.
```

---

## Problemas ya corregidos

### 1. Celda especial en dead-end

Problema:

```text
Después de pivot 180, el avance de salida desde una celda especial podía interpretar mal la cartulina.
```

Solución:

```text
APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL_AFTER_PIVOT
```

### 2. Smooth terminado por diagonal/pared

Problema:

```text
DONE_WALL actualizaba heading pero no celda, y arrancaba un ADVANCE separado.
Eso podía atrasar el mapa lógico una celda.
```

Solución:

```text
Diagonal/wall ya no termina la acción.
Entra a POST_YAW_SEEK_REAR_TAPE y espera confirmación por sensor trasero.
```

### 3. Detección prematura de especial

Problema potencial:

```text
Si se marcaba especial antes de actualizar posición lógica, podía marcarse la celda anterior.
```

Solución:

```text
La detección especial se evalúa solo después de App_Maze_AdvanceRobotPosition().
```

---

## Pendientes conocidos

Pendiente principal inmediato:

```text
- Completar FIND_CELLS con una navegación inteligente de exploración.
```

La misión actual no debe considerarse cerrada solo porque el contador de especiales y las primitivas funcionen. Falta una estrategia capaz de explorar el mapa de forma robusta, evitar ciclos y dirigirse hacia celdas/fronteras no exploradas cuando la heurística local ya no alcanza.

No trabajar todavía en estos puntos salvo decisión explícita:

```text
- GO_A_TO_B.
- auto-sync continuo de mapa.
- limpieza profunda de pageControl en HMI.
- extracción grande de app_core.c en varios módulos.
- cambio de protocolo UNERBUS.
```

Posibles mejoras futuras:

```text
- planner/floodfill/frontier o estrategia equivalente para FIND_CELLS;
- mejorar telemetría de perfiles de cinta trasera;
- documentar comandos UNERBUS por familia;
- agregar pruebas sistemáticas para mapas con special_cells;
- revisar si APP_NAV_SMOOTH_ACTION_DONE_WALL puede eliminarse definitivamente.
```

---

## Checklist antes de seguir desarrollando

Antes de agregar lógica nueva, verificar:

```text
[ ] STM32 compila.
[ ] HMI compila.
[ ] Mapa simple/controlado con 3 especiales es recorrible con la política actual.
[ ] Mapa con especial en dead-end funciona.
[ ] Mapa con smooth desde celda especial funciona.
[ ] Autito real encuentra 3 especiales en un mapa de prueba compatible.
[ ] Sync manual de mapa STM32 -> HMI muestra CELL_SPECIAL correctamente.
[ ] Start/Stop supervisor desde HMI funciona.
[ ] Start/Stop por botón físico funciona.
[ ] Primitive test smooth sigue funcionando.
```

---

## Regla de mantenimiento

Para esta etapa, la fuente de verdad es el repo STM32 + Qt real. No modificar el simulador salvo que se decida explícitamente volver a usarlo como banco de pruebas.

Cuando se agregue navegación nueva:

```text
1. Primero analizar el cambio y el contrato esperado.
2. Implementar en código portable cuando corresponda.
3. Probar en autito real con mapa simple/controlado.
4. Recién después probar casos límite.
5. Si se decide usar simulador, sincronizar firmware portable desde STM32 hacia Sim_Autito y probar ahí como banco adicional.
```

No poner lógica de misión directamente en `app_core.c` si puede vivir en `app_nav_supervisor.c`.