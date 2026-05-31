# README_NAV_SUPERVISOR

## Objetivo

Este documento resume el estado actual de la navegación portable del autito micromouse STM32 + HMI Qt, con foco en:

- supervisor portable `app_nav_supervisor`;
- modo `FIND_CELLS`;
- modo `GO_A_TO_B`;
- política inteligente `app_find_cells_policy`;
- política optimista `app_go_to_b_policy`;
- planner común `app_route_planner`;
- mapa lógico `app_maze`;
- primitivas de navegación y percepción portable `app_nav`;
- integración con HMI Qt, publicación autónoma de estado y simulador.

La intención es que este archivo sea la referencia principal para continuar el desarrollo sin reconstruir contexto desde chats anteriores.

> Fuente de verdad funcional al actualizar este README: repo STM32+Qt real versión `repomix-output-resumido-autitoReal-5.xml` de este chat, con actualización de percepción única por tick.

---

## Regla general de arquitectura

La separación de responsabilidades vigente es:

```text
app_core.c
    Adaptador STM32:
    - lee sensores reales;
    - arma AppNavInput;
    - evalúa percepción portable con App_Nav_EvaluatePerception(...);
    - actualiza sensor_snapshot.detection_flags desde AppNavPerception;
    - ejecuta supervisor o primitive tests;
    - aplica AppNavOutput a motores;
    - atiende comandos UNERBUS/HMI;
    - gestiona botón físico y OLED;
    - no debe contener lógica de misión si puede vivir en el supervisor.

app_nav.c
    Capa portable de percepción, controladores y primitivas:
    - convierte AppNavInput en AppNavPerception;
    - aplica histéresis de paredes/suelo;
    - wall-follow;
    - yaw-hold;
    - AdvanceAction;
    - SmoothAction;
    - PivotAction;
    - ApproachFrontWallAction;
    - CenterByFrontTapeForPivotAction;
    - helper interno común de avance guiado: wall-follow -> fallback yaw-hold.

app_find_cells_policy.c
    Política portable de alto nivel para FIND_CELLS:
    - decide próxima acción conceptual de exploración;
    - prioriza vecinos no visitados;
    - define qué es una frontera de exploración;
    - carga fronteras como seeds del planner común;
    - usa app_route_planner en modo KNOWN_OPEN_VISITED_ONLY;
    - no mueve motores;
    - no accede a HAL;
    - no ejecuta primitivas.

app_go_to_b_policy.c
    Política portable de alto nivel para GO_A_TO_B:
    - decide próxima acción conceptual hacia una celda objetivo B;
    - carga B como seed del planner común;
    - usa app_route_planner en modo OPTIMISTIC_UNKNOWN_ALLOWED;
    - bloquea solo paredes conocidas;
    - permite edges desconocidas como hipótesis de camino;
    - no mueve motores;
    - no accede a HAL;
    - no ejecuta primitivas.

app_route_planner.c
    Planner BFS/flood fill portable común:
    - calcula campos de distancia sobre el mapa lógico;
    - usa workspace estático compartido y no reentrante;
    - soporta modo seguro por edges conocidas abiertas y celdas visitadas;
    - soporta modo optimista bloqueando solo paredes conocidas;
    - no decide acciones ni reasons de misión;
    - no mueve motores;
    - no modifica el mapa.

app_nav_supervisor.c
    Supervisor portable de misión:
    - mapea celda actual;
    - consulta app_find_cells_policy o app_go_to_b_policy según misión;
    - decide qué primitiva arrancar;
    - secuencia primitivas;
    - actualiza pose lógica después de movimientos confirmados;
    - marca celdas especiales para perfiles de suelo;
    - cuenta celdas especiales solo en FIND_CELLS;
    - finaliza la misión.

app_maze.c
    Mapa lógico portable:
    - pose lógica x/y/heading;
    - celdas visitadas;
    - paredes presentes;
    - celdas especiales;
    - edges conocidas/open/desconocidas para planificación;
    - helpers geométricos comunes de celda, heading, indexado y direcciones relativas.
```

Regla estricta:

```text
app_core adapta hardware y publica flags de detección derivados de AppNavPerception.
app_nav evalúa percepción portable y ejecuta primitivas.
app_find_cells_policy decide exploración FIND_CELLS.
app_go_to_b_policy decide navegación optimista GO_A_TO_B.
app_route_planner calcula distancias BFS/flood fill comunes.
app_nav_supervisor secuencia misión y actualiza mapa.
app_maze guarda el mapa lógico y helpers geométricos.
```

---

## Percepción portable `AppNavPerception`

La percepción operativa de bajo nivel está separada del debug y se concentra en:

```c
bool App_Nav_EvaluatePerception(const AppNavInput *input,
                                AppNavPerception *perception_out);
```

Entrada:

```text
AppNavInput
    Mediciones ya disponibles para la capa portable:
    - ADC filtrados de sensores de piso;
    - distancias IR en milímetros;
    - yaw/yaw-rate y dt.

    No contiene flags de piso negro. Las flags se calculan en
    AppNavPerception a partir de los ADC de piso.
```

Salida:

```text
AppNavPerception
    Interpretación portable filtrada:
    - floor_front_black;
    - floor_rear_black;
    - wall_front;
    - wall_left;
    - wall_right;
    - wall_diag_left;
    - wall_diag_right;
    - ADCs de piso;
    - distancias IR copiadas.
```

Reglas importantes:

```text
- AppNavPerception es percepción de producción, no debug.
- App_Nav_EvaluatePerception(...) aplica la histéresis de paredes y suelo.
- App_Nav_EvaluatePerception(...) se llama una sola vez por tick de control, desde app_core.c.
- app_nav.c conserva internamente el estado previo necesario para esa histéresis.
- app_core pasa el mismo snapshot AppNavPerception al supervisor y a las primitivas.
- app_nav_supervisor usa AppNavPerception para mapear paredes de la celda actual y detectar celdas especiales.
- app_core usa AppNavPerception para actualizar sensor_snapshot.detection_flags.
- App_Nav_Tick(), App_Nav_GetDebug(), AppNavDebug y app_nav_debug.h fueron eliminados.
```

Flujo operativo actual:

```text
Sensores STM32 / simulador
-> SensorSnapshotTypeDef
-> AppNavInput
-> App_Nav_EvaluatePerception(...) una vez por tick
-> AppNavPerception
-> detection_flags / mapeo de paredes / detección de especiales / primitivas
```

Separación deseada:

```text
percepción operativa = AppNavPerception
estado de misión     = AppNavSupervisorDebug
```

No se debe volver a usar una estructura de debug como fuente obligatoria para mapear paredes o detectar piso.

---

## Convención lógica del mapa

El firmware portable usa una convención lógica propia:

```text
X positivo: Este
Y positivo: Norte
```

Headings:

```c
HEADING_NORTH = 0
HEADING_EAST  = 1
HEADING_SOUTH = 2
HEADING_WEST  = 3
```

Movimiento lógico:

```text
HEADING_NORTH -> y++
HEADING_EAST  -> x++
HEADING_SOUTH -> y--
HEADING_WEST  -> x--
```

La HMI Qt o el simulador pueden dibujar con Y visual hacia abajo, pero esa conversión debe hacerse solo en la capa de UI/dibujo. Internamente:

```text
robot_maze_map[x][y] = coordenadas lógicas STM32
current_x/current_y  = coordenadas lógicas STM32
conversión visual Y  = solo al dibujar
```

---

## Mapa lógico: byte de celda

Cada celda sincronizada STM32/Qt se guarda como `uint8_t`:

```c
#define WALL_NORTH   0x01
#define WALL_SOUTH   0x02
#define WALL_EAST    0x04
#define WALL_WEST    0x08
#define CELL_VISITED 0x10
#define CELL_SPECIAL 0x20
```

Semántica:

```text
WALL_*       = pared presente
CELL_VISITED = celda visitada
CELL_SPECIAL = celda especial detectada
```

El byte completo se usa para sincronización con la HMI/simulador, por lo que no se deben agregar bits nuevos ahí sin verificar compatibilidad.

Ejemplo:

```text
0x1E = 0x10 + 0x08 + 0x04 + 0x02

CELL_VISITED = sí
WALL_WEST    = sí
WALL_EAST    = sí
WALL_SOUTH   = sí
WALL_NORTH   = no
CELL_SPECIAL = no
```

---

## Known edges

Para planificación inteligente se agregó un mapa interno separado:

```c
static uint8_t maze_known_edges[MAZE_WIDTH][MAZE_HEIGHT];
```

Bits:

```c
#define EDGE_KNOWN_NORTH 0x01
#define EDGE_KNOWN_SOUTH 0x02
#define EDGE_KNOWN_EAST  0x04
#define EDGE_KNOWN_WEST  0x08
```

Esto resuelve la ambigüedad entre:

```text
no hay pared conocida
```

y:

```text
todavía no sé si hay pared
```

La semántica correcta es:

```text
edge desconocida:
    EDGE_KNOWN = 0
    WALL       = 0

edge abierta conocida:
    EDGE_KNOWN = 1
    WALL       = 0

edge con pared conocida:
    EDGE_KNOWN = 1
    WALL       = 1
```

`App_Maze_MapCurrentCell(...)` marca como conocidas las direcciones observadas desde la pose actual:

```text
frente
derecha
izquierda
```

Eso ocurre haya pared o no. Además, el conocimiento de edge se espeja hacia la celda vecina cuando existe.

Ejemplo:

```text
Desde (x,y) observo NORTH sin pared:
    known_edges[x][y] |= NORTH
    known_edges[x][y+1] |= SOUTH

maze_map no recibe WALL_NORTH porque no hay pared.
```

El planner seguro debe cruzar solo si:

```text
App_Maze_IsKnownOpenEdge(x, y, dir) == true
```

---

## app_route_planner: BFS/flood fill común

`app_route_planner` es el módulo común de planificación por BFS/flood fill sobre el mapa lógico.

No es una policy de misión. No decide acciones, no devuelve reasons de misión, no mueve motores y no modifica el mapa. Su responsabilidad es generar un campo de distancias que luego interpretan las policies.

Flujo de uso:

```text
1. App_RoutePlanner_Reset()
2. App_RoutePlanner_AddSeed(...)
3. App_RoutePlanner_Run(mode)
4. App_RoutePlanner_GetDistance(...)
```

El workspace es estático y compartido. El módulo es no reentrante por diseño: una policy debe completar el ciclo reset -> seeds -> run -> query antes de que otra evaluación vuelva a usarlo.

Modos actuales:

```text
APP_ROUTE_TRAVERSAL_KNOWN_OPEN_VISITED_ONLY
    Cruza solo si:
        - existe vecino;
        - la edge es conocida abierta;
        - el vecino está visitado.

    Lo usa FIND_CELLS para rutas seguras hacia fronteras.

APP_ROUTE_TRAVERSAL_OPTIMISTIC_UNKNOWN_ALLOWED
    Cruza solo si:
        - existe vecino;
        - no hay pared conocida en esa dirección.

    Permite edges desconocidas como hipótesis de camino.
    Lo usa GO_A_TO_B para planificación optimista hacia B.
```

Separación correcta:

```text
app_route_planner:
    calcula distancias.

app_find_cells_policy / app_go_to_b_policy:
    interpretan distancias y devuelven decisiones conceptuales.

app_nav_supervisor:
    convierte decisiones conceptuales en secuencias de primitivas.
```

---

## Modo FIND_CELLS

`FIND_CELLS` es la misión implementada actualmente.

Objetivo:

```text
Encontrar 3 celdas especiales únicas.
```

Finalizaciones posibles:

```text
FIND_CELLS_COMPLETE
    Se detectaron 3 celdas especiales únicas.

FIND_CELLS_INCOMPLETE_NO_FRONTIER
    No quedan fronteras alcanzables y se encontraron menos de 3 especiales.

ERROR
    Falló una primitiva, argumento, start, o acción no soportada.
```


---

## Modo GO_A_TO_B

`GO_A_TO_B` es una misión implementada y validada inicialmente en Bluepill.

Objetivo:

```text
Ir desde la celda A actual/inicial hasta una celda objetivo B
sin asumir conocimiento previo del mapa.
```

Reglas fijadas:

```text
A = pose lógica inicial/current pose configurada desde HMI.
B = celda objetivo configurada desde HMI.
Al iniciar GO_A_TO_B se resetea el mapa lógico.
GO_A_TO_B no depende de haber ejecutado FIND_CELLS antes.
```

Finalizaciones posibles:

```text
GO_TO_B_COMPLETE
    La pose lógica actual coincide con B.

GO_TO_B_INVALID_TARGET
    B no fue configurada o está fuera del mapa.
    Debe fallar sin mover.

GO_TO_B_NO_PATH
    Durante la ejecución, las paredes descubiertas hacen imposible llegar a B
    incluso permitiendo edges desconocidas de forma optimista.

ERROR / PRIMITIVE_ERROR
    Falló una primitiva o el arranque de una acción física.
```

### Inicio de GO_A_TO_B

Si `A == B`:

```text
termina sin mover con GO_TO_B_COMPLETE
```

Si `A != B`:

```text
START_INITIAL_ADVANCE
-> RUN_INITIAL_ADVANCE
-> DECIDE
```

Esto es intencional. El robot se asume físicamente en el centro de una celda al arrancar, por lo que debe hacer la adquisición inicial/controlada antes de tomar decisiones de ruta. No debe arrancar directamente en `DECIDE`.

### Policy de GO_A_TO_B

La política está en:

```text
app_go_to_b_policy.c/h
```

No es shadow/debug; es lógica de producción.

`App_GoToBPolicy_Evaluate(...)` devuelve una decisión conceptual:

```text
GOAL_REACHED
    La celda actual ya es B.

ROUTE_STEP
    Existe un próximo paso de ruta hacia B.

BACKTRACK_REQUIRED
    La ruta hacia B empieza hacia atrás respecto al heading actual.
    No debe traducirse a APP_NAV_ACTION_GO_BACK.

NO_PATH
    No hay ruta posible hacia B con el mapa parcial actual,
    aun suponiendo transitables las edges desconocidas.
```

### BFS optimista

`GO_A_TO_B` usa el planner común `app_route_planner`.

En cada evaluación, la policy carga B como única seed:

```text
App_RoutePlanner_Reset()
App_RoutePlanner_AddSeed(goal_x, goal_y)
App_RoutePlanner_Run(APP_ROUTE_TRAVERSAL_OPTIMISTIC_UNKNOWN_ALLOWED)
```

Criterio de cruce del modo optimista:

```text
si la celda vecina no existe:
    no cruzar

si hay pared conocida:
    no cruzar

si la edge está abierta conocida:
    cruzar

si la edge es desconocida:
    cruzar optimistamente
```

Diferencia central con `FIND_CELLS`:

```text
FIND_CELLS usa APP_ROUTE_TRAVERSAL_KNOWN_OPEN_VISITED_ONLY.
GO_A_TO_B usa APP_ROUTE_TRAVERSAL_OPTIMISTIC_UNKNOWN_ALLOWED.
```

El planner común recalcula distancias en cada `DECIDE`, por lo que al descubrir una pared nueva la policy puede elegir otra ruta o terminar con `GO_TO_B_NO_PATH`.

### Desempate de ruta

Cuando hay varios pasos equivalentes hacia B:

```text
frente -> derecha -> izquierda -> atrás
```

Si el siguiente paso elegido es atrás, la policy reporta `BACKTRACK_REQUIRED`. El supervisor decide la preparación física del pivot 180 según la geometría frontal actual.


---

## Inicio de FIND_CELLS

Al iniciar el supervisor:

```text
App_NavSupervisor_SetMission(APP_NAV_SUPERVISOR_MISSION_FIND_CELLS)
App_NavSupervisor_Start()
```

Flujo inicial:

```text
START_INITIAL_ADVANCE
-> RUN_INITIAL_ADVANCE
-> DECIDE
```

El avance inicial confirma una cinta trasera válida antes de empezar a tomar decisiones de celda.

---

## Punto de decisión

Regla conceptual:

```text
Cuando el sensor trasero pisa cinta,
el robot ya ingresó/confirmó la celda nueva.
Desde esa celda lógica actual se decide la próxima acción.
```

La decisión no significa “terminar de entrar a la celda actual”; significa:

```text
salir desde la celda lógica actual hacia la próxima celda
```

Por eso el supervisor actualiza mapa/pose cuando las primitivas confirman eventos físicos como:

```text
AdvanceAction DONE_REAR_TAPE
SmoothAction DONE_REAR_TAPE
SmoothAction DONE_POST_YAW_REAR_TAPE
PivotAction DONE
```

---

## Política inteligente de FIND_CELLS

La política de exploración está en:

```text
app_find_cells_policy.c/h
```

No es shadow/debug; es lógica de producción.

### Prioridad general

En cada punto de decisión:

```text
1. Mapear celda actual.
2. Si ya hay 3 especiales, terminar con FIND_CELLS_COMPLETE.
3. Intentar vecino inmediato no visitado:
       frente -> derecha -> izquierda.
4. Si no hay vecino inmediato ejecutable:
       usar app_route_planner con BFS multi-source hacia frontera.
5. Si la ruta hacia frontera empieza:
       frente  -> ADVANCE
       derecha -> SMOOTH_RIGHT
       izquierda -> SMOOTH_LEFT
       atrás   -> BACKTRACK_REQUIRED
6. Si no hay frontera:
       FIND_CELLS_INCOMPLETE_NO_FRONTIER.
```

### Desempate

Cuando varias opciones tienen el mismo costo:

```text
frente -> derecha -> izquierda -> atrás
```

Esta prioridad se usa tanto para:

```text
vecinos inmediatos no visitados
```

como para:

```text
elección de vecino con menor distancia hacia frontera
```

---

## Fronteras de exploración

Una frontera de exploración es:

```text
celda visitada
+ edge conocida abierta
+ vecino no visitado
```

Formalmente:

```text
App_Maze_IsCellVisited(x, y)
&& App_Maze_IsKnownOpenEdge(x, y, dir)
&& App_Maze_GetNeighbor(x, y, dir, &nx, &ny)
&& !App_Maze_IsCellVisited(nx, ny)
```

El robot se mueve por celdas visitadas hasta una celda frontera, y desde ahí entra a una celda no visitada.

---

## Flood fill / BFS

Para `FIND_CELLS` se usa el planner común `app_route_planner` como BFS multi-source hacia fronteras.

La policy sigue definiendo qué es una frontera. Luego carga todas las fronteras como seeds del planner común y ejecuta:

```text
App_RoutePlanner_Run(APP_ROUTE_TRAVERSAL_KNOWN_OPEN_VISITED_ONLY)
```

Características:

```text
- sin heap;
- sin malloc;
- sin recursión;
- sin Dijkstra;
- sin A*;
- costos uniformes por celda;
- workspace estático compartido en app_route_planner;
- apto para STM32F103/Bluepill.
```

Flujo del BFS para `FIND_CELLS`:

```text
1. App_RoutePlanner_Reset().
2. Buscar todas las celdas frontera.
3. Cargar todas las fronteras como seeds con distancia 0.
4. Ejecutar APP_ROUTE_TRAVERSAL_KNOWN_OPEN_VISITED_ONLY.
5. Desde la celda actual, elegir el vecino con menor distancia.
```

La propagación no atraviesa celdas no visitadas. La entrada a una celda no visitada ocurre solo desde una frontera local.

---

## Decisiones de app_find_cells_policy

`App_FindCellsPolicy_Evaluate(...)` puede devolver:

```c
APP_FIND_CELLS_DECISION_REASON_NONE
APP_FIND_CELLS_DECISION_REASON_IMMEDIATE_UNVISITED
APP_FIND_CELLS_DECISION_REASON_ROUTE_TO_FRONTIER
APP_FIND_CELLS_DECISION_REASON_BACKTRACK_REQUIRED
APP_FIND_CELLS_DECISION_REASON_NO_FRONTIER
```

Interpretación:

```text
IMMEDIATE_UNVISITED
    Hay vecino no visitado inmediato en frente/derecha/izquierda.

ROUTE_TO_FRONTIER
    No hay vecino inmediato no visitado, pero existe una frontera alcanzable
    por camino conocido. El próximo paso es frente/derecha/izquierda.

BACKTRACK_REQUIRED
    Existe ruta hacia frontera, pero el próximo paso requerido es hacia atrás.
    No debe traducirse a APP_NAV_ACTION_GO_BACK.

NO_FRONTIER
    No queda ninguna frontera alcanzable.
```

Regla crítica:

```text
BACKTRACK_REQUIRED != APP_NAV_ACTION_GO_BACK
```

`APP_NAV_ACTION_GO_BACK` queda reservado para la recomendación local de dead-end. El supervisor maneja `BACKTRACK_REQUIRED` con una preparación de pivot elegida según la geometría frontal.

---

## Manejo de BACKTRACK_REQUIRED

Cuando la policy devuelve:

```text
APP_FIND_CELLS_DECISION_REASON_BACKTRACK_REQUIRED
```

el supervisor interpreta:

```text
la ruta hacia la próxima frontera empieza detrás del robot
```

La preparación física del giro 180 depende del frente actual:

```text
si hay pared frontal conocida:
    RUN_APPROACH_FRONT_WALL_FOR_PIVOT
    -> RUN_PIVOT_180
    -> RUN_ADVANCE

si el frente está abierto:
    RUN_CENTER_FRONT_TAPE_FOR_PIVOT
    -> RUN_PIVOT_180
    -> RUN_ADVANCE
```

La selección se hace en el supervisor, no en la policy.

Motivo:

```text
BACKTRACK_REQUIRED es una decisión lógica.
La forma física de preparar el pivot depende de la geometría actual.
```

---

## Dead-end

Un dead-end local ocurre cuando no hay salida abierta conocida en:

```text
frente
derecha
izquierda
```

En ese caso `app_find_cells_policy` no reporta `BACKTRACK_REQUIRED`, sino que deja caer al fallback local.

El fallback local (`App_Nav_RecommendAction`) devuelve:

```text
APP_NAV_ACTION_GO_BACK
```

y el supervisor ejecuta la secuencia clásica:

```text
RUN_APPROACH_FRONT_WALL_FOR_PIVOT
-> RUN_PIVOT_180
-> RUN_ADVANCE
-> DECIDE
```

Esto se mantiene separado de `BACKTRACK_REQUIRED`.

---

## Primitivas principales

### AdvanceAction

Avanza hasta que el sensor trasero confirma la siguiente cinta de frontera.

Perfiles traseros:

```c
APP_NAV_REAR_TAPE_PROFILE_NORMAL_CELL
APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL
APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL_AFTER_PIVOT
```

Secuencia en celda normal:

```text
cinta de entrada
-> blanco
-> cinta de salida
```

Secuencia al salir desde celda especial:

```text
cinta de entrada
-> blanco
-> cartulina especial
-> blanco
-> cinta de salida
```

Secuencia desde celda especial después de pivot:

```text
posible inicio sobre cartulina especial
-> blanco
-> cinta de salida
```

`AdvanceAction` actualiza pose lógica solo a través del supervisor, cuando termina correctamente.

---

### SmoothAction

Ejecuta un giro suave izquierda/derecha.

Condición lógica de finalización válida:

```text
DONE_REAR_TAPE
DONE_POST_YAW_REAR_TAPE
```

El smooth no debe considerarse completo solo por detectar pared/diagonal. La fase post-yaw continúa hasta confirmar cinta trasera. Durante `POST_YAW_SEEK_REAR_TAPE` no se corta por seguridad de pared frontal; la condición válida sigue siendo cinta trasera o timeout.

Flujo conceptual:

```text
TURNING
-> POST_YAW_SEEK_REAR_TAPE
-> DONE_POST_YAW_REAR_TAPE
```

o:

```text
TURNING
-> DONE_REAR_TAPE
```

Al terminar un smooth válido:

```text
App_Maze_UpdateRobotHeading(...)
App_Maze_AdvanceRobotPosition()
mapear/detectar especial si corresponde
```

---

### PivotAction

Pivote en celda.

En el supervisor se usa principalmente para:

```text
giro 180 de dead-end
giro 180 de backtracking hacia frontera
```

Al terminar:

```text
App_Maze_UpdateRobotHeading(TURN_AROUND)
```

Si está armado el latch:

```text
pivot_180_exit_requires_advance != 0
```

el supervisor inicia:

```text
RUN_ADVANCE
```

Esto hace que después del pivot 180 el robot salga de la celda y confirme la próxima cinta trasera antes de decidir de nuevo.

---

### ApproachFrontWallAction

Prepara un pivot 180 usando pared frontal.

Uso actual:

```text
dead-end clásico
BACKTRACK_REQUIRED con pared frontal conocida
```

Secuencia:

```text
avanzar con wall-follow/yaw-hold
-> terminar por distancia frontal objetivo
-> frenar
-> pivot 180
```

No actualiza mapa ni pose por sí misma.

---

### CenterByFrontTapeForPivotAction

Prepara un pivot 180 usando el sensor delantero de suelo.

Uso actual:

```text
BACKTRACK_REQUIRED con frente abierto
```

Secuencia:

```text
avanzar con wall-follow/yaw-hold
-> detectar flanco positivo válido de cinta frontal
-> frenar
-> pivot 180
```

No actualiza mapa ni pose por sí misma.

Reglas importantes:

```text
- no usa timer de ignore;
- no usa debounce por ticks adicional;
- no usa safety por pared frontal;
- usa AppNavPerception.floor_front_black;
- se apoya en la histéresis/percepción calculada una sola vez en el tick;
- si arranca en negro, espera salir a blanco antes de armar la detección;
- luego el próximo flanco positivo es la cinta límite.
```

Regla física fijada:

```text
En una celda NORMAL, cuando el sensor trasero está detectando negro en punto
de decisión, el sensor delantero no debe estar negro. Si aparece negro inicial,
no se acepta como DONE inmediato; primero se espera blanco y luego flanco.
```

En celda especial, si el sensor delantero está negro al inicio, se interpreta como cartulina especial y también se espera blanco antes de buscar la cinta límite.

---

## Detección de celdas especiales

Una celda especial se detecta al confirmar entrada a una celda.

Condición física base:

```text
floor_front_black == true
floor_rear_black  == true
```

La condición no se evalúa en cualquier tick, sino después de actualizar la posición lógica.

### Advance

```text
AdvanceAction DONE_REAR_TAPE
-> App_Maze_AdvanceRobotPosition()
-> si front_black && rear_black:
       App_Maze_MarkCurrentCellSpecial()
```

### Smooth

```text
SmoothAction DONE_REAR_TAPE o DONE_POST_YAW_REAR_TAPE
-> App_Maze_UpdateRobotHeading(turn)
-> App_Maze_AdvanceRobotPosition()
-> si front_black && rear_black:
       App_Maze_MarkCurrentCellSpecial()
```

`App_Maze_MarkCurrentCellSpecial()` devuelve true solo si la celda no estaba marcada antes.

Semántica por misión:

```text
FIND_CELLS:
    marca CELL_SPECIAL;
    si la celda era nueva, incrementa special_found_count;
    al llegar a 3 especiales, finaliza con FIND_CELLS_COMPLETE.

GO_A_TO_B:
    marca CELL_SPECIAL;
    no incrementa special_found_count;
    no finaliza por detectar especiales.
```

Esto permite que `GO_A_TO_B` ignore las cartulinas como objetivo de misión, pero las reconozca como condición física para usar perfiles de suelo correctos al salir de una celda especial.

---

## Salida desde celda especial

Si la celda actual está marcada como `CELL_SPECIAL`, el supervisor selecciona perfiles especiales al iniciar `AdvanceAction` o `SmoothAction`.

Caso general:

```text
APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL
```

Caso después de pivot 180 dentro de celda especial:

```text
APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL_AFTER_PIVOT
```

Esto evita confundir la cartulina central con la cinta de salida.

---

## Supervisor state/action

Estados principales:

```c
APP_NAV_SUPERVISOR_IDLE = 0
APP_NAV_SUPERVISOR_START_INITIAL_ADVANCE = 1
APP_NAV_SUPERVISOR_RUN_INITIAL_ADVANCE = 2
APP_NAV_SUPERVISOR_DECIDE = 3
APP_NAV_SUPERVISOR_RUN_ADVANCE = 4
APP_NAV_SUPERVISOR_RUN_APPROACH_FRONT_WALL_FOR_PIVOT = 5
APP_NAV_SUPERVISOR_RUN_SMOOTH_LEFT = 6
APP_NAV_SUPERVISOR_RUN_SMOOTH_RIGHT = 7
APP_NAV_SUPERVISOR_RUN_PIVOT_180 = 8
APP_NAV_SUPERVISOR_ERROR = 9
APP_NAV_SUPERVISOR_RUN_CENTER_FRONT_TAPE_FOR_PIVOT = 10
```

Acciones principales:

```c
APP_NAV_SUPERVISOR_ACTION_NONE = 0
APP_NAV_SUPERVISOR_ACTION_INITIAL_ADVANCE = 1
APP_NAV_SUPERVISOR_ACTION_ADVANCE = 2
APP_NAV_SUPERVISOR_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT = 3
APP_NAV_SUPERVISOR_ACTION_SMOOTH_LEFT = 4
APP_NAV_SUPERVISOR_ACTION_SMOOTH_RIGHT = 5
APP_NAV_SUPERVISOR_ACTION_PIVOT_180 = 6
APP_NAV_SUPERVISOR_ACTION_CENTER_FRONT_TAPE_FOR_PIVOT = 7
```

Importante:

```text
APP_NAV_SUPERVISOR_ERROR se mantiene en 9 por compatibilidad con HMI/debug.
El nuevo estado CENTER_FRONT_TAPE_FOR_PIVOT se agregó como 10.
```

---

## Result codes

Valores públicos de `last_result`:

```c
APP_NAV_SUPERVISOR_RESULT_OK = 0
APP_NAV_SUPERVISOR_RESULT_INVALID_ARGUMENT = 1
APP_NAV_SUPERVISOR_RESULT_START_FAILED = 2
APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR = 3
APP_NAV_SUPERVISOR_RESULT_UNSUPPORTED_ACTION = 4
APP_NAV_SUPERVISOR_RESULT_FIND_CELLS_COMPLETE = 5
APP_NAV_SUPERVISOR_RESULT_FIND_CELLS_INCOMPLETE_NO_FRONTIER = 6
APP_NAV_SUPERVISOR_RESULT_GO_TO_B_COMPLETE = 7
APP_NAV_SUPERVISOR_RESULT_GO_TO_B_INVALID_TARGET = 8
APP_NAV_SUPERVISOR_RESULT_GO_TO_B_NO_PATH = 9
```

Cuando una misión termina:

```text
active = 0
state = IDLE
action = NONE
last_result = resultado correspondiente
```

Interpretación de resultados específicos:

```text
FIND_CELLS_COMPLETE
    Se encontraron 3 celdas especiales únicas.

FIND_CELLS_INCOMPLETE_NO_FRONTIER
    No quedan fronteras alcanzables y faltan especiales.

GO_TO_B_COMPLETE
    La celda actual coincide con B.

GO_TO_B_INVALID_TARGET
    B no fue configurada o está fuera del mapa. No debe mover.

GO_TO_B_NO_PATH
    B era válida, pero las paredes descubiertas hacen imposible llegar.
```

---

## Debug supervisor / comando 0x9C

Comando:

```c
CMD_GET_SUPERVISOR_DEBUG_STATUS = 0x9C
```

Payload de 9 bytes:

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

La HMI muestra:

```text
Activo
Estado
Acción
Resultado
Pose
Celda
Especiales
```

El campo `maze_cell` contiene el byte completo con:

```text
WALL_* / CELL_VISITED / CELL_SPECIAL
```

---

## Publicación autónoma de estado supervisor / comando 0x9F

Además del pedido manual `CMD_GET_SUPERVISOR_DEBUG_STATUS = 0x9C`, el STM32 publica estado operativo hacia Qt:

```c
CMD_SUPERVISOR_STATUS_UPDATE = 0x9F
```

Payload de 9 bytes, igual al de `0x9C`:

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

Uso:

```text
Durante una corrida de supervisor:
    enviar cada 200 ms.

Al finalizar la misión:
    enviar una última actualización final.
```

Objetivo:

```text
permitir que la HMI Qt grafique posición, heading y celda actual
en tiempo real sin tener que pedir todo el mapa.
```

La sincronización completa por columnas sigue existiendo como herramienta secundaria/manual para resincronizar el mapa completo.

No se debe usar este comando para enviar telemetría pesada como sensores IR, yaw físico, PWM o datos de bajo nivel. Es estado operativo compacto de la misión.


---

## Flujo STM32 app_core

`app_core.c` hace de adaptador hardware:

```text
1. Lee sensores reales.
2. Construye un AppNavInput desde SensorSnapshotTypeDef con mediciones: ADC, IR, yaw/yaw-rate y dt.
3. Evalúa percepción portable una sola vez con App_Nav_EvaluatePerception(...).
4. Actualiza sensor_snapshot.detection_flags desde AppNavPerception.
5. Entrega el mismo par AppNavInput + AppNavPerception al supervisor o a primitive tests.
6. Ejecuta App_NavSupervisor_Tick(&input, &perception, &output) o primitive tests.
7. Aplica AppNavOutput a motores.
8. Expone telemetría/comandos UNERBUS.
```


Cuando se detiene navegación portable, deben pararse todas las primitivas:

```text
App_NavSupervisor_Stop()
App_Nav_StopAdvanceAction()
App_Nav_StopSmoothAction()
App_Nav_StopPivotAction()
App_Nav_StopApproachFrontWallAction()
App_Nav_StopCenterByFrontTapeForPivotAction()
```

---

## HMI y comandos relevantes

Comandos principales relacionados con mapa/supervisor:

```text
CMD_UPDATE_MAZE_CELL                 = 0x92
CMD_SYNC_MAZE_COLUMN                 = 0x93
CMD_PRIMITIVE_TEST                   = 0x95
CMD_SET_APPROACH_FRONT_WALL_TARGET   = 0x96
CMD_GET_APPROACH_FRONT_WALL_TARGET   = 0x97
CMD_SET_SUPERVISOR_INITIAL_POSE      = 0x98
CMD_GET_SUPERVISOR_INITIAL_POSE      = 0x99
CMD_START_SUPERVISOR_RUN             = 0x9A
CMD_STOP_SUPERVISOR_RUN              = 0x9B
CMD_GET_SUPERVISOR_DEBUG_STATUS      = 0x9C
CMD_SET_SUPERVISOR_GOAL_CELL         = 0x9D
CMD_GET_SUPERVISOR_GOAL_CELL         = 0x9E
CMD_SUPERVISOR_STATUS_UPDATE         = 0x9F
```

La HMI no debe reinterpretar coordenadas lógicas internamente como visuales. La conversión de Y se hace solo al dibujar.

`pageLaberinth` se entiende como panel operativo del STM32:

```text
- sincroniza mapa desde STM32;
- configura pose inicial;
- configura celda objetivo B para GO_A_TO_B;
- inicia/detiene run;
- muestra estado de supervisor;
- muestra mapa lógico recibido desde STM32;
- actualiza pose/celda en tiempo real con CMD_SUPERVISOR_STATUS_UPDATE.
```

Regla de coordenadas:

```text
HMI envía y recibe x/y lógicos STM32.
La inversión de Y solo ocurre al dibujar.
```

---

## Simulador y bridge

El simulador usa una copia sincronizada del firmware portable bajo `firmware_core`.

Cuando se agregan nuevos estados/acciones de supervisor, el bridge del simulador debe actualizar:

```text
- textos de state/action;
- whitelist de estados que permiten aplicar PWM;
- telemetría de supervisor;
- sincronización de firmware_core desde STM32.
```

Caso ya corregido conceptualmente:

```text
APP_NAV_SUPERVISOR_RUN_CENTER_FRONT_TAPE_FOR_PIVOT = 10
APP_NAV_SUPERVISOR_ACTION_CENTER_FRONT_TAPE_FOR_PIVOT = 7
```

Si el bridge no reconoce esos valores, puede mostrar:

```text
state=unknown
action=unknown
left_pwm=0
right_pwm=0
```

aunque el supervisor portable esté en `result=OK`.

---

## Casos corregidos importantes

### 1. FIND_CELLS local incompleto

Antes, `FIND_CELLS` usaba una recomendación local frente/derecha/izquierda/fallback. Ahora usa:

```text
vecinos no visitados
app_route_planner con BFS multi-source hacia frontera
backtracking con pivot 180 si hace falta
```

### 2. Smooth terminado por diagonal/pared

La detección diagonal/pared durante smooth no debe cerrar la acción lógica. Debe pasar a fase post-yaw y seguir hasta cinta trasera.

### 3. Celda especial en dead-end

Se mantiene perfil:

```text
APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL_AFTER_PIVOT
```

para no confundir cartulina central con cinta de salida después de pivot 180.

### 4. No frontier

Si ya no hay fronteras alcanzables y faltan especiales:

```text
FIND_CELLS_INCOMPLETE_NO_FRONTIER
```

No se debe caer a navegación local indefinida.

### 5. Backtracking abierto

Si la ruta hacia frontera empieza atrás:

```text
BACKTRACK_REQUIRED
```

El supervisor elige:

```text
front wall known -> ApproachFrontWallForPivot
front open       -> CenterByFrontTapeForPivot
```

### 6. Dead-end vs backtracking

Un dead-end puro no se reporta como `BACKTRACK_REQUIRED`. Se deja al fallback local para que use:

```text
APP_NAV_ACTION_GO_BACK
-> ApproachFrontWallForPivot
```

### 7. GO_A_TO_B optimista

`GO_A_TO_B` quedó implementado con el planner común en modo optimista:

```text
- resetea mapa al iniciar;
- usa A como pose inicial/current pose;
- usa B como destino configurado desde HMI;
- carga B como seed de app_route_planner;
- permite edges desconocidas;
- bloquea paredes conocidas;
- replanifica en cada DECIDE;
- finaliza con COMPLETE o NO_PATH.
```

### 8. Cartulinas en GO_A_TO_B

Las celdas especiales físicas permanecen en el laberinto aunque la misión no sea `FIND_CELLS`.

Corrección aplicada:

```text
GO_A_TO_B marca CELL_SPECIAL si detecta cartulina,
pero no incrementa el contador de especiales ni finaliza por ellas.
```

Esto permite que las primitivas posteriores usen perfiles especiales y no confundan cartulina central con cinta límite.

---

## Pendientes conocidos

### GO_A_TO_B

Implementado y validado inicialmente en Bluepill.

Pendientes posibles, no bloqueantes:

```text
- más pruebas de estrés con mapas largos;
- documentar casos de uso desde HMI si la UI sigue creciendo.
```

`app_route_planner.c/h` ya existe como planner común y es usado por `FIND_CELLS` y `GO_A_TO_B`.

### Validación real

`FIND_CELLS` y `GO_A_TO_B` tienen validación inicial correcta en Bluepill.

Puntos esperables de ajuste real futuro:

```text
umbrales de suelo
umbrales IR
velocidad base
target de approach a pared
comportamiento de cinta frontal
casos extremos de backtracking
rutas largas con varias cartulinas
```

### Documentación secundaria

Este README es la referencia del supervisor. Si existe una guía externa de `FIND_CELLS`, debe mantenerse alineada con este estado.

---

## Checklist de validación

Antes de considerar estable un cambio futuro:

```text
[ ] STM32 compila.
[ ] HMI compila si se tocaron textos/protocolo.
[ ] Simulador sincronizado compila si se usa como banco de prueba.
[ ] Start supervisor funciona.
[ ] Stop supervisor funciona.
[ ] App_Nav_EvaluatePerception(...) mantiene histéresis correcta de paredes/suelo.
[ ] sensor_snapshot.detection_flags se actualiza desde AppNavPerception.
[ ] FIND_CELLS_COMPLETE funciona al encontrar 3 especiales.
[ ] FIND_CELLS_INCOMPLETE_NO_FRONTIER funciona al agotar fronteras.
[ ] GO_A_TO_B con B inválida no mueve y reporta GO_TO_B_INVALID_TARGET.
[ ] GO_A_TO_B con A == B termina sin mover con GO_TO_B_COMPLETE.
[ ] GO_A_TO_B arranca con START_INITIAL_ADVANCE si A != B.
[ ] GO_A_TO_B llega a B al frente.
[ ] GO_A_TO_B llega a B con smooth.
[ ] GO_A_TO_B reporta GO_TO_B_NO_PATH si B queda inalcanzable.
[ ] GO_A_TO_B atraviesa celdas especiales sin confundir cartulina con cinta límite.
[ ] Dead-end clásico usa RUN_APPROACH_FRONT_WALL_FOR_PIVOT.
[ ] Backtracking con frente abierto usa RUN_CENTER_FRONT_TAPE_FOR_PIVOT.
[ ] Backtracking con pared frontal usa RUN_APPROACH_FRONT_WALL_FOR_PIVOT.
[ ] Después de pivot 180 se ejecuta ADVANCE antes de volver a DECIDE.
[ ] Celdas especiales se cuentan una sola vez en FIND_CELLS.
[ ] Celdas especiales se marcan pero no se cuentan en GO_A_TO_B.
[ ] CMD_SUPERVISOR_STATUS_UPDATE actualiza pose/celda en Qt cada 200 ms durante run.
[ ] Mapa sincronizado HMI mantiene coordenadas lógicas correctas.
```

---

## Regla de mantenimiento

Cambios futuros deben respetar:

```text
1. No poner lógica de misión en app_core.c.
2. No poner control de motores en app_nav_supervisor.c.
3. No hacer que app_find_cells_policy ni app_go_to_b_policy ejecuten primitivas.
4. No usar APP_NAV_ACTION_GO_BACK para BACKTRACK_REQUIRED.
5. GO_A_TO_B no debe arrancar directo en DECIDE si A != B; debe pasar por START_INITIAL_ADVANCE.
6. No modificar el byte de celda sincronizado sin revisar HMI/simulador.
7. No agregar heap, malloc ni recursión en planificación portable.
8. No hacer que app_route_planner devuelva acciones, reasons de misión o modifique el mapa.
9. No usar estructuras de debug como fuente operativa de percepción; usar AppNavPerception.
10. No reintroducir App_Nav_Tick/App_Nav_GetDebug/AppNavDebug como shell legacy.
11. No agregar telemetría pesada o temporal si no es necesaria para operación/debug real.
12. No agregar comportamiento shadow; si se implementa un modo, debe estar conectado o permanecer explícitamente no soportado.
13. Cuando se agregue un estado/action de supervisor:
      actualizar STM32,
      HMI,
      simulador/bridge,
      textos/debug,
      y whitelists de PWM si existen.
```
