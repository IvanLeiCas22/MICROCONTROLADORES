# HANDOFF OPERATIVO PARA IA — Micromouse STM32 + Qt

**Proyecto:** Autito micromouse real STM32 + HMI Qt  
**Objetivo de este handoff:** definir cómo debe comportarse una IA al trabajar sobre este proyecto.  
**Enfoque:** proceso de trabajo, entrada en contexto, análisis, parches, validación y límites.  
**Importante:** este archivo complementa la documentación técnica del repo. No reemplaza `README_NAV_SUPERVISOR.md` ni `README_COMUNICACION.md`.

---

## 1. Fuente de verdad

El usuario trabaja pasando snapshots del repo mediante Repomix.

Regla obligatoria:

```text
Usar siempre el Repomix de número más alto pasado en el chat actual.
No usar versiones anteriores si contradicen el repo actual.
Si el usuario pasa un Repomix nuevo, ese pasa a ser la fuente de verdad.
```

Si una tarea requiere modificar código y no hay un Repomix actualizado, la IA debe pedir el Repomix antes de proponer parches.

No asumir que el estado recordado de otro chat es más confiable que el repo actual.

---

## 2. Documentación que debe leerse primero

Para entrar en contexto, leer primero:

```text
docs/README_NAV_SUPERVISOR.md
MICROCONTROLADORES-STM32/Core/Inc/README_COMUNICACION.md
```

Después, leer solo los archivos relevantes según la tarea.

No hace falta releer todo el repo si el pedido es puntual, pero sí se debe ubicar correctamente la capa afectada antes de proponer cambios.

---

## 3. Arquitectura general que debe respetarse

Separación conceptual vigente:

```text
app_core.c
    Adaptador STM32/HAL/HMI.
    Lee sensores reales.
    Construye AppNavInput.
    Evalúa AppNavPerception una vez por tick.
    Ejecuta supervisor o adapta primitive tests hacia App_NavPrimitiveTest_*.
    Aplica PWM a motores.
    Atiende comandos UNERBUS/HMI.
    No debe absorber lógica de misión ni detalles internos de control.

app_nav.c
    Percepción portable y primitivas físicas.
    Ejecuta acciones concretas:
        - Advance
        - Smooth turn
        - Pivot
        - Approach front wall
        - Center by front tape
    Contiene el runner portable App_NavPrimitiveTest_* para pruebas manuales de primitivas.
    También contiene el fallback local determinístico App_Nav_RecommendAction(),
    con prioridad frente -> derecha -> izquierda -> atrás.
    La salida frontal conceptual es única: APP_NAV_ACTION_GO_FRONT.
    AdvanceAction decide internamente entre wall-follow y yaw-hold.
    Los controladores low-level son privados de app_nav.c.
    No decide la misión.

app_nav_supervisor.c
    Secuencia misiones.
    Consulta policies.
    Lanza y tickea primitivas.
    Actualiza pose lógica y mapa cuando corresponde.
    Detecta celdas especiales.
    No debe hacer control directo de bajo nivel ni HAL.

app_find_cells_policy.c
    Decide acciones conceptuales de FIND_CELLS.
    No ejecuta primitivas.

app_go_to_b_policy.c
    Decide acciones conceptuales de GO_A_TO_B.
    No ejecuta primitivas.

app_route_planner.c
    BFS/flood fill común.
    Calcula distancias.
    No decide acciones.
    No mueve motores.
    No modifica mapa.

app_maze.c
    Mapa lógico, paredes, known_edges, pose, celdas visitadas y especiales.

HMI_MICROMOUSE_V4/
    Interfaz Qt.
    No debe contener lógica portable de navegación.
```

Regla rápida:

```text
¿Es decisión de misión?                 -> policy
¿Es secuencia de primitivas?            -> supervisor
¿Es movimiento/control físico?          -> app_nav
¿Es mapa/pose/paredes?                  -> app_maze
¿Es cálculo de distancias/ruta?         -> app_route_planner
¿Es hardware, HMI, protocolo o PWM?     -> app_core / Qt
¿Es visualización/interfaz?             -> Qt
```

---

## 4. Invariantes críticos del diseño actual

Esta sección no reemplaza la documentación técnica. Resume restricciones de diseño que una IA no debe romper al proponer cambios.

### 4.1 Percepción portable

Reglas obligatorias:

```text
App_Nav_EvaluatePerception() debe llamarse una sola vez por tick de control, desde app_core.c.
El mismo AppNavInput + AppNavPerception debe pasarse al supervisor o al runner portable de primitive tests.
Las primitivas y el supervisor no deben recalcular percepción.
No volver a introducir flags de piso negro dentro de AppNavInput.
No volver a copiar ADCs o distancias dentro de AppNavPerception.
```

### 4.2 Configuración runtime de navegación

Reglas obligatorias:

```text
AppNavConfig es la fuente de verdad runtime para configuración de navegación.
app_nav_config.h define los defaults vivos de navegación.
app_nav.c mantiene la instancia activa de configuración.
app_core.c no mantiene variables runtime legacy duplicadas de navegación.
app_config.h no debe volver a contener defaults runtime de navegación.
```

Los comandos HMI/UNERBUS que leen o escriben configuración de navegación deben pasar por:

```text
App_Nav_GetConfig()
App_Nav_SetConfig()
```

### 4.3 Primitive tests portables

Flujo vigente:

```text
HMI / CMD_PRIMITIVE_TEST
-> app_core.c adapta protocolo, seguridad y estado HMI
-> App_NavPrimitiveTest_Start(...)
-> cada tick: App_NavPrimitiveTest_Tick(input, perception, output)
-> app_core.c aplica AppNavOutput a motores y publica status
```

Reglas obligatorias:

```text
El runner portable actual vive en app_nav.h/c.
Los primitive tests deben ejecutar acciones completas, no controladores low-level.
app_core.c no debe llamar controladores internos como StartSmoothTurn, ComputeSmoothTurnPwm, StartPivotTurn, ComputePivotTurnPwm, WallFollow o YawHold directo.
No reexponer controladores low-level en app_nav.h para resolver primitive tests.
Si se agregan tests de Advance/Pivot/Approach/Center, agregarlos a App_NavPrimitiveTest_*.
```

### 4.4 Legacy eliminado

No reintroducir patrones legacy ya eliminados:

```text
- shell global App_Nav_Tick/AppNavDebug/app_nav_debug.h;
- debug nav paralelo como fuente de verdad;
- braking controller legacy y comandos asociados;
- configuración runtime duplicada fuera de AppNavConfig;
- parámetro random_value en App_Nav_RecommendAction();
- acciones frontales duplicadas para avanzar al frente;
- primitive tests que llamen controladores low-level desde app_core.c;
- controladores low-level públicos en app_nav.h si solo los usa app_nav.c.
```

Si aparece una necesidad parecida, diseñarla explícitamente en la capa correcta y justificarla. No restaurar código eliminado solo porque parece resolver rápido el problema.

### 4.5 Approach front wall

La aproximación frontal para pivot es funcional. No confundirla con el braking controller eliminado.

Conservar su configuración/protocolo mientras siga vigente:

```text
CMD_SET_APPROACH_FRONT_WALL_TARGET = 0x96
CMD_GET_APPROACH_FRONT_WALL_TARGET = 0x97
approach_front_wall_target_mm
```

---

## 5. Reglas de comportamiento de la IA

### 5.1 Conducta general

La IA debe trabajar de forma conservadora:

```text
- Analizar antes de implementar.
- Hacer cambios pequeños, verificables y definitivos.
- No hacer refactors grandes por inercia.
- No agregar código temporal.
- No dejar comportamiento shadow salvo decisión explícita.
- No agregar telemetría basura.
- Eliminar legacy muerto si ya no se usa.
- No mezclar cambios independientes.
- No tocar HMI/protocolo si el cambio es interno portable.
- No afirmar que compiló/probó salvo que el usuario lo confirme.
```

### 5.2 Reutilización equilibrada

```text
- Reutilizar código existente cuando sea razonable, especialmente primitivas, helpers portables, APIs públicas y lógica ya validada.
- Antes de crear una función nueva, revisar si ya existe una función equivalente o una abstracción cercana en la capa correcta.
- No duplicar lógica funcional por comodidad si puede reutilizarse sin romper responsabilidades.
- No forzar reutilización si vuelve el código menos claro, menos eficiente, más acoplado o mezcla capas.
- No crear abstracciones genéricas grandes solo para evitar pocas líneas duplicadas.
- Duplicar localmente una lógica simple puede ser aceptable si evita acoplamiento artificial o abstracciones prematuras.
```

### 5.3 Conciencia STM32/Bluepill

```text
- Tener siempre presente que el firmware corre en STM32 Bluepill con recursos limitados.
- Evitar malloc/free, recursión, buffers grandes en stack, float/double innecesarios y abstracciones que aumenten RAM/Flash sin beneficio claro.
- Si una solución nueva agrega tablas, workspaces, telemetría o estructuras persistentes, justificar brevemente su costo y su necesidad.
- Preferir lógica portable simple, determinística y de bajo costo.
```

### 5.4 Control de alcance

```text
- No incluir mejoras oportunistas dentro de un patch cuyo objetivo era otro.
- Si durante el análisis aparece una limpieza, bug potencial o refactor conveniente pero no necesario, mencionarlo aparte como hallazgo.
- Solo incluirlo en los parches si el usuario lo aprueba o si es requisito directo para resolver el objetivo actual.
```

### 5.5 Validación y límites de la IA

```text
- Distinguir siempre entre revisión estática, compilación, prueba en simulador y prueba en hardware real.
- No tratar una prueba en simulador como validación completa del robot físico.
- Si una lógica depende de sensores reales, ruido, tiempos o dinámica física, marcarla como pendiente de validación en hardware aunque el simulador funcione.
```

El usuario compila y prueba. La IA puede hacer revisión estática, pero no debe presentar eso como validación completa del proyecto STM32/Qt.

---

## 6. Pipeline recomendado de trabajo

Para cualquier cambio de código:

```text
1. Entender el objetivo del usuario.
2. Identificar archivos/capas afectadas.
3. Revisar el Repomix más actualizado.
4. Hacer análisis específico antes de parches.
5. Separar:
      - cambios necesarios;
      - cambios opcionales;
      - cambios que conviene evitar.
6. Delimitar el alcance; pedir confirmación solo si hay ambigüedad, riesgo arquitectónico o alternativas razonables.
7. Generar un .patch por archivo modificado.
8. Entregar comandos:
      git apply --check ...
      git apply ...
9. El usuario compila y prueba.
10. El usuario pasa repo actualizado.
11. La IA verifica que los cambios quedaron bien aplicados.
12. Si todo está correcto, sugerir nombre de commit.
```

No saltar directamente a parches si el cambio toca varias capas o tiene riesgo arquitectónico.

---

## 7. Reglas para parches

Formato esperado:

```text
Un archivo .patch por cada archivo modificado.
No pegar diffs largos en el chat.
Entregar los .patch como archivos descargables.
Si son varios, entregar también un .zip.
```

Incluir comandos de aplicación:

```bat
git apply --check patch1.patch patch2.patch patch3.patch
git apply patch1.patch patch2.patch patch3.patch
```

Si un patch borra un archivo completo y falla por fin de línea/whitespace, proponer alternativa con `git rm`, pero solo si corresponde.

No pedir a Codex u otra IA “diff completo” en texto si eso puede contaminar el contexto. Preferir archivos `.patch`.

Formato de código en parches:

```text
El repositorio define el estilo C/C++ mediante .clang-format en la raíz.
Toda modificación de código STM32 o Qt debe respetar ese formato.
No reintroducir alineaciones verticales manuales, cortes innecesarios de argumentos/parámetros ni estilos inconsistentes con .clang-format.
Si un cambio requiere reformateo amplio, debe proponerse como commit separado de cualquier cambio funcional.
```

---

## 8. Reglas para protocolo/HMI

Si se toca protocolo UNERBUS, revisar y actualizar en conjunto:

```text
MICROCONTROLADORES-STM32/Core/Inc/app_config.h
MICROCONTROLADORES-STM32/Core/Src/app_core.c
MICROCONTROLADORES-STM32/Core/Inc/README_COMUNICACION.md
HMI_MICROMOUSE_V4/Comunicacion/unerbus_protocol.h
HMI_MICROMOUSE_V4/mainwindow.cpp
HMI_MICROMOUSE_V4/mainwindow.h
HMI_MICROMOUSE_V4/mainwindow.ui
```

Si cambia un comando, payload o enum compartido, debe actualizarse firmware + HMI + documentación.

El mapa lógico actual representa el laberinto físico real 8x8:

```text
MAZE_WIDTH  = 8
MAZE_HEIGHT = 8
coordenadas válidas: x/y = 0..7
pose inicial por defecto: (0, 0), HEADING_NORTH
```

La HMI configura pose inicial A y destino B dentro de ese rango. No reintroducir una matriz 15x15 ni un arranque artificial en el centro salvo decisión explícita.

No reutilizar IDs de comandos eliminados sin decisión explícita.

---

## 9. Reglas para navegación portable

No romper estas responsabilidades:

```text
app_route_planner
    Solo calcula distancias.
    No devuelve acciones de misión.
    No modifica mapa.
    No mueve motores.

policies
    Deciden acciones conceptuales.
    No ejecutan primitivas.
    No modifican hardware.

supervisor
    Secuencia acciones.
    Actualiza mapa/pose después de eventos físicos confirmados.
    No implementa control PWM bajo nivel.

app_nav
    Ejecuta primitivas físicas y percepción.
    No decide FIND_CELLS ni GO_A_TO_B.

app_core
    Adapta hardware/HMI.
    No decide misión ni BFS.
```

---

## 10. Validación esperada

La validación sugerida debe ser proporcional al alcance del cambio. No todos los cambios requieren probar todo el flujo.

Validación base:

```text
- Compilación STM32.
- Compilación HMI Qt solo si se tocó HMI/protocolo.
- Prueba funcional mínima de la zona afectada.
```

Si se tocó una primitiva:

```text
- Primitive test correspondiente, si existe.
- Caso nominal de terminación.
- Caso de seguridad/error si aplica.
```

Si se tocó supervisor o policies:

```text
- FIND_CELLS.
- GO_A_TO_B con ruta simple.
- Replanificación solo si se tocó ruta, mapa, paredes o GO_A_TO_B.
```

Si se tocó mapa o celdas especiales:

```text
- Paso por celda especial.
- Verificar que no haya falsa detección de cinta límite posterior.
```

Si se tocó backtracking, pivots o preparación de pivots:

```text
- Backtracking con frente abierto.
- Backtracking con frente cerrado.
- CenterByFrontTapeForPivot o ApproachFrontWallForPivot según corresponda.
```

La IA puede sugerir pruebas, pero el usuario es quien confirma resultados.

---

## 11. Cómo responder ante dudas

Si la IA no está segura:

```text
- No inventar.
- Revisar el Repomix.
- Citar el archivo/fragmento relevante si corresponde.
- Proponer análisis específico antes de parches.
- Separar hipótesis de hechos.
```

Si el usuario pide “¿necesitas análisis específico?”:

```text
Responder directamente sí/no.
Si sí, explicar qué se analizará.
```

Si el usuario pide parches:

```text
Crear archivos .patch.
No escribir el diff completo en el chat.
```

---

## 12. Qué evitar

Evitar estos anti-patrones específicos del proyecto:

```text
- Mover lógica de misión a app_core.
- Mover HAL o protocolo a módulos portables.
- Duplicar fuentes de verdad.
- Evaluar percepción varias veces por tick.
- Agregar flags redundantes en AppNavInput.
- Agregar mediciones redundantes en AppNavPerception.
- Mezclar documentación, protocolo y lógica en un cambio si no corresponde.
- Cambiar payloads sin actualizar Qt y README_COMUNICACION.
```

---

## 13. Reglas de documentación

Actualizar documentación si cambia:

```text
- API pública.
- Flujo de percepción.
- Protocolo UNERBUS.
- Payloads HMI.
- Estados/actions/results del supervisor.
- Separación de responsabilidades.
- Reglas de navegación.
```

Documentos principales:

```text
docs/README_NAV_SUPERVISOR.md
MICROCONTROLADORES-STM32/Core/Inc/README_COMUNICACION.md
```

No documentar detalles internos triviales si no ayudan al mantenimiento.

---

## 14. Resumen operativo para IA

Antes de actuar, recordar:

```text
El objetivo no es escribir mucho código.
El objetivo es mantener una arquitectura entendible, portable y verificable.

Primero analizar.
Después delimitar.
Después parchear por archivo.
Después esperar prueba del usuario.
Después sugerir commit.
```
