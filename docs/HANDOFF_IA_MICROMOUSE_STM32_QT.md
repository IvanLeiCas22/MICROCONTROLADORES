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
    Ejecuta supervisor o primitive tests.
    Aplica PWM a motores.
    Atiende comandos UNERBUS/HMI.
    No debe absorber lógica de misión.

app_nav.c
    Percepción portable y primitivas físicas.
    Ejecuta acciones concretas:
        - Advance
        - Smooth turn
        - Pivot
        - Approach front wall
        - Center by front tape
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

## 4. Estado de diseño importante

### 4.1 Percepción portable

Modelo vigente:

```text
AppNavInput
    Mediciones del tick:
    - ADC filtrados;
    - distancias IR en mm;
    - yaw/yaw-rate;
    - dt.

AppNavPerception
    Interpretación filtrada:
    - floor_front_black;
    - floor_rear_black;
    - wall_front;
    - wall_left;
    - wall_right;
    - wall_diag_left;
    - wall_diag_right.
```

Regla obligatoria:

```text
App_Nav_EvaluatePerception() debe llamarse una sola vez por tick de control, desde app_core.c.
El mismo AppNavInput + AppNavPerception debe pasarse al supervisor o primitive tests.
Las primitivas y el supervisor no deben recalcular percepción.
```

No volver a introducir flags de piso negro dentro de `AppNavInput`.

No volver a copiar ADCs o distancias dentro de `AppNavPerception`.

### 4.2 Legacy eliminado

No reintroducir:

```text
App_Nav_Tick() legacy shell
App_Nav_GetDebug()
AppNavDebug
app_nav_debug.h
CMD_GET_NAV_DEBUG_STATUS = 0x94
nav_debug legacy de app_core.c
legacy braking controller
CMD_SET/GET_BRAKING_*
PID_ROLE_BRAKING
APP_NAV_SMOOTH_ACTION_FRONT_WALL_SAFETY
```

Si aparece una necesidad parecida, diseñarla explícitamente y justificarla. No restaurar legacy.

### 4.3 Approach front wall

Conservar:

```text
CMD_SET_APPROACH_FRONT_WALL_TARGET = 0x96
CMD_GET_APPROACH_FRONT_WALL_TARGET = 0x97
approach_front_wall_target_mm
```

La aproximación frontal para pivot es funcional. No confundirla con el braking controller eliminado.

---

## 5. Reglas de comportamiento de la IA

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
6. Confirmar el alcance.
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

Después de cambios que afecten navegación, el usuario debería probar como mínimo:

```text
- Compilación STM32.
- Compilación HMI Qt si se tocó HMI/protocolo.
- FIND_CELLS completo.
- GO_A_TO_B con ruta simple.
- GO_A_TO_B con replanificación por pared descubierta.
- Advance normal hasta cinta trasera.
- Smooth turn hasta cinta trasera.
- Backtracking con frente abierto.
- Backtracking con frente cerrado.
- Paso por celda especial.
- CenterByFrontTapeForPivot.
- ApproachFrontWallForPivot.
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

Evitar:

```text
- Refactors grandes sin necesidad.
- Reintroducir legacy eliminado.
- Mover lógica de misión a app_core.
- Mover HAL o protocolo a módulos portables.
- Duplicar fuentes de verdad.
- Evaluar percepción varias veces por tick.
- Agregar flags redundantes en AppNavInput.
- Agregar mediciones redundantes en AppNavPerception.
- Mezclar documentación, protocolo y lógica en un cambio si no corresponde.
- Cambiar payloads sin actualizar Qt y README_COMUNICACION.
- Decir que algo “funciona” sin prueba del usuario.
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
