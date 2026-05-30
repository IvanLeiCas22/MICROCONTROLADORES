# Arquitectura de Comunicación – Proyecto STM32

## 1. Descripción General
El sistema implementa una arquitectura de comunicación robusta y modular para un vehículo autónomo basado en STM32. Soporta dos canales principales:
- **USB (CDC, comunicación con PC)**
- **WiFi (ESP01, UDP)**
Ambos canales utilizan el protocolo UNERBUS para el intercambio de datos estructurados.

## 2. Protocolo UNERBUS
### Estructura de Paquete
| HEADER (4B) | LENGTH (1B) | TOKEN (1B) | CMD (1B) | PAYLOAD (N) | CHECKSUM (1B) |
|------------|-------------|------------|----------|-------------|---------------|
| 'U','N','E','R' | Cantidad de bytes a enviar (CMD+PAYLOAD+CHECKSUM) | ':' | Código de comando | Datos útiles | XOR de todos los bytes anteriores |

- **HEADER:** Identifica el inicio del paquete.
- **LENGTH:** CMD + PAYLOAD + CHECKSUM.
- **TOKEN:** Constante ':' (0x3A).
- **CMD:** Código de comando.
- **PAYLOAD:** Datos útiles.
- **CHECKSUM:** XOR de todos los bytes anteriores, incluyendo HEADER.

### Flujo de Datos
- Los datos recibidos se almacenan en buffers circulares.
- El parser de UNERBUS valida la cabecera, longitud y checksum.
- Si el paquete es válido, invoca un callback para procesar el comando.
- Las respuestas se arman y transmiten usando el mismo protocolo.

## 3. Canales de Comunicación
### USB (CDC)
- **Recepción:**
  - Los datos recibidos por USB se pasan a `UNERBUS_ReceiveBuf`.
  - El parser procesa el buffer y ejecuta el callback de comando.
- **Transmisión:**
  - Las respuestas se colocan en el buffer de transmisión y se envían por USB.

### WiFi (ESP01, UDP)
- **Recepción:**
  - El ESP01 recibe datos por UDP y los envía por UART al STM32.
  - Cada byte recibido se pasa a `UNERBUS_ReceiveByte`.
- **Transmisión:**
  - Los datos pendientes se envían al ESP01, que los transmite por UDP.

## 4. Principales Funciones y Callbacks
- `UNERBUS_Init`: Inicializa el handler y los buffers.
- `UNERBUS_ReceiveByte` / `UNERBUS_ReceiveBuf`: Procesan datos recibidos.
- `UNERBUS_Task`: Decodifica paquetes y gestiona transmisión.
- `UNERBUS_Send`: Arma y transmite un paquete.
- `MyDataReady`: Callback invocado al recibir un paquete válido.

## 5. Robustez y Manejo de Errores
- **Buffers circulares**: Evitan bloqueos y pérdida de datos.
- **Timeouts**: Si la cabecera no se completa en tiempo, se reinicia el estado.
- **Checksum**: Garantiza la integridad de los paquetes.
- **Callbacks**: Permiten desacoplar la lógica de recepción y procesamiento de comandos.

## 6. Ejemplo de Envío de Mensaje
Supongamos que se quiere enviar el mensaje "Juan hoy comió bien" con CMD=0x01:

```c
const char mensaje[] = "Juan hoy comió bien";
uint8_t cmd = 0x01;
UNERBUS_WriteByte(&unerbusPC, cmd);
UNERBUS_Write(&unerbusPC, (uint8_t*)mensaje, sizeof(mensaje) - 1);
UNERBUS_Send(&unerbusPC, cmd, sizeof(mensaje));
```

## 6b. Ejemplo de Recepción y Procesamiento de un Nuevo Comando

Supongamos que se define un nuevo comando CMD=0x10 para recibir un mensaje de texto y procesarlo:

```c
#define CMD_MENSAJE_TEXTO 0x10

// Callback de recepción de comandos UNERBUS
void DecodeCMD(struct UNERBUSHandle *aBus, uint8_t iStartData) {
    uint8_t id = UNERBUS_GetUInt8(aBus); // Extrae el CMD
    switch(id) {
        case CMD_MENSAJE_TEXTO: {
            char buffer[32];
            // Extrae el payload recibido (por ejemplo, 20 bytes)
            UNERBUS_GetBuf(aBus, (uint8_t*)buffer, 20);
            buffer[20] = '\0'; // Asegura fin de cadena
            // Procesa el mensaje recibido
            ProcesarMensaje(buffer);
            break;
        }
        // ... otros comandos ...
    }
}
```

- El callback `DecodeCMD` es invocado automáticamente al recibir un paquete válido.
- Se extrae el comando y el payload usando las funciones de UNERBUS.
- Se recomienda validar la longitud del payload según el comando esperado.


## 6c. Comandos de supervisor y mapa lógico

Además de los comandos generales, el proyecto usa un conjunto de comandos UNERBUS para operar y visualizar el laberinto desde la HMI Qt.

### Comandos principales

| Comando | Dirección típica | Uso |
|---|---:|---|
| `CMD_UPDATE_MAZE_CELL` (`0x92`) | STM32 -> Qt | Actualiza una celda individual del mapa. |
| `CMD_SYNC_MAZE_COLUMN` (`0x93`) | STM32 -> Qt | Sincroniza una columna completa del mapa lógico. |
| `CMD_SET_SUPERVISOR_INITIAL_POSE` (`0x98`) | Qt -> STM32 | Configura pose inicial `A`: `x`, `y`, `heading`. |
| `CMD_GET_SUPERVISOR_INITIAL_POSE` (`0x99`) | Qt -> STM32 | Solicita pose inicial guardada. |
| `CMD_START_SUPERVISOR_RUN` (`0x9A`) | Qt -> STM32 | Inicia corrida del supervisor: `FIND_CELLS` o `GO_A_TO_B`. |
| `CMD_STOP_SUPERVISOR_RUN` (`0x9B`) | Qt -> STM32 | Detiene corrida del supervisor. |
| `CMD_GET_SUPERVISOR_DEBUG_STATUS` (`0x9C`) | Qt -> STM32 | Solicita estado compacto del supervisor. |
| `CMD_SET_SUPERVISOR_GOAL_CELL` (`0x9D`) | Qt -> STM32 | Configura celda objetivo `B` para `GO_A_TO_B`. |
| `CMD_GET_SUPERVISOR_GOAL_CELL` (`0x9E`) | Qt -> STM32 | Solicita celda objetivo `B` y flag de validez. |
| `CMD_SUPERVISOR_STATUS_UPDATE` (`0x9F`) | STM32 -> Qt | Publica estado compacto del supervisor de forma autónoma. |

### Payload de estado supervisor

`CMD_GET_SUPERVISOR_DEBUG_STATUS` y `CMD_SUPERVISOR_STATUS_UPDATE` usan el mismo payload compacto de 9 bytes:

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

`CMD_GET_SUPERVISOR_DEBUG_STATUS` es una consulta manual desde la HMI.

`CMD_SUPERVISOR_STATUS_UPDATE` es una publicación autónoma del STM32. Actualmente se envía cada 200 ms durante una corrida activa del supervisor y una vez más al finalizar la misión. Su objetivo es que Qt pueda graficar en tiempo real la pose lógica, heading y byte de celda actual sin pedir sincronización completa del mapa.

### Sincronización de mapa

La visualización en tiempo real debe usar preferentemente `CMD_SUPERVISOR_STATUS_UPDATE` para la celda/pose actual.

`CMD_SYNC_MAZE_COLUMN` sigue existiendo como mecanismo secundario para:

- sincronización manual completa;
- recuperación si la HMI se conectó tarde;
- verificación de consistencia del mapa.

Regla de coordenadas:

```text
STM32 envía coordenadas lógicas.
Qt guarda coordenadas lógicas.
Qt invierte Y solo al dibujar.
```

## 7. Extensibilidad y Recomendaciones
- Para agregar nuevos comandos, definir nuevos CMD y su lógica en el callback.
- Para nuevos canales, reutilizar la lógica de UNERBUS y adaptar el transporte físico.
- Para depuración, agregar logs en los callbacks y verificar el flujo de paquetes.

## 8. Referencias
- Ver archivos `UNERBUS.h` y `UNERBUS.c` para detalles de implementación.
- Ver integración con ESP01 en `ESP01.h` y `ESP01.c` para la capa WiFi. 