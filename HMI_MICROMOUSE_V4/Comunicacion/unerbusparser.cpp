#include "unerbusparser.h"
#include <QDebug>

UnerbusParser::UnerbusParser(QObject *parent)
    : QObject(parent)
{
    reset();
}

/**
 * @brief Procesa los datos crudos recibidos del puerto serie.
 * @param data QByteArray con los nuevos datos.
 */
void UnerbusParser::processData(const QByteArray &data)
{
    buffer.append(data);

    // Bucle principal para procesar el buffer.
    // Usamos 'continue' para re-evaluar el buffer después de encontrar un paquete completo.
    while (!buffer.isEmpty())
    {
        // CASO 1: Estamos esperando el payload y el checksum.
        if (currentState == State::ReadingPayloadAndChecksum)
        {
            // Verificamos si ya tenemos suficientes bytes para el paquete completo.
            if (buffer.size() >= expectedLength)
            {
                // Extraemos el paquete del buffer.
                QByteArray fullPacket = buffer.left(expectedLength);
                buffer.remove(0, expectedLength);

                // Descomponemos el paquete.
                quint8 command = fullPacket.at(0);
                QByteArray payload = fullPacket.mid(1, expectedLength - 2);
                quint8 receivedChecksum = fullPacket.back();

                // Verificamos el checksum.
                calculatedChecksum ^= command;
                for (char payloadByte : payload)
                {
                    calculatedChecksum ^= static_cast<quint8>(payloadByte);
                }

                if (calculatedChecksum == receivedChecksum)
                {
                    // ¡Éxito! Emitimos la señal.
                    emit packetReceived(command, payload);
                }
                else
                {
                    // Error de checksum.
                    emit parsingError(QString("Error de Checksum. Calculado: 0x%1, Recibido: 0x%2")
                                          .arg(calculatedChecksum, 2, 16, QChar('0'))
                                          .arg(receivedChecksum, 2, 16, QChar('0')));
                }

                // Reiniciamos la máquina de estados para buscar el siguiente paquete.
                reset();
                // Usamos 'continue' para que el bucle 'while' se ejecute de nuevo
                // por si hay más datos en el buffer.
                continue;
            }
            else
            {
                // No han llegado todos los datos del paquete, salimos y esperamos más.
                return;
            }
        }

        // CASO 2: Estamos buscando el encabezado, byte por byte.
        quint8 byte = buffer.at(0);
        buffer.remove(0, 1); // Consumimos el byte inmediatamente.

        switch (currentState)
        {
        case State::WaitingForU:
            if (static_cast<char>(byte) == Unerbus::HEADER[0])
            {
                currentState = State::WaitingForN;
                calculatedChecksum = byte; // Inicia el cálculo
            }
            break;

        case State::WaitingForN:
            if (static_cast<char>(byte) == Unerbus::HEADER[1])
            {
                currentState = State::WaitingForE;
                calculatedChecksum ^= byte;
            }
            else
            {
                reset();
            }
            break;

        case State::WaitingForE:
            if (static_cast<char>(byte) == Unerbus::HEADER[2])
            {
                currentState = State::WaitingForR;
                calculatedChecksum ^= byte;
            }
            else
            {
                reset();
            }
            break;

        case State::WaitingForR:
            if (static_cast<char>(byte) == Unerbus::HEADER[3])
            {
                currentState = State::WaitingForLength;
                calculatedChecksum ^= byte;
            }
            else
            {
                reset();
            }
            break;

        case State::WaitingForLength:
            expectedLength = byte;
            calculatedChecksum ^= byte;
            currentState = State::WaitingForToken;
            break;

        case State::WaitingForToken:
            if (static_cast<char>(byte) == Unerbus::TOKEN)
            {
                calculatedChecksum ^= byte;
                // Transición al estado de lectura de payload.
                // El bucle se encargará del resto en la siguiente iteración.
                currentState = State::ReadingPayloadAndChecksum;
            }
            else
            {
                reset();
            }
            break;

        case State::ReadingPayloadAndChecksum:
            // Este caso no debería alcanzarse aquí debido a la lógica
            // al principio de la función, pero por seguridad, reseteamos.
            reset();
            break;
        }
    }
}

/**
 * @brief Reinicia la máquina de estados a su estado inicial.
 */
void UnerbusParser::reset()
{
    currentState = State::WaitingForU;
    expectedLength = 0;
    calculatedChecksum = 0;
    // No limpiamos el buffer aquí, porque puede haber un paquete válido más adelante
}