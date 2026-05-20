#ifndef UNERBUSPARSER_H
#define UNERBUSPARSER_H

#include <QObject>
#include <QByteArray>
#include "Comunicacion/unerbus_protocol.h"

class UnerbusParser : public QObject
{
    Q_OBJECT

public:
    explicit UnerbusParser(QObject *parent = nullptr);
    void processData(const QByteArray &data);

signals:
    void packetReceived(quint8 command, const QByteArray &payload);
    void parsingError(const QString &error);

private:
    enum class State
    {
        WaitingForU,
        WaitingForN,
        WaitingForE,
        WaitingForR,
        WaitingForLength,
        WaitingForToken,
        ReadingPayloadAndChecksum
    };

    State currentState;
    QByteArray buffer;
    int expectedLength;
    quint8 calculatedChecksum;

    void reset();
};

#endif // UNERBUSPARSER_H