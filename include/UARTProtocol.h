#ifndef UARTPROTOCOL_H
#define UARTPROTOCOL_H

#include <Arduino.h>
#define PROTOCOL_DEBUG_LOGS 0
// Enable or disable debugging

#if PROTOCOL_DEBUG_LOGS
#define PROTOCOL_DEBUG_PRINT(...)  \
    do                             \
    {                              \
        Serial.print(__VA_ARGS__); \
    } while (0)
#define PROTOCOL_DEBUG_PRINTLN(...)  \
    do                               \
    {                                \
        Serial.println(__VA_ARGS__); \
    } while (0)
#else
#define PROTOCOL_DEBUG_PRINT(...)
#define PROTOCOL_DEBUG_PRINTLN(...)
#endif

class UARTProtocol
{
private:
    uint8_t header;
    uint8_t maxPacketSize;
    unsigned long baudRate;
    HardwareSerial &serial;

public:
    /**
     * @brief Constructs a UARTProtocol object.
     * @param serialPort The HardwareSerial object to use for communication.
     * @param headerByte The byte that marks the start of a packet.
     * @param maxPacketSize The maximum size of a packet in bytes.
     * @param baudRate The baud rate at which to communicate.
     */
    UARTProtocol(HardwareSerial &serialPort, uint8_t headerByte, uint8_t maxPacketSize, unsigned long baudRate);
    /**
     * @brief Initializes the UART protocol with the specified configuration.
     */
    void begin();
    /**
     * @brief Sends a command over UART.
     * @param commandType The type of command to send.
     */
    void SendCommand(uint8_t commandType);
    /**
     * @brief Sends data over UART.
     * @param data Pointer to the data array to send.
     * @param length Length of the data array.
     */
    void SendData(byte *data, uint8_t length);
    /**
     * @brief Reads a command from UART with a timeout.
     * @param commandType Reference to store the received command type.
     * @param timeout Timeout duration in milliseconds.
     * @return True if a command was successfully read, false otherwise.
     */
    bool ReadCommand(uint8_t &commandType, uint32_t timeoutMs = 1000);
    /**
     * @brief Reads data from UART with a timeout.
     * @param data Pointer to store the received data.
     * @param length Length of the data to read.
     * @param timeout Timeout duration in milliseconds.
     * @return True if data was successfully read, false otherwise.
     */
    bool ReadData(byte *data, uint8_t length, int timeout);
    /**
     * @brief Reads data from UART.
     * @param data Pointer to store the received data.
     * @param length Length of the data to read.
     * @return True if data was successfully read, false otherwise.
     */
    bool ReadData(byte *data, uint8_t length);
    bool IsAvailable();
};

#endif // UARTPROTOCOL_H