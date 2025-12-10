/*
Author: Logan Wright

Description: Protocol tools for communication between applications and avionics
*/

#pragma once

#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

#define PACKET_BUFFER_SIZE 1024

namespace XProtocol
{

/// @brief Data structure for grid fin commands
struct GridFinCommand
{
    double thetaInRadians = 0.0;
};

/// @brief Data structure for tvc (engine) commands
struct TvcCommand
{
    double thrustInPercent = 0.0, thetaXInRadians = 0.0, thetaYInRadians = 0.0;
};

}  // namespace XProtocol

enum XRocketPayloadType : uint8_t
{
    TELEMETRY = 0,
    COMMAND,
    UNKNOWN
};

/// @brief Base class for payload types
class XRocketPayload
{
   public:
    virtual ~XRocketPayload()
    {
    }

    /// @brief Gets payload type
    /// @return
    virtual XRocketPayloadType
    GetType() const
    {
        return XRocketPayloadType::UNKNOWN;
    }

    /// @brief Packs payload into buffer
    /// @param buffer
    virtual void
    Pack(std::vector<std::byte>&)
    {
    }

    /// @brief Unpacks data into payload object
    /// @param data
    /// @param size
    virtual void
    Unpack(const std::byte*, std::size_t)
    {
    }
};

/// @brief Telemetry payload for XRocket protocol
class XRocketTelemetryPayload : public XRocketPayload
{
   public:
    /// @brief Minimal effort constructor
    XRocketTelemetryPayload()
    {
    }

    /// @brief Basic constructor for a telemetry payload
    /// @param acc must be 3-component vector
    /// @param gyro must be 3-component vector
    /// @param mag must be 3-component vector
    /// @param baro
    /// @param lat
    /// @param lon
    /// @param alt
    XRocketTelemetryPayload(const std::vector<double> acc,
                            const std::vector<double> gyro,
                            const std::vector<double> mag,
                            const double baro,
                            const double lat,
                            const double lon,
                            const double alt)
        : xAcc(acc),
          xGyro(gyro),
          xMag(mag),
          xBaro(baro),
          xLat(lat),
          xLon(lon),
          xAlt(alt)
    {
    }

    /// @brief Gets payload type
    /// @return
    XRocketPayloadType
    GetType() const override
    {
        return XRocketPayloadType::TELEMETRY;
    }

    /// @brief Packs this object into buffer
    /// @param buffer
    void
    Pack(std::vector<std::byte>& buffer) override
    {
        auto AppendToBuffer = [&](const auto& v)
        {
            const std::byte* p = reinterpret_cast<const std::byte*>(&v);
            buffer.insert(buffer.end(), p, p + sizeof(v));
        };

        // Stack accelerometer readings
        AppendToBuffer(xAcc[0]);
        AppendToBuffer(xAcc[1]);
        AppendToBuffer(xAcc[2]);

        // Stack gyroscope readings
        AppendToBuffer(xGyro[0]);
        AppendToBuffer(xGyro[1]);
        AppendToBuffer(xGyro[2]);

        // Stack magnetometer readings
        AppendToBuffer(xMag[0]);
        AppendToBuffer(xMag[1]);
        AppendToBuffer(xMag[2]);

        // Stack barometer reading
        AppendToBuffer(xBaro);

        // Stack GPS readings
        AppendToBuffer(xLat);
        AppendToBuffer(xLon);
        AppendToBuffer(xAlt);
    }

    /// @brief Unpacks data into this object
    /// @param data
    /// @param size
    void
    Unpack(const std::byte* data, const std::size_t size) override
    {
        if (size < sizeof(xBaro) * 13)
            std::cout << "TelemetryPayload size too small!\n";

        auto ReadToVariable = [&](auto& v, std::size_t& offset)
        {
            std::memcpy(&v, data + offset, sizeof(v));
            offset += sizeof(v);
        };

        std::size_t offset = 0;

        // Stack accelerometer readings
        ReadToVariable(xAcc[0], offset);
        ReadToVariable(xAcc[1], offset);
        ReadToVariable(xAcc[2], offset);

        // Stack gyroscope readings
        ReadToVariable(xGyro[0], offset);
        ReadToVariable(xGyro[1], offset);
        ReadToVariable(xGyro[2], offset);

        // Stack magnetometer readings
        ReadToVariable(xMag[0], offset);
        ReadToVariable(xMag[1], offset);
        ReadToVariable(xMag[2], offset);

        // Stack barometer reading
        ReadToVariable(xBaro, offset);

        // Stack GPS readings
        ReadToVariable(xLat, offset);
        ReadToVariable(xLon, offset);
        ReadToVariable(xAlt, offset);
    }

    /// @brief Get accelerometer reading
    /// @return
    const std::vector<double>&
    GetAccelerometerReading() const
    {
        return xAcc;
    }

    /// @brief Get gyro reading
    /// @return
    const std::vector<double>&
    GetGyroscopeReading() const
    {
        return xGyro;
    }

    /// @brief Get magnetometer reading
    /// @return
    const std::vector<double>&
    GetMagnetometerReading() const
    {
        return xMag;
    }

    /// @brief Get barometer reading
    /// @return
    double
    GetBarometerReading() const
    {
        return xBaro;
    }

    /// @brief Get latitude reading
    /// @return
    double
    GetLatitudeReading() const
    {
        return xLat;
    }

    /// @brief Get longitude reading
    /// @return
    double
    GetLongitudeReading() const
    {
        return xLon;
    }

    /// @brief Get altitude reading
    /// @return
    double
    GetAltitudeReading() const
    {
        return xAlt;
    }

   private:
    std::vector<double> xAcc{ 3, 0.0 }, xGyro{ 3, 0.0 }, xMag{ 3, 0.0 };
    double xBaro = 0.0, xLat = 0.0, xLon = 0.0, xAlt = 0.0;
};

/// @brief Payload object for commands
class XRocketCommandPayload : public XRocketPayload
{
   public:
    /// @brief Generic constructor for minimal effort
    XRocketCommandPayload() : xGridFinCommands{}, xTvcCommands{}
    {
        xGridFinCommands.resize(4);
        xTvcCommands.resize(0);
    }

    /// @brief Simple constructor from commands
    /// @param gridFinCommands
    /// @param tvcCommands
    XRocketCommandPayload(
        const std::vector<XProtocol::GridFinCommand> gridFinCommands,
        const std::vector<XProtocol::TvcCommand> tvcCommands)
        : xGridFinCommands(gridFinCommands), xTvcCommands(tvcCommands)
    {
    }

    /// @brief Gets payload type
    /// @return
    XRocketPayloadType
    GetType() const override
    {
        return XRocketPayloadType::COMMAND;
    }

    /// @brief Packs object into buffer
    /// @param buffer
    void
    Pack(std::vector<std::byte>& buffer) override
    {
        // Define function to handle buffer append operation
        auto AppendToBuffer = [&](auto& v)
        {
            const std::byte* p = reinterpret_cast<std::byte*>(&v);
            buffer.insert(buffer.end(), p, p + sizeof(v));
        };

        // Always 4 grid fin commands
        uint32_t count = xGridFinCommands.size();
        const std::byte* gridFinCount = reinterpret_cast<std::byte*>(&count);

        buffer.insert(buffer.end(), gridFinCount, gridFinCount + sizeof(count));

        for (auto& gridFinCommand : xGridFinCommands)
        {
            AppendToBuffer(gridFinCommand.thetaInRadians);
        }

        // Then repeat for N-Tvc commands
        count = xTvcCommands.size();
        const std::byte* tvcCount = reinterpret_cast<std::byte*>(&count);

        buffer.insert(buffer.end(), tvcCount, tvcCount + sizeof(count));

        for (auto& tvcCommand : xTvcCommands)
        {
            AppendToBuffer(tvcCommand.thrustInPercent);
            AppendToBuffer(tvcCommand.thetaXInRadians);
            AppendToBuffer(tvcCommand.thetaYInRadians);
        }
    }

    /// @brief Unpacks data to object
    /// @param data
    /// @param size
    void
    Unpack(const std::byte* data, const std::size_t size) override
    {
        // Create function to unpack data from buffer and write to variables
        auto ReadToVariable = [&](auto& v, std::size_t& offset)
        {
            std::memcpy(&v, data + offset, sizeof(v));
            offset += sizeof(v);
        };

        // Validate minimum count size to unpack
        if (size < sizeof(uint32_t))
            std::cout <<
                "CommandPayload missing gridfin command count!\n";

        // Unpack gridfin commands
        uint32_t count;
        std::memcpy(&count, data, sizeof(count));

        std::size_t expected =
            sizeof(count) + count * sizeof(XProtocol::GridFinCommand);
        if (size < expected)
            std::cout << "CommandPayload missing commands!\n";

        // Debug message
        std::cout << "Unpacked " << count << " gridfins!\n";

        xGridFinCommands.resize(count);

        std::size_t offset = sizeof(count);

        for (auto& gridFinCommand : xGridFinCommands)
        {
            ReadToVariable(gridFinCommand.thetaInRadians, offset);
        }

        // Then unpack what's left as engines
        expected += sizeof(uint32_t);

        if (size < expected)
            std::cout <<
                "CommandPayload missing engine command count!\n";

        // Copy data to new count variable with offset
        std::memcpy(&count, data + offset, sizeof(count));
        offset += sizeof(count);

        // Debug message
        std::cout << "Unpacked " << count << " engines!\n";

        xTvcCommands.resize(count);

        for (uint32_t i = 0; i < count; i++)
        {
            ReadToVariable(xTvcCommands[i].thrustInPercent, offset);
            ReadToVariable(xTvcCommands[i].thetaXInRadians, offset);
            ReadToVariable(xTvcCommands[i].thetaYInRadians, offset);
        }
    }

    /// @brief Get read-only reference to grid fins commands
    /// @return
    const std::vector<XProtocol::GridFinCommand>&
    GetGridFinCommands() const
    {
        return xGridFinCommands;
    }

    /// @brief Get read-only reference to tvc commands
    /// @return
    const std::vector<XProtocol::TvcCommand>&
    GetTvcCommands() const
    {
        return xTvcCommands;
    }

   private:
    std::vector<XProtocol::GridFinCommand> xGridFinCommands;
    std::vector<XProtocol::TvcCommand> xTvcCommands;
};

/*-----------------------------------*/
/* XROCKET PROTOCOL PACKET STRUCTURE */
/*-----------------------------------*/

class XRocketPacket
{
   public:
    /// @brief Bad constructor, just for quick object declarations
    XRocketPacket() : xPayload(nullptr)
    {
    }

    /// @brief Constructor for connecting a payload to a packet
    /// @param payload
    XRocketPacket(std::unique_ptr<XRocketPayload> payload)
        : xPayload(std::move(payload))
    {
        xType = xPayload->GetType();
    }

    /// @brief Constructor for packet object from raw byte array
    /// @param data
    /// @param size
    XRocketPacket(const std::byte* data, const std::size_t size)
    {
        // Initialize offset to 0
        std::size_t offset = 0;

        // Also initialize expeceted size to timestamp for now
        std::size_t expected = sizeof(xTimeStampInMicroSeconds);

        // Verify size for timestamp
        if (size < expected)
            std::cout << "Data array missing TimeStamp bytes!\n";

        // Copy timestamp data
        std::memcpy(
            &xTimeStampInMicroSeconds, data, sizeof(xTimeStampInMicroSeconds));
        offset += sizeof(xTimeStampInMicroSeconds);

        // Increment by only 1 for uint8_t (payload type byte)
        expected += 1;

        if (size < expected)
            std::cout << "Data array missing PayloadType byte!\n";

        // Copy payload type byte
        std::memcpy(&xType, data + offset, 1);
        offset += 1;

        // Increment by sizeof(uint32_t) bytes for payload length bytes
        expected += sizeof(uint32_t);

        if (size < expected)
            std::cout << "Data array missing PayloadLength bytes!\n";

        // Capture size of payload to let the payload unpack itself
        uint32_t payloadLength;
        std::memcpy(&payloadLength, data + offset, sizeof(payloadLength));
        offset += sizeof(payloadLength);

        // Increment expected by the payload length for the full packet size
        expected += payloadLength;

        // Final check to confirm size of the data
        if (size < expected)
            std::cout <<
                "Data array does not have enough payload bytes!\n";

        // Initialize the smart pointer for payload
        switch (xType)
        {
            case XRocketPayloadType::COMMAND:
                xPayload = std::make_unique<XRocketCommandPayload>();
                break;

            case XRocketPayloadType::TELEMETRY:
                xPayload = std::make_unique<XRocketTelemetryPayload>();
                break;

            default:
                // This is the base class with no real implementations
                xPayload = std::make_unique<XRocketPayload>();
                break;
        }

        // Let payload unpack data
        xPayload->Unpack(data + offset, payloadLength);
    }

    /// @brief Packs this object and returns a buffer array of bytes
    /// @return
    std::vector<std::byte>
    Pack()
    {
        // Create empty buffer
        std::vector<std::byte> buffer;

        // Pack payload
        std::vector<std::byte> payloadBuffer;
        xPayload->Pack(payloadBuffer);

        // Capture payload size
        uint32_t payloadSize = payloadBuffer.size();

        // Resize buffer: timestamp -> payload type -> payload length -> payload
        buffer.reserve(sizeof(xTimeStampInMicroSeconds) + 1 +
                       sizeof(payloadSize) + payloadSize);

        // Pack timestamp first
        xTimeStampInMicroSeconds = GetTimeStamp();
        const std::byte* timeStampBytes =
            reinterpret_cast<std::byte*>(&xTimeStampInMicroSeconds);
        buffer.insert(buffer.end(),
                      timeStampBytes,
                      timeStampBytes + sizeof(xTimeStampInMicroSeconds));

        // Then pack payload type
        buffer.push_back(static_cast<std::byte>(xType));

        // Then pack payload length
        const std::byte* payloadSizeBytes =
            reinterpret_cast<std::byte*>(&payloadSize);
        buffer.insert(buffer.end(),
                      payloadSizeBytes,
                      payloadSizeBytes + sizeof(payloadSize));

        // Finally, pack the payload bytes
        buffer.insert(buffer.end(), payloadBuffer.begin(), payloadBuffer.end());

        return buffer;
    }

    /// @brief Get read/write reference to the message type
    /// @return
    XRocketPayloadType&
    GetType()
    {
        return xType;
    }

    /// @brief Overrides assignment operator to make new packets dynamically
    /// @param other
    /// @return
    XRocketPacket&
    operator=(const XRocketPacket& other)
    {
        if (this == &other)
        {
            return *this;
        }

        this->xTimeStampInMicroSeconds = other.xTimeStampInMicroSeconds;
        this->xType = other.xType;
        this->xPayload = std::make_unique<XRocketPayload>(*other.xPayload);

        return *this;
    }

   private:
    uint64_t xTimeStampInMicroSeconds = 0;
    XRocketPayloadType xType = UNKNOWN;
    std::unique_ptr<XRocketPayload> xPayload;

    /// @brief Function wrapper for system call to simplify usage
    /// @return
    uint64_t
    GetTimeStamp()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }
};
