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

    virtual std::unique_ptr<XRocketPayload>
    clone() const
    {
        return std::make_unique<XRocketPayload>(*this);
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
        auto AppendToBuffer = [&](const auto v)
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
        constexpr std::size_t TELEMETRY_FIELDS =
            3 + 3 + 3 + 1 + 3;  // acc + gyro + mag + baro + lat/lon/alt
        constexpr std::size_t EXPECTED_TELEMETRY_BYTES =
            TELEMETRY_FIELDS * sizeof(double);
        if (size < EXPECTED_TELEMETRY_BYTES)
        {
            std::cerr << "TelemetryPayload size too small! expected "
                      << EXPECTED_TELEMETRY_BYTES << " got " << size << "\n";
            return;
        }

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

    /// @brief Clone method makes new unique pointer to *this
    /// @return
    std::unique_ptr<XRocketPayload>
    clone() const override
    {
        return std::make_unique<XRocketTelemetryPayload>(*this);
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
        auto AppendToBuffer = [&](auto v)
        {
            const std::byte* p = reinterpret_cast<std::byte*>(&v);
            buffer.insert(buffer.end(), p, p + sizeof(v));
        };

        // Always 4 grid fin commands
        uint32_t count = static_cast<uint32_t>(xGridFinCommands.size());
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
        // helper that reads into a temporary and assign to target
        auto ReadToVariable = [&](auto& v, std::size_t& offset) -> bool
        {
            using T = std::decay_t<decltype(v)>;
            if (offset + sizeof(T) > size)
                return false;
            T tmp;
            std::memcpy(&tmp, data + offset, sizeof(T));
            offset += sizeof(T);
            v = tmp;
            return true;
        };

        std::size_t offset = 0;

        // Sanity limits
        constexpr uint32_t MAX_GRIDFINS = 4;
        constexpr uint32_t MAX_TVCS = 100;

        // Need at least 4 bytes for first count
        if (size < sizeof(uint32_t))
        {
            std::cerr << "CommandPayload missing gridfin command count!\n";
            return;  // abort parsing
        }

        uint32_t count = 0;
        std::memcpy(&count, data + offset, sizeof(count));
        offset += sizeof(count);

        if (count > MAX_GRIDFINS)
        {
            std::cerr << "CommandPayload invalid gridfin count: " << count
                      << " (max " << MAX_GRIDFINS << ")\n";
            return;
        }

        // Ensure we have enough bytes for all gridfin doubles
        if (offset + count * sizeof(double) > size)
        {
            std::cerr << "CommandPayload missing gridfin command bytes!\n";
            return;
        }

        std::cout << "Unpacked " << count << " gridfins!\n";
        xGridFinCommands.resize(count);

        for (auto& gridFinCommand : xGridFinCommands)
        {
            // read into a local double then assign to the struct member
            double tmp;
            if (!ReadToVariable(tmp, offset))
            {
                std::cerr << "CommandPayload truncated while reading gridfin "
                             "command\n";
                return;
            }
            gridFinCommand.thetaInRadians = tmp;
        }

        // Next must contain the tvc count (uint32_t)
        if (offset + sizeof(uint32_t) > size)
        {
            std::cerr << "CommandPayload missing engine command count!\n";
            return;
        }

        std::memcpy(&count, data + offset, sizeof(count));
        offset += sizeof(count);

        if (count > MAX_TVCS)
        {
            std::cerr << "CommandPayload invalid engine count: " << count
                      << " (max " << MAX_TVCS << ")\n";
            return;
        }

        // Ensure enough bytes for all tvc triples
        if (offset + count * (3 * sizeof(double)) > size)
        {
            std::cerr << "CommandPayload missing engine command bytes!\n";
            return;
        }

        std::cout << "Unpacked " << count << " engines!\n";
        xTvcCommands.resize(count);

        for (uint32_t i = 0; i < count; ++i)
        {
            double tmp;
            if (!ReadToVariable(tmp, offset))
            {
                std::cerr << "Truncated reading thrust\n";
                return;
            }
            xTvcCommands[i].thrustInPercent = tmp;

            if (!ReadToVariable(tmp, offset))
            {
                std::cerr << "Truncated reading thetaX\n";
                return;
            }
            xTvcCommands[i].thetaXInRadians = tmp;

            if (!ReadToVariable(tmp, offset))
            {
                std::cerr << "Truncated reading thetaY\n";
                return;
            }
            xTvcCommands[i].thetaYInRadians = tmp;
        }
    }

    /// @brief Make a new unique pointer to *this
    /// @return
    std::unique_ptr<XRocketPayload>
    clone() const override
    {
        return std::make_unique<XRocketCommandPayload>(*this);
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
        std::size_t offset = 0;

        // Need at least timestamp (8) + type (1) + payload length (4)
        constexpr std::size_t MIN_HEADER =
            sizeof(xTimeStampInMicroSeconds) + 1 + sizeof(uint32_t);
        if (size < MIN_HEADER)
        {
            std::cerr
                << "XRocketPacket ctor: packet too small for header. size="
                << size << " expected>=" << MIN_HEADER << "\n";
            xPayload.reset();
            return;
        }

        // Read timestamp
        std::memcpy(&xTimeStampInMicroSeconds,
                    data + offset,
                    sizeof(xTimeStampInMicroSeconds));
        offset += sizeof(xTimeStampInMicroSeconds);

        // Read raw type byte for debug, then assign to enum safely
        uint8_t rawType = 0;
        std::memcpy(&rawType, data + offset, 1);
        offset += 1;
        xType = static_cast<XRocketPayloadType>(rawType);

        // Read payload length
        uint32_t payloadLength = 0;
        std::memcpy(&payloadLength, data + offset, sizeof(payloadLength));
        offset += sizeof(payloadLength);

        // Debug: print the raw header (optional, keep during debugging)
        std::cerr << "XRocketPacket ctor: timestamp="
                  << xTimeStampInMicroSeconds
                  << " rawType=" << static_cast<int>(rawType)
                  << " payloadLength=" << payloadLength << " totalSize=" << size
                  << "\n";

        // Validate that payloadLength fits in the remaining bytes
        if (offset + static_cast<std::size_t>(payloadLength) > size)
        {
            std::cerr
                << "XRocketPacket ctor: payloadLength extends beyond packet. "
                << "offset=" << offset << " payloadLength=" << payloadLength
                << " size=" << size << "\n";
            xPayload.reset();
            return;
        }

        // Create proper payload object for unpacking
        switch (xType)
        {
            case XRocketPayloadType::COMMAND:
                xPayload = std::make_unique<XRocketCommandPayload>();
                break;

            case XRocketPayloadType::TELEMETRY:
                xPayload = std::make_unique<XRocketTelemetryPayload>();
                break;

            default:
                xPayload = std::make_unique<XRocketPayload>();
                break;
        }

        // Now safe to call Unpack with exact payload bytes
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

    /// @brief Gets pointer to *this payload
    /// @return
    XRocketPayload&
    GetPayload() const
    {
        return *xPayload;
    }

    /// @brief Get read/write reference to the message type
    /// @return
    const XRocketPayloadType&
    GetType() const
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
            return *this;

        xTimeStampInMicroSeconds = other.xTimeStampInMicroSeconds;
        xType = other.xType;
        if (other.xPayload)
            xPayload = other.xPayload->clone();
        else
            xPayload.reset();

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
