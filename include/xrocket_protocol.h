/*
Author: Logan Wright

Description: Protocol tools for communication between applications and avionics
*/

#pragma once

// C++20 STL
#include <chrono>

// XRocket tools
#include <ack_payload.h>
#include <command_payload.h>
#include <telemetry_payload.h>

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
#if DEBUGGING_PROTOCOL
            std::cerr
                << "XRocketPacket ctor: packet too small for header. size="
                << size << " expected>=" << MIN_HEADER << "\n";
#endif
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

        // Validate that payloadLength fits in the remaining bytes
        if (offset + static_cast<std::size_t>(payloadLength) > size)
        {
#if DEBUGGING_PROTOCOL
            std::cerr
                << "XRocketPacket ctor: payloadLength extends beyond packet. "
                << "offset=" << offset << " payloadLength=" << payloadLength
                << " size=" << size << "\n";
#endif
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

            case XRocketPayloadType::ACK:
                xPayload = std::make_unique<XRocketAckPayload>();
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
