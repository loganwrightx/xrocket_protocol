/*
Author: Logan Wright

Description: Payload base type
*/

#pragma once

#include <cstdint>
#include <cstring>
#include <format>
#include <memory>
#include <vector>

#include <xrocket_types.h>

enum XRocketPayloadType : uint8_t
{
    TELEMETRY = 0U,
    COMMAND,
    ACK,
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

    /// @brief Gets a format string of the data without line ending
    /// @return
    virtual std::string
    GetFormatString()
    {
        return "GetFormatString() not implemented!";
    }
};
