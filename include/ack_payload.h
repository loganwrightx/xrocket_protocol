/*
Author: Logan Wright
*/

#pragma once

#include <base_payload.h>

/// @brief Ack payload for xrocket, does not contain data
class XRocketAckPayload : public XRocketPayload
{
   public:
    XRocketAckPayload()
    {
    }

    /// @brief Gets payload type
    /// @return
    XRocketPayloadType
    GetType() const override
    {
        return XRocketPayloadType::ACK;
    }

    /// @brief Packs payload into buffer
    /// @param buffer
    void
    Pack(std::vector<std::byte>&) override
    {
    }

    /// @brief Unpacks data into payload object
    /// @param data
    /// @param size
    void
    Unpack(const std::byte*, std::size_t) override
    {
    }

    std::unique_ptr<XRocketPayload>
    clone() const override
    {
        return std::make_unique<XRocketAckPayload>(*this);
    }

    /// @brief Gets a format string of the data without line ending
    /// @return
    std::string
    GetFormatString() override
    {
        return "ACK";
    }
};
