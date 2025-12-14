/*
Author: Logan Wright
*/

#pragma once

#include <base_payload.h>

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
#if DEBUGGING_PROTOCOL
            std::cerr << "CommandPayload missing gridfin command count!\n";
#endif
            return;  // abort parsing
        }

        uint32_t count = 0;
        std::memcpy(&count, data + offset, sizeof(count));
        offset += sizeof(count);

        if (count > MAX_GRIDFINS)
        {
#if DEBUGGING_PROTOCOL
            std::cerr << "CommandPayload invalid gridfin count: " << count
                      << " (max " << MAX_GRIDFINS << ")\n";
#endif
            return;
        }

        // Ensure we have enough bytes for all gridfin doubles
        if (offset + count * sizeof(double) > size)
        {
#if DEBUGGING_PROTOCOL
            std::cerr << "CommandPayload missing gridfin command bytes!\n";
#endif
            return;
        }

#if DEBUGGING_PROTOCOL
        std::cout << "Unpacked " << count << " gridfins!\n";
#endif
        xGridFinCommands.resize(count);

        for (auto& gridFinCommand : xGridFinCommands)
        {
            // read into a local double then assign to the struct member
            double tmp;
            if (!ReadToVariable(tmp, offset))
            {
#if DEBUGGING_PROTOCOL
                std::cerr << "CommandPayload truncated while reading gridfin "
                             "command\n";
#endif
                return;
            }
            gridFinCommand.thetaInRadians = tmp;
        }

        // Next must contain the tvc count (uint32_t)
        if (offset + sizeof(uint32_t) > size)
        {
#if DEBUGGING_PROTOCOL
            std::cerr << "CommandPayload missing engine command count!\n";
#endif
            return;
        }

        std::memcpy(&count, data + offset, sizeof(count));
        offset += sizeof(count);

        if (count > MAX_TVCS)
        {
#if DEBUGGING_PROTOCOL
            std::cerr << "CommandPayload invalid engine count: " << count
                      << " (max " << MAX_TVCS << ")\n";
#endif
            return;
        }

        // Ensure enough bytes for all tvc triples
        if (offset + count * (3 * sizeof(double)) > size)
        {
#if DEBUGGING_PROTOCOL
            std::cerr << "CommandPayload missing engine command bytes!\n";
#endif
            return;
        }

#if DEBUGGING_PROTOCOL
        std::cout << "Unpacked " << count << " engines!\n";
#endif
        xTvcCommands.resize(count);

        for (uint32_t i = 0; i < count; ++i)
        {
            double tmp;
            if (!ReadToVariable(tmp, offset))
            {
#if DEBUGGING_PROTOCOL
                std::cerr << "Truncated reading thrust\n";
#endif
                return;
            }
            xTvcCommands[i].thrustInPercent = tmp;

            if (!ReadToVariable(tmp, offset))
            {
#if DEBUGGING_PROTOCOL
                std::cerr << "Truncated reading thetaX\n";
#endif
                return;
            }
            xTvcCommands[i].thetaXInRadians = tmp;

            if (!ReadToVariable(tmp, offset))
            {
#if DEBUGGING_PROTOCOL
                std::cerr << "Truncated reading thetaY\n";
#endif
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

    /// @brief Gets formatted string of *this command without line ending
    /// @return
    std::string
    GetFormatString() override
    {
        std::string commandString;

        // Get grid fins
        commandString += "Gridfins: ";
        for (const auto& gridFinCommand : GetGridFinCommands())
        {
            commandString +=
                std::format("{:.4f}", gridFinCommand.thetaInRadians) + ' ';
        }

        // Get tvcs
        commandString += " | Tvcs: ";
        for (const auto& tvcCommand : GetTvcCommands())
        {
            commandString +=
                "Throttle=" +
                std::format("{:.1f}", tvcCommand.thrustInPercent) +
                " X=" + std::format("{:.2f}", tvcCommand.thetaXInRadians) +
                " Y=" + std::format("{:.2f}", tvcCommand.thetaYInRadians) + ' ';
        }

        return commandString;
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
