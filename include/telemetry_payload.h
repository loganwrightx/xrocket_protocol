/*
Author: Logan Wright
*/

#pragma once

#include <base_payload.h>

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
#if DEBUGGING_PROTOCOL
            std::cerr << "TelemetryPayload size too small! expected "
                      << EXPECTED_TELEMETRY_BYTES << " got " << size << "\n";
#endif
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

    /// @brief Gets formatted string of *this telemetry without line ending
    /// @return
    std::string
    GetFormatString() override
    {
        std::string telemetryString;

        // Accelerometer readings
        const std::vector<double>& acc = GetAccelerometerReading();

        telemetryString += "Acc: " + std::format("{:.4f}", acc[0]) + ' ' +
                           std::format("{:.4f}", acc[1]) + ' ' +
                           std::format("{:.4f}", acc[2]) + " | ";

        // Gyroscope readings
        const std::vector<double>& gyro = GetGyroscopeReading();

        telemetryString += "Gyro: " + std::format("{:.4f}", gyro[0]) + ' ' +
                           std::format("{:.4f}", gyro[1]) + ' ' +
                           std::format("{:.4f}", gyro[2]) + " | ";

        // Magnetometer readings
        const std::vector<double>& mag = GetMagnetometerReading();

        telemetryString += "Mag: " + std::format("{:.4f}", mag[0]) + ' ' +
                           std::format("{:.4f}", mag[1]) + ' ' +
                           std::format("{:.4f}", mag[2]) + " | ";

        // Latitude reading
        telemetryString +=
            "Lat: " + std::format("{:.4f}", GetLatitudeReading()) + " | ";

        // Longitude reading
        telemetryString +=
            "Lon: " + std::format("{:.4f}", GetLongitudeReading()) + " | ";

        // Barometer reading
        telemetryString +=
            "Baro: " + std::format("{:.4f}", GetBarometerReading());

        return telemetryString;
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
    std::vector<double> xAcc{ 0.0, 0.0, 0.0 }, xGyro{ 0.0, 0.0, 0.0 },
        xMag{ 0.0, 0.0, 0.0 };
    double xBaro = 0.0, xLat = 0.0, xLon = 0.0, xAlt = 0.0;
};
