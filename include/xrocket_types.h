/*
Author: Logan Wright
*/

#pragma once

#define DEBUGGING_PROTOCOL 0

namespace XProtocol
{
// Better than a #define
const unsigned int PacketBufferSize = 512;

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
