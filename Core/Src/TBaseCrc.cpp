/*
 * TBaseCrc.cpp
 *
 *  Created on: 2014-01-29
 */

#include "TBaseCrc.h"
#include <limits>

using std::uint8_t
    , std::int32_t
    , std::uint32_t
    , std::size_t
    , std::numeric_limits
    ;

void TBaseCrc::calc(uint8_t data)
{
    static constexpr auto nbits = numeric_limits<uint8_t>::digits;
    static constexpr auto bmask = 1 << numeric_limits<int32_t>::digits;

    // The standard, one-byte-at-a-time CRC.
    crc ^= uint32_t{data} << 030;

    for (int i = 0; i < nbits; ++i) {
        const bool bit = crc & bmask;
        crc = (crc << 1) ^ (bit? polynomial : 0);
    }
}

void TBaseCrc::calcBlock(const void *pBuffer, size_t bufferLength)
{
    auto p = reinterpret_cast<const uint8_t *>(pBuffer);
    const auto end = p + bufferLength;

    while (p != end) {
        calc(*p++);
    }
}
