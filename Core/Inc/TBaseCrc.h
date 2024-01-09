/*
 * TBaseCrc.h
 *
 *  Created on: Oct 28, 2016
 */

#ifndef TBASECRC_H_
#define TBASECRC_H_

#include <cstdint>
#include <cstdlib>
#include "ICRC.h"

/**
 * @brief Base CRC object to be used by almost all implementations.
 */
class TBaseCrc : public ICRC
{
public:
    /**
     * @copydoc ICRC::reset
     */
    inline void reset() override { crc = init_val; }

    /**
     * @copydoc ICRC::calc
     *
     * This implementation uses a simple bitwise algorithm to accumulate a CRC. It can be useful to
     * reuse this method if some hardware can't do a bytewise CRC.
     */
    void calc(std::uint8_t data) override;

    /**
     * @copydoc ICRC::calcBlock
     *
     * This implementation simply call `calc` on all the bytes in the supplied buffer.
     */
    void calcBlock(const void *pBuffer, std::size_t bufferLength) override;

    /**
     * @copydoc ICRC::get
     */
    inline std::uint32_t get() override { return crc; }

    /**
     * @copydoc ICRC::lock
     *
     * By default, there's nothing to lock.
     */
    inline void lock() override {};

    /**
     * @copydoc ICRC::unlock
     */
    inline void unlock() override {};

    static constexpr std::uint32_t init_val   = 0xFFFF'FFFF;
    static constexpr std::uint32_t polynomial = 0x04C1'1DB7;

protected:
    std::uint32_t crc = init_val;
};

#endif /* TBASECRC_H_ */
