/*
 * ICRC.h
 *
 * Created on: 2015-10-20
 *
 * Author: dumarjo
 * Author: James-Adam Renquinha Henri
 *
 * Specification reference (if available)
 *
 * Modifications: (major modification only)
 * Date: 2015-10-20
 * Check in: File Created
 * Author: dumarjo
 * Description
 *
 */

#ifndef ICRC_H_
#define ICRC_H_

#include <cstddef>
#include <cstdint>

/**
 * @brief Base interface for computing a CRC.
 *
 * This class abstracts away the hardware used for the computation of a CRC.
 */
class ICRC
{
public:
    /**
     * @brief Virtual dtor.
     */
    virtual ~ICRC() = default;

    /**
     * @brief Reset the CRC to its default value.
     */
    virtual void reset() = 0;

    /**
     * @brief Accumulate one byte to the CRC.
     * @param data Data (byte) to be computed
     */
    virtual void calc(std::uint8_t data) = 0;

    /**
     * @brief Accumulate a block of data to the CRC.
     * @param pBuffer      Pointer of the data to be computed.
     * @param bufferLength Size (in bytes) of the area pointed by `pBuffer`.
     *
     * @note  There is no alignment constraints on `pBuffer`.
     */
    virtual void calcBlock(const void *pBuffer, std::size_t bufferLength) = 0;

    /**
     * @brief  Return the cumulated value.
     * @return Cumulated CRC value.
     *
     * @note   Depending on the underlying implementation, it may not be safe to call this method
     *         between invokations of `calc` and `calcBlock`. Thus, it's best to use this method
     *         only when the computation is done and no further CRC cumulation is needed.
     */
    virtual std::uint32_t get() = 0;

    /**
     * @brief Lock the CRC object.
     *
     * If the underlying implementation is based on actual hardware, then there is a possibility of
     * concurrent accesses to the hardware (in an RTOS). This method signals that the hardware shall
     * be locked to prevent concurrent uses of the same hardware.
     */
    virtual void lock() = 0;

    /**
     * @brief Unlock the previously locked CRC object.
     */
    virtual void unlock() = 0;
};

#endif /* ICRC_H_ */
