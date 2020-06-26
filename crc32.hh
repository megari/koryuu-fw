#ifndef CRC32_HH
#define CRC32_HH

#include <yaal/requirements.hh>

#ifdef __YAAL__
#include <yaal/types/autounion.hh>

/*
 * Based on a simple public domain implementation of the standard CRC32
 * checksum by Björn Samuelsson at http://home.thep.lu.se/~bjorn/crc/
 */

namespace crc {
    static_assert(sizeof(size_t) > 1, "size_t is too small");
    static_assert(sizeof(unsigned long) >= 4, "unsigned long is too small");

    uint32_t crc32_for_byte(uint32_t r)
    {
        for (uint8_t j = 0; j < 8; ++j)
            r = (r & 1 ? 0 : (uint32_t)0xEDB88320UL) ^ r >> 1;
        return r ^ (uint32_t)0xFF000000UL;
    }

    namespace internal {
        // Rquires 1 kB of memory...
        uint32_t table[0x100] = { 0UL };
    }

    template<typename T>
    uint32_t crc32(const T *const data_T, const size_t n_bytes,
            const uint32_t *const init)
    {
        using internal::table;
        using yaal::autounion;
        const uint8_t *data = reinterpret_cast<const uint8_t *>(data_T);
        autounion<uint32_t, true> crc(init ? *init : 0UL);

        if (!table[0])
            for (size_t i = 0; i < 0x100UL; ++i)
                table[i] = crc32_for_byte(i);

        for (size_t i = 0; i < n_bytes; ++i)
            crc[0] = table[crc[0] ^ data[i]] ^ crc[0] >> 8;
        return crc;
    }
}
#endif // __YAAL__
#endif // CRC32_HH
