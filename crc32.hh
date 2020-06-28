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

    namespace internal {
        constexpr uint32_t crc32_for_byte(const uint32_t r_)
        {
            uint32_t r = r_;
            for (uint8_t j = 0; j < 8; ++j)
                r = (r & 1 ? 0 : (uint32_t)0xEDB88320UL) ^ r >> 1;
            return r ^ (uint32_t)0xFF000000UL;
        }

        template<uint32_t size>
        struct crc32_shim {
            using arr_type = uint32_t[size];
            arr_type arr;
        };

        template<uint32_t size>
        constexpr auto crc32_table() -> const crc32_shim<size> {
            crc32_shim<size> ret = { 0UL };
            for (uint32_t i = 0; i < size; ++i) {
                ret.arr[i] = crc32_for_byte(i);
            }
            return ret;
        }

        // Requires 1 kB of data memory, which is about 50% of it.
        // Hopefully this won't be a problem.
        const constexpr auto crc32table = crc32_table<0x100UL>();
    }

    template<typename T>
    uint32_t crc32(const T *const data_T, const size_t n_bytes,
            const uint32_t *const init = nullptr)
    {
        using yaal::autounion;
        const constexpr auto &table = internal::crc32table.arr;
        const uint8_t *data = reinterpret_cast<const uint8_t *>(data_T);
        autounion<uint32_t, true> crc(init ? *init : 0UL);

        for (size_t i = 0; i < n_bytes; ++i)
            crc.value() = table[crc[0] ^ data[i]] ^ crc.value() >> 8UL;
        return crc;
    }
}
#endif // __YAAL__
#endif // CRC32_HH
