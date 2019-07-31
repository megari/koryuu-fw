#ifndef I2C_HELPERS_HH
#define I2C_HELPERS_HH

#include <yaal/communication/i2c_hw.hh>

using yaal::I2c_HW;

#define I2C_INIT(args...) I2c_HW.setup(##args)

template<typename ...Ts>
static inline void I2C_WRITE(uint8_t addr, Ts... args)
{
    uint8_t err = I2c_HW.write(addr, args...);
    if (err) {
#if DEBUG
        serial << _T("I2C write of size ") << asdec(sizeof...(args))
               << _T(" to addr 0x") << ashex(addr) << _T(" FAILED!\r\n");
#endif
        // TODO: should we enable all the blinkenlights here?
        while (true);
    }
}

static inline uint8_t I2C_READ_ONE(uint8_t addr, uint8_t reg)
{
    I2c_HW.write<true, false>(addr, reg);
    return I2c_HW.read(addr);
}

#endif
