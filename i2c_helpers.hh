#ifndef I2C_HELPERS_HH
#define I2C_HELPERS_HH

#include <yaal/requirements.hh>

#ifdef __YAAL__
#include <yaal/communication/i2c_hw.hh>
namespace i2c_helpers {
    using i2c_err_f_t = void (*)(uint8_t addr, uint8_t arg_count);

    template<typename ...Ts>
    inline void I2C_INIT(Ts... args)
    {
        using yaal::I2c_HW;
        I2c_HW.setup(args...);
    }

    namespace internal {
        i2c_err_f_t err_func = nullptr;
    }

    inline void I2C_set_err_func(i2c_err_f_t f)
    {
        using internal::err_func;
        err_func = f;
    }

    template<bool fail_fatal = true, typename ...Ts>
    void I2C_WRITE(uint8_t addr, Ts... args)
    {
        using yaal::I2c_HW;
        using internal::err_func;
        bool err = I2c_HW.write(addr, args...);
        if (fail_fatal && err && err_func)
            err_func(addr, sizeof...(args));
    }

    inline uint8_t I2C_READ_ONE(uint8_t addr, uint8_t reg)
    {
        using yaal::I2c_HW;
        I2c_HW.write<true, false>(addr, reg);
        return I2c_HW.read(addr);
    }
}

#endif
#endif
