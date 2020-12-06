#ifndef ADV7391_HH
#define ADV7391_HH
#include <yaal/requirements.hh>

#ifdef __YAAL__
#include <yaal/io/ports.hh>
#include <yaal/communication/i2c_hw.hh>

namespace ad_encoder {
    using namespace yaal;
    using namespace i2c_helpers;

    template<typename RESET>
    class ADV7391 {
    public:
        RESET reset;

        unsigned char address;

        ADV7391(unsigned char addr) : address(addr) {
            reset.mode = OUTPUT;
            reset = false;
        }

        void mode_select(uint8_t value) {
            I2C_WRITE(address, 0x01, value);
        }

        void set_mode0(uint8_t value) {
            I2C_WRITE(address, 0x02, value);
        }

        void set_ed_hd_mode1(uint8_t value) {
            I2C_WRITE(address, 0x30, value);
        }

        void set_ed_hd_mode2(uint8_t value) {
            I2C_WRITE(address, 0x31, value);
        }

        void set_ed_hd_mode3(uint8_t value) {
            I2C_WRITE(address, 0x32, value);
        }
    };
}

#endif
#endif
