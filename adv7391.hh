#ifndef ADV7391_HH
#define ADV7391_HH

#include <yaal/io/ports.hh>
#include <yaal/communication/i2c_hw.hh>

namespace ad_encoder {
	using namespace yaal;

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
            /*
                56 17 02 ; Software Reset [Encoder Writes Begin]
                56 00 9C ; Power up DAC's and PLL
                56 01 70 ; ED at 54MHz input
                56 30 1C ; 625p at 50 Hz with Embedded Timing
                56 31 01 ; Pixel Data Valid [Encoder Writes finished]
            */
            uint8_t mode[] = { 0x01, value };
            I2c_HW.write_multi(address, mode, mode + sizeof(mode));
        }

        void set_mode0(uint8_t value) {
            uint8_t mode[] = { 0x02, value };
            I2c_HW.write_multi(address, mode, mode + sizeof(mode));
        }

        void set_ed_hd_mode1(uint8_t value) {
            uint8_t mode[] = { 0x30, value };
            I2c_HW.write_multi(address, mode, mode + sizeof(mode));
        }

        void set_ed_hd_mode2(uint8_t value) {
            uint8_t mode[] = { 0x31, value };
            I2c_HW.write_multi(address, mode, mode + sizeof(mode));
        }

        void set_ed_hd_mode3(uint8_t value) {
            uint8_t mode[] = { 0x32, value };
            I2c_HW.write_multi(address, mode, mode + sizeof(mode));
        }
	};
}

#endif
