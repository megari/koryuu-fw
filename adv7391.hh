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
	};
}

#endif
