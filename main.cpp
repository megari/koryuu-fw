#include <yaal/io/ports.hh>
#include <yaal/io/serial.hh>
#include <yaal/communication/i2c_hw.hh>

#include <util/delay.h>

#define DEBUG 1

using namespace yaal;

template<typename INTRQ, typename RESET, typename PWRDWN>
class ADV7280A {
public:
    INTRQ intrq;
    RESET reset;
    PWRDWN pwrdwn;

    unsigned char address;

    ADV7280A(unsigned char addr) : address(addr) {
        intrq.mode = INPUT_PULLUP;
        reset.mode = OUTPUT;
        reset = false;
        pwrdwn.mode = OUTPUT;
        pwrdwn = false;
    }
};

ADV7280A<PortD2, PortD6, PortC2> decoder(0x20);
// TODO: encoder

PortC4 sda;
PortC5 scl;

#if DEBUG
Serial0 serial;
#endif

int main(void)
{
    _delay_ms(100);
    cli();

    // Using external 2kohm pullups for the I2C bus
    sda.mode = INPUT;
    scl.mode = INPUT;

    I2c_HW.setup();

#if DEBUG
    serial.setup(9600, DATA_EIGHT, STOP_ONE, PARITY_DISABLED);
#endif

    sei();

    _delay_ms(6);
    decoder.reset = true;
    decoder.pwrdwn = true;
    _delay_ms(6);

#if 1
while(1) {
#if DEBUG
    serial << _T("converter starting...\r\n");
#endif

#if 1
    serial << _T("writing I2C multi\r\n");
    // Use CVBS input on A_in1
    // INSEL[4:0] = 00000
    uint8_t insel[] = { 0x00, 0x00 };
    I2c_HW.write_multi(decoder.address, insel, insel + sizeof(insel));
#if DEBUG
#if DEBUG > 1
    serial << _T("got_sla_ack: ") << ashex(I2c_HW.got_sla_ack) << _T("\r\n");
    serial << _T("got_data_ack: ") << ashex(I2c_HW.got_data_ack) << _T("\r\n");
    serial << _T("wrote_all: ") << ashex(I2c_HW.wrote_all) << _T("\r\n");
    serial << _T("wrote I2C multi\r\n");
#endif
    I2c_HW.write(decoder.address, 0x00, true, false);
#if DEBUG > 1
    serial << _T("got_sla_ack: ") << ashex(I2c_HW.got_sla_ack) << _T("\r\n");
    serial << _T("got_data_ack: ") << ashex(I2c_HW.got_data_ack) << _T("\r\n");
    serial << _T("wrote_all: ") << ashex(I2c_HW.wrote_all) << _T("\r\n");
#endif
    uint8_t insel_rb = I2c_HW.read(decoder.address, false);
#if DEBUG > 1
    serial << _T("got_sla_ack: ") << ashex(I2c_HW.got_sla_ack) << _T("\r\n");
    serial << _T("got_data_ack: ") << ashex(I2c_HW.got_data_ack) << _T("\r\n");
#endif
    if (insel_rb == insel[1])
        serial << _T("insel OK: 0x") << ashex(insel_rb) << _T("\r\n");
    else
        serial << _T("insel FAIL: 0x") << ashex(insel_rb) << _T("\r\n");
#endif

    // Extended output control
    // Output extended range, enable SFL, blank chroma during VBI etc.
    uint8_t ext_outc[] = { 0x04, 0x37 };
    I2c_HW.write_multi(decoder.address, ext_outc, ext_outc + sizeof(ext_outc));
#if DEBUG
    I2c_HW.write(decoder.address, 0x04, true, false);
    uint8_t rb = I2c_HW.read(decoder.address);
    if (rb == ext_outc[1])
        serial << _T("ext out control OK: 0x") << ashex(rb) << _T("\r\n");
    else
        serial << _T("ext out control FAIL: 0x") << ashex(rb) << _T("\r\n");
#endif

    // Power management
    // Power on
    uint8_t pwr_mgmt[] = { 0x0f, 0x00 };
    I2c_HW.write_multi(decoder.address, pwr_mgmt, pwr_mgmt + sizeof(pwr_mgmt));
#if DEBUG
    I2c_HW.write(decoder.address, 0x0f, true, false);
    rb = I2c_HW.read(decoder.address);
    if (rb == pwr_mgmt[1])
        serial << _T("pwr mgmt OK: 0x") << ashex(rb) << _T("\r\n");
    else
        serial << _T("pwr mgmt FAIL: 0x") << ashex(rb) << _T("\r\n");
#endif

    // VS/FIELD Control 1
    // EAV/SAV codes generated for Analog Devices encoder
    uint8_t vs_fieldc[] = { 0x31, 0x02 };
    I2c_HW.write_multi(decoder.address, vs_fieldc, vs_fieldc + sizeof(vs_fieldc));
#if DEBUG
    I2c_HW.write(decoder.address, 0x31, true, false);
    rb = I2c_HW.read(decoder.address);
    if (rb == vs_fieldc[1])
        serial << _T("VS/Field Control 1 OK: 0x") << ashex(rb) << _T("\r\n");
    else
        serial << _T("VS/Field Control 1 FAIL: 0x") << ashex(rb) << _T("\r\n");
#endif
#endif

    _delay_ms(5000);
}
#endif
}
