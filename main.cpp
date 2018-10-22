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

ADV7280A<PortD2, PortD6, PortC2> decoder(0x20);
ADV7391<PortD7> encoder(0x2a);

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
    encoder.reset = true;
    _delay_ms(6);

#if DEBUG
    serial << _T("converter starting...\r\n");
#endif


    // Decoder setup
#if DEBUG
    serial << _T("writing I2C multi\r\n");
#endif
    // Use CVBS input on A_in1
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

    // Autodetect SD video mode
    uint8_t autodetect[] = { 0x02, 0x04 };
    I2c_HW.write_multi(decoder.address, autodetect, autodetect + 2);

    // Output control
    // Enable output drivers
    uint8_t outc[] = { 0x03, 0x0c };
    I2c_HW.write_multi(decoder.address, outc, outc + sizeof(outc));

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

    // Analog clamp control
    // 100% color bars
    uint8_t ana_clampc[] = { 0x14, 0x11 };
    I2c_HW.write_multi(decoder.address, ana_clampc, ana_clampc + 2);

    // Analog Devices control 2
    uint8_t adc2[] = { 0x1d, 0x40 };
    I2c_HW.write_multi(decoder.address, adc2, adc2 + 2);

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

    // Output sync select 2
    // Output SFL on the VS/FIELD/SFL pin
    uint8_t out_sync_sel2[] = { 0x6b, 0x14 };
    I2c_HW.write_multi(decoder.address, out_sync_sel2, out_sync_sel2 + 2);


    // Encoder setup

    // Software reset
    uint8_t enc_reset_seq[] = { 0x17, 0x02 };
    I2c_HW.write_multi(encoder.address, enc_reset_seq, enc_reset_seq + sizeof(enc_reset_seq));

    // All DACs and PLL enabled
    uint8_t seq_1_1[] = { 0x00, 0x1c };
    I2c_HW.write_multi(encoder.address, seq_1_1, seq_1_1 + sizeof(seq_1_1));

// Set this to 0 if you want to output the test pattern.
#if 1
    // SD input mode
    uint8_t seq_1_2[] = { 0x01, 0x00 };
    I2c_HW.write_multi(encoder.address, seq_1_2, seq_1_2 + sizeof(seq_1_2));

    // NTSC, SSAF luma filter, 1.3MHz chroma filter
    uint8_t seq_1_3[] = { 0x80, 0x10 };
    I2c_HW.write_multi(encoder.address, seq_1_3, seq_1_3 + sizeof(seq_1_3));
#endif

    // Pixel data valid, YPrPb, SSAF PrPb filter, AVE control, pedestal
    uint8_t seq_1_4[] = { 0x82, 0xc9 };
    I2c_HW.write_multi(encoder.address, seq_1_4, seq_1_4 + sizeof(seq_1_4));

    // Autodetect SD input standard
    uint8_t sdm6[] = { 0x87, 0x20 };
    I2c_HW.write_multi(encoder.address, sdm6, sdm6 + sizeof(sdm6));

    // Enable SD progressive mode (for allowing 240p/288p)
    uint8_t sdm7[] = { 0x88, 0x02 };
    I2c_HW.write_multi(encoder.address, sdm7, sdm7 + sizeof(sdm7));

// Set this to 1 if you want to output the test pattern.
#if 0
    // Test pattern
    uint8_t pleasework1[] = { 0x84, 0x40 };
    I2c_HW.write_multi(encoder.address, pleasework1, pleasework1 + 2);
#endif


#if DEBUG
    // A simple loop to read and print out status changes in the decoder
    uint8_t dec_status1 = 0x00;
    uint8_t dec_status2 = 0x00;
    uint8_t dec_status3 = 0x00;
    while (1) {
        I2c_HW.write(decoder.address, 0x10, true, false);
        uint8_t new_status1 = I2c_HW.read(decoder.address);
        I2c_HW.write(decoder.address, 0x12, true, false);
        uint8_t new_status2 = I2c_HW.read(decoder.address);
        I2c_HW.write(decoder.address, 0x13, true, false);
        uint8_t new_status3 = I2c_HW.read(decoder.address);

        if (new_status1 != dec_status1) {
            serial << _T("Status 1 changed:\r\n");
            serial << _T("In lock: ") << asdec(new_status1 & 0x01) << _T("\r\n");
            serial << _T("Lost lock: ") << asdec(!!(new_status1 & 0x02)) << _T("\r\n");
            serial << _T("fSC lock: ") << asdec(!!(new_status1 & 0x04)) << _T("\r\n");
            serial << _T("Follow PW: ") << asdec(!!(new_status1 & 0x08)) << _T("\r\n");
            serial << _T("Video standard: ");
            switch ((new_status1 >> 4) & 0x07) {
            case 0x00:
                serial << _T("NTSC M/J\r\n");
                break;
            case 0x01:
                serial << _T("NTSC 4.43\r\n");
                break;
            case 0x02:
                serial << _T("PAL M\r\n");
                break;
            case 0x03:
                serial << _T("PAL 60\r\n");
                break;
            case 0x04:
                serial << _T("PAL B/G/H/I/D\r\n");
                break;
            case 0x05:
                serial << _T("SECAM\r\n");
                break;
            case 0x06:
                serial << _T("PAL Combination N\r\n");
                break;
            case 0x07:
                serial << _T("SECAM 525\r\n");
                break;
            }
            serial << _T("Color kill: ") << asdec(!!(new_status1 & 0x80)) << _T("\r\n");
            serial << _T("\r\n");
        }
        dec_status1 = new_status1;

        if (new_status2 != dec_status2) {
            serial << _T("Status 2 changed:\r\n");
            serial << _T("Line length nonstandard: ") << asdec(!!(new_status2 & 0x10)) << _T("\r\n");
            serial << _T("fSC nonstandard: ") << asdec(!!(new_status2 & 0x20)) << _T("\r\n");
            serial << _T("\r\n");
        }
        dec_status2 = new_status2;

        if (new_status3 != dec_status3) {
            serial << _T("Status 3 changed:\r\n");
            serial << _T("Horizontal lock: ") << asdec(new_status3 & 0x01) << _T("\r\n");
            serial << _T("Frequency: ") << ((new_status3 & 0x04) ? _T("50") : _T("60")) << _T("\r\n");
            serial << _T("Freerun active: ") << asdec(!!(new_status3 & 0x10)) << _T("\r\n");
            serial << _T("Field length standard: ") << asdec(!!(new_status3 & 0x20)) << _T("\r\n");
            serial << _T("Interlaced: ") << asdec(!!(new_status3 & 0x40)) << _T("\r\n");
            serial << _T("PAL SW lock: ") << asdec(!!(new_status3 & 0x80)) << _T("\r\n");
            serial << _T("\r\n");
        }
        dec_status3 = new_status3;

        _delay_ms(1000);
    }
#endif

    I2c_HW.deinit();

    return 0;
}
