#include <yaal/io/ports.hh>
#include <yaal/io/serial.hh>
#include <yaal/communication/i2c_hw.hh>

#include <util/delay.h>

#include "adv7280.hh"
#include "adv7391.hh"
#include "i2c_helpers.hh"

#ifndef DEBUG
    #define DEBUG 1
#endif
#ifndef CALIBRATE
    #define CALIBRATE 0
#endif
#ifndef ENC_TEST_PATTERN
    #define ENC_TEST_PATTERN 0
#endif
#ifndef AUTORESET
    #define AUTORESET 1
#endif
#ifndef ERROR_PANIC
    #define ERROR_PANIC 1
#endif

#if AUTORESET
#include <avr/wdt.h>
#endif

using namespace yaal;
using namespace ad_decoder;
using namespace ad_encoder;
using namespace i2c_helpers;

ADV7280A<PortD2, PortD6, PortC2> decoder(0x20);
ADV7391<PortD7> encoder(0x2a);

PortC4 sda;
PortC5 scl;

PortB1 led_CVBS;
PortB2 led_YC;
PortB6 led_OPT;

#if DEBUG
Serial0 serial;
#endif

enum {
    CVBS,
    CVBS_PEDESTAL,
    SVIDEO,
    SVIDEO_PEDESTAL,
} curr_input;

#if ERROR_PANIC
__attribute__((noreturn))
static void i2c_err_func(uint8_t addr, uint8_t arg_count)
{
#if DEBUG
        serial << _T("I2C write of size ") << asdec(arg_count)
               << _T(" to addr 0x") << ashex(addr) << _T(" FAILED!\r\n");
#else
        (void)addr;
        (void)arg_count;
#endif

#if AUTORESET
        wdt_enable(WDTO_4S);
#endif

        led_CVBS = true;
        led_YC = true;
        led_OPT = true;
        decoder.reset = false;
        decoder.pwrdwn = false;
        encoder.reset = false;
        while (true) {
            _delay_ms(500);
            led_CVBS = !led_CVBS;
            led_YC = !led_YC;
            led_OPT = !led_OPT;
        }
}
#endif

bool smoothing_enabled = false;

PortD5 input_change;
volatile bool input_change_down = false;

static inline bool read_input_change()
{
    cli();
    const bool ret = input_change_down;
    input_change_down = false;
    sei();
    return ret;
}

static inline void input_change_debounce()
{
    static uint8_t counter = 0;
    static bool state = false;

    bool curr_state = !input_change;

    if (curr_state != state) {
        if (++counter == 4) {
            state = curr_state;
            if (state) {
                // Will be reset to false when read
                // in read_input_change().
                input_change_down = true;
            }
            counter = 0;
        }
    }
    else {
        counter = 0;
    }
}

PortB7 option;
volatile bool option_down = false;

static inline bool read_option()
{
    cli();
    const bool ret = option_down;
    option_down = false;
    sei();
    return ret;
}

static inline void option_debounce()
{
    static uint8_t counter = 0;
    static bool state = false;

    bool curr_state = !option;

    if (curr_state != state) {
        if (++counter == 4) {
            state = curr_state;
            if (state) {
                // Will be reset to false when read
                // in read_option().
                option_down = true;
            }
            counter = 0;
        }
    }
    else {
        counter = 0;
    }
}

// ISR to run debouncing every 10 ms
ISR(TIMER0_COMPA_vect)
{
    input_change_debounce();
    option_debounce();
}

static void setup_timer0()
{
    // CTC mode, prescaler 1024, target frequency 100 Hz
    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS00) | _BV(CS02);
    OCR0A = 4;

    // Generate an interrupt on compare match
    TIMSK0 = _BV(OCIE0A);
}

static void setup_encoder()
{
#if 0
    // All DACs and PLL enabled
    I2C_WRITE(encoder.address, 0x00, 0x1c);
#else
    // All DACs enabled, PLL disabled (only 2x oversampling)
    I2C_WRITE(encoder.address, 0x00, 0x1e);
#endif

    // Enable DAC autopower-down (based on cable detection)
    I2C_WRITE(encoder.address, 0x10, 0x10);

#if !ENC_TEST_PATTERN
    // SD input mode
    I2C_WRITE(encoder.address, 0x01, 0x00);

    // NTSC, SSAF luma filter, 1.3MHz chroma filter
    I2C_WRITE(encoder.address, 0x80, 0x10);
#endif

#if 0
    // Pixel data valid, YPrPb, PrPb SSAF filter, AVE control, pedestal
    I2C_WRITE(encoder.address, 0x82, 0xc9);
#else
    // Pixel data valid, YPrPb, *no* PrPb SSAF filter, AVE control, pedestal
    I2C_WRITE(encoder.address, 0x82, 0xc8);
#endif

#if 1
    // Enable VBI. Otherwise default.
    I2C_WRITE(encoder.address, 0x83, 0x14);
#endif

    // Enable subcarrier frequency lock
    I2C_WRITE(encoder.address, 0x84, 0x06);

    // Autodetect SD input standard
    I2C_WRITE(encoder.address, 0x87, 0x20);

    // Enable SD progressive mode
    I2C_WRITE(encoder.address, 0x88, 0x02);

#if ENC_TEST_PATTERN
    // Color bar test pattern
    I2C_WRITE(encoder.address, 0x84, 0x40);
#endif
}

static inline void setup_ad_black_magic()
{
    // Undocumented black magic from AD scripts:
    // 42 0E 80 ; ADI Required Write
    // 42 9C 00 ; Reset Current Clamp Circuitry [step1]
    // 42 9C FF ; Reset Current Clamp Circuitry [step2]
    // 42 0E 00 ; Enter User Sub Map
    // 42 80 51 ; ADI Required Write
    // 42 81 51 ; ADI Required Write
    // 42 82 68 ; ADI Required Write
    decoder.select_submap(DEC_SUBMAP_0x80);
    I2C_WRITE(decoder.address, 0x9c, 0x00);
    I2C_WRITE(decoder.address, 0x9c, 0xff);
    decoder.select_submap(DEC_SUBMAP_USER);
    I2C_WRITE(decoder.address, 0x80, 0x51);
    I2C_WRITE(decoder.address, 0x81, 0x51);
    I2C_WRITE(decoder.address, 0x82, 0x68);
}

enum ConvInputSelection : uint8_t {
    INPUT_CVBS   = 0,
    INPUT_SVIDEO = 1,
};

static void set_smoothing(ConvInputSelection input, bool smoothing)
{
    if (smoothing) {
        // Shaping filter control 1, SVHS 3 luma & chroma LPF
        I2C_WRITE(decoder.address, 0x17, 0x44);
    }
    else if (input == INPUT_CVBS) {
        // Shaping filter control 1, AD recommendation for CVBS
        I2C_WRITE(decoder.address, 0x17, 0x41);
    }
    else /* if (input == INPUT_SVIDEO) */ {
        // Shaping filter control 1, set to default
        I2C_WRITE(decoder.address, 0x17, 0x01);
    }

    if (smoothing) {
        // Shaping filter control 2
        // SVHS 3 luma LPF
        I2C_WRITE(decoder.address, 0x18, 0x84);
    }
    else {
        // Shaping filter control 2
        // Select best filter automagically
        I2C_WRITE(decoder.address, 0x18, 0x13);
    }

    // Antialiasing Filter Control 1
    decoder.set_aa_filters(!smoothing, false, false, false, false);
}

static void setup_video(ConvInputSelection input, bool pedestal, bool smoothing)
{
    // Software reset decoder and encoder
    decoder.set_power_management(false, true);
    I2C_WRITE<false>(encoder.address, 0x17, 0x02);

    // Decoder setup

    // Exit powerdown
    decoder.set_power_management(false, false);
    _delay_ms(10);

    // AFE IBIAS (undocumented register, used in recommended scripts)
    if (input == INPUT_CVBS) {
        I2C_WRITE(decoder.address, 0x52, 0xcd);
    }
    else {
        I2C_WRITE(decoder.address, 0x53, 0xce);
    }

    // Select input
    if (input == INPUT_CVBS) {
        decoder.select_input(INSEL_CVBS_Ain1);
    }
    else {
        decoder.select_input(INSEL_YC_Ain3_4);
    }

    setup_ad_black_magic();

    // Setup interrupts:
    // Interrupt on various SD events, active low, active until cleared
    decoder.select_submap(DEC_SUBMAP_INTR_VDP);
    decoder.set_interrupt_mask1(true, true, true, true, false);
    decoder.interrupt_clear1(true, true, true, true, false);
    decoder.set_interrupt_mask3(true, true, true, true, true, true, false);
    decoder.interrupt_clear3(true, true, true, true, true, true, false);
    decoder.set_interrupt_config(IDL_ACTIVE_LOW, false, 0x10, ID_MUST_CLEAR, false);
    decoder.select_submap(DEC_SUBMAP_USER);

    // Autodetect SD video mode
    if (pedestal)
        decoder.select_autodetection(AD_PALBGHID_NTSCM_SECAM);
    else
        decoder.select_autodetection(AD_PALBGHID_NTSCJ_SECAM);

    // Output control
    // Enable output drivers, enable VBI
    decoder.set_output_control(false, true);

    // Extended output control
    // Output full range, enable SFL, blank chroma during VBI
    decoder.set_ext_output_control(true, true, true, false, false);

    // A write to a supposedly read-only register, recommended by AD scripts.
    {
        I2C_WRITE(decoder.address, 0x13, 0x00);
    }

    // Analog clamp control
    // 100% color bars
    I2C_WRITE(decoder.address, 0x14, 0x11);

    // Digital clamp control
    // Digital clamp on, time constant adaptive
    I2C_WRITE(decoder.address, 0x15, 0x60);

    // Optional smoothing
    set_smoothing(input, smoothing);

#if 0
    // Comb filter control
    // PAL: wide bandwidth, NTSC: medium-low bandwidth (01)
    I2C_WRITE(decoder.address, 0x19, 0xf6);
#endif

    // Analog Devices control 2
    // LLC pin active
    I2C_WRITE(decoder.address, 0x1d, 0x40);

    // VS/FIELD Control 1
    // EAV/SAV codes generated for Analog Devices encoder
    I2C_WRITE(decoder.address, 0x31, 0x02);

    // CTI DNR control
    // Disable CTI and CTI alpha blender, enable DNR
    decoder.set_cti_dnr_control(false, false, AB_SMOOTHEST, true);

    // Output sync select 2
    // Output SFL on the VS/FIELD/SFL pin
    I2C_WRITE(decoder.address, 0x6b, 0x14);

#if 0
    // Drive strength of digital outputs
    // Low drive strength for all
    I2C_WRITE(decoder.address, 0xf4, 0x00);
#endif

    // Encoder setup
    setup_encoder();
}

#if DEBUG > 1
static void i2c_trace(uint8_t addr, const uint8_t *begin, const uint8_t *end, bool start, bool stop)
{
    serial << _T("I2C write (start == ") << asdec(start) << _T(", stop == ")
           << asdec(stop) << _T(") to addr 0x") << ashex(addr) << _T(": { ");
    while (begin < end) {
        serial << _T("0x") << ashex(*begin) << _T(", ");
        ++begin;
    }
    serial << _T(" }\r\n");
}
#endif

int main(void)
{
#if AUTORESET
    // Must disable the watchdog timer ASAP.
    cli();
    wdt_reset();
    MCUSR &= ~_BV(WDRF);
    WDTCSR |= _BV(WDE) | _BV(WDCE);
    WDTCSR = 0x00;
    sei();
#endif

    _delay_ms(100);
    cli();

    // Setup reading the "input change" and "option" switches
    input_change.mode = INPUT_PULLUP;
    option.mode = INPUT_PULLUP;
    setup_timer0();

    // Setup LEDs
    led_CVBS.mode = OUTPUT;
    led_CVBS = false;
    led_YC.mode = OUTPUT;
    led_YC = false;
    led_OPT.mode = OUTPUT;
    led_OPT = false;

    // Using external 2kohm pullups for the I2C bus
    sda.mode = INPUT;
    scl.mode = INPUT;

    I2C_INIT();

#if ERROR_PANIC
    I2C_set_err_func(i2c_err_func);
#endif

#if DEBUG > 1
    I2c_HW.set_trace(i2c_trace);
#endif

#if DEBUG || CALIBRATE
    serial.setup(9600, DATA_EIGHT, STOP_ONE, PARITY_DISABLED);
#endif

    sei();

    _delay_ms(6);
    decoder.reset = true;
    decoder.pwrdwn = true;
    encoder.reset = true;
    _delay_ms(10);

#if CALIBRATE
    const auto old_osccal = OSCCAL;
    const auto osccal_min = (old_osccal < 20) ? 0 : (old_osccal - 20);
    const auto osccal_max = (old_osccal > 0xff - 20) ? 0xff : (old_osccal + 20);
    for (auto i = osccal_min; i != osccal_max; ++i) {
        OSCCAL = i;
        _delay_ms(10);
        serial << _T("OSCCAL = 0x") << ashex(i) << _T(" (old: 0x")
            << ashex(old_osccal) << _T(") The quick brown fox jumps over the lazy dog. åäö, ÅÄÖ\r\n");
        OSCCAL = old_osccal;
        for (size_t j = 0; j < 10; ++j)
            serial << _T("\r\n");
    }
    OSCCAL = old_osccal;

    return 0;
#endif

#if DEBUG
    serial << _T("converter starting...\r\n");
#endif

    setup_video(INPUT_CVBS, false, false);
    curr_input = CVBS;
    led_CVBS = true;

    // Main loop.
    // Reads the switch status, decoder interrupt line and the status registers.
#if DEBUG
    uint8_t dec_status1 = 0x00;
    uint8_t dec_status2 = 0x00;
#endif
    uint8_t dec_status3 = 0x00;
    bool got_interrupt = false;
    bool check_once_more = false;
    while (1) {
        if (read_input_change()) {
            switch (curr_input) {
            case CVBS:
                decoder.select_autodetection(AD_PALBGHID_NTSCM_SECAM);
                curr_input = CVBS_PEDESTAL;
                break;
            case CVBS_PEDESTAL:
                setup_video(INPUT_SVIDEO, false, false);
                curr_input = SVIDEO;
                led_CVBS = false;
                led_YC = true;
                break;
            case SVIDEO:
                decoder.select_autodetection(AD_PALBGHID_NTSCM_SECAM);
                curr_input = SVIDEO_PEDESTAL;
                break;
            case SVIDEO_PEDESTAL:
                setup_video(INPUT_CVBS, false, false);
                curr_input = CVBS;
                led_CVBS = true;
                led_YC = false;
                break;
            }
        }

        if (read_option()) {
            smoothing_enabled = !smoothing_enabled;
            switch (curr_input) {
            case CVBS:
            case CVBS_PEDESTAL:
                set_smoothing(INPUT_CVBS, smoothing_enabled);
                break;
            case SVIDEO:
            case SVIDEO_PEDESTAL:
                set_smoothing(INPUT_SVIDEO, smoothing_enabled);
                break;
            }
            led_OPT = smoothing_enabled;
        }

        got_interrupt = !decoder.intrq;

        if (got_interrupt || check_once_more) {
#if DEBUG
            if (got_interrupt) {
                serial << _T("Interrupt\r\n");
                decoder.select_submap(DEC_SUBMAP_INTR_VDP);
                uint8_t intrs1 = I2C_READ_ONE(decoder.address, 0x42);
                uint8_t intrs3 = I2C_READ_ONE(decoder.address, 0x4a);
                decoder.select_submap(DEC_SUBMAP_USER);
                serial << _T("Interrupt status 1: 0x") << ashex(intrs1) << _T("\r\n");
                serial << _T("Interrupt status 3: 0x") << ashex(intrs3) << _T("\r\n");
            }

            uint8_t new_status1 = I2C_READ_ONE(decoder.address, 0x10);
            uint8_t new_status2 = I2C_READ_ONE(decoder.address, 0x12);
#endif

            uint8_t new_status3 = I2C_READ_ONE(decoder.address, 0x13);

#if DEBUG
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

#if 1
                uint8_t fsc[4] = { 0, 0, 0, 0 };
                for (uint8_t i = 0; i < 4; ++i)
                    fsc[i] = I2C_READ_ONE(encoder.address, 0x8c + i);

                uint32_t fsc32 = (uint32_t)fsc[3] << 24ul;
                fsc32 |= (uint32_t)fsc[2] << 16ul;
                fsc32 |= (uint32_t)fsc[1] << 8ul;
                fsc32 |= (uint32_t)fsc[0];

                // Actually, the calculation is more involved.
                // See the ADV7391 datasheet, section "SD Subcarrier frequency
                // control"
                serial << _T("Subcarrier frequency reg: 0x") << ashex(fsc32) << _T("\r\n");
                serial << _T("Subcarrier frequency reg: ") << asdec(fsc32) << _T("\r\n");
#endif

                serial << _T("\r\n");
            }
            dec_status1 = new_status1;

            if (new_status2 != dec_status2) {
                serial << _T("Status 2 changed:\r\n");
                serial << _T("Macrovision color striping detected: ") << asdec(!!(new_status2 & 0x01)) << _T("\r\n");
                serial << _T("Macrovision color striping type: ") << asdec(!!(new_status2 & 0x02)) << _T("\r\n");
                serial << _T("Macrovision pseudo sync pulses detected: ") << asdec(!!(new_status2 & 0x04)) << _T("\r\n");
                serial << _T("Macrovision AGC pulses detected: ") << asdec(!!(new_status2 & 0x08)) << _T("\r\n");
                serial << _T("Line length nonstandard: ") << asdec(!!(new_status2 & 0x10)) << _T("\r\n");
                serial << _T("fSC nonstandard: ") << asdec(!!(new_status2 & 0x20)) << _T("\r\n");
                serial << _T("\r\n");
            }
            dec_status2 = new_status2;
#endif

            if (new_status3 != dec_status3) {
#if DEBUG
                serial << _T("Status 3 changed:\r\n");
                serial << _T("Horizontal lock: ") << asdec(new_status3 & 0x01) << _T("\r\n");
                serial << _T("Frequency: ") << ((new_status3 & 0x04) ? _T("50") : _T("60")) << _T("\r\n");
                serial << _T("Freerun active: ") << asdec(!!(new_status3 & 0x10)) << _T("\r\n");
                serial << _T("Field length standard: ") << asdec(!!(new_status3 & 0x20)) << _T("\r\n");
                serial << _T("Interlaced: ") << asdec(!!(new_status3 & 0x40)) << _T("\r\n");
                serial << _T("PAL SW lock: ") << asdec(!!(new_status3 & 0x80)) << _T("\r\n");
                serial << _T("\r\n");
#endif
            }
            dec_status3 = new_status3;

            // Clear all interrupt flags...
            if (got_interrupt) {
                decoder.select_submap(DEC_SUBMAP_INTR_VDP);
                decoder.interrupt_clear1(true, true, true, true, false);
                decoder.interrupt_clear3(true, true, true, true, true, true, false);
                decoder.select_submap(DEC_SUBMAP_USER);
            }

            // ... but check the status registers once more in case something
            // happened in the meanwhile.
            check_once_more = got_interrupt;
        }
    }

    I2c_HW.deinit();

    return 0;
}
