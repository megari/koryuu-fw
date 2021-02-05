#ifndef KORYUU_TIMER_HH
#define KORYUU_TIMER_HH

#include <yaal/requirements.hh>

#ifdef __YAAL__
#include <yaal/io.hh>
#include <yaal/types/autounion.hh>

namespace koryuu_timers {

    enum OpMode {
        OM_NORMAL,
        OM_CTC,
        OM_FAST_PWM,
        OM_PHASE_CORR_PWM,
        OM_PHASE_FREQ_CORR_PWM,
    };

    template<typename T>
    struct timer_traits {
        static const uint8_t bits;
        static const struct {
            bool has_normal;
            bool has_ctc;
            bool has_fast_pwm;
            bool has_phase_correct_pwm;
            bool has_phase_freq_correct_pwm;
        } opmode_traits;
    };

    template<typename TccrA, typename TccrB, typename TccrC,
        typename TCnt, typename OcrA, typename OcrB, typename Icr,
        typename Timsk, typename Tifr>
    class Timer16 {
        TccrA tccrA;
        TccrB tccrB;
        TccrC tccrC;
        TCnt tcnt;
        OcrA ocrA;
        OcrB ocrB;
        Icr icr;
        Timsk timsk;
        Tifr tifr;
    public:
        template<OpMode opmode>
        void set_clock(uint32_t freq);

        void set_clock<OM_NORMAL>(uint32_t freq) {

        }

        void set_clock<OM_CRC>(uint32_t freq) {

        }

        void set_clock<OM_FAST_PWM>(uint32_t freq) {

        }

        void set_clock<OM_PHASE_CORR_PWM>(uint32_t freq) {

        }

        void set_clock<OM_PHASE_FREQ_CORR_PWM>(uint32_t freq) {

        }

    };


    using Timer1 =
        Timer16<Tccr1a, Tccr1b, Tccr1c, Tcnt1, Ocr1a, Icr1, Timsk1, Tifr1>;

    template<>
    struct timer_traits<Timer1> {
        static const uint8_t bits = 16;
        static const struct {
            bool has_normal = true;
            bool has_ctc = true;
            bool has_fast_pwm = true;
            bool has_phase_correct_pwm = true;
            bool has_phase_freq_correct_pwm = true;
        } opmode_traits;
    };
}
#endif // __YAAL__
#endif // KORYUU_TIMER_HH
