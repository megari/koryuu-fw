#ifndef KORYUU_TIMER_HH
#define KORYUU_TIMER_HH

#include <yaal/requirements.hh>

#ifdef __YAAL__
#include <yaal/core/cpu.hh>
#include <yaal/io.hh>
#include <yaal/types/autounion.hh>

namespace koryuu_timers {

    using yaal::internal::enable_if_t;
    //using freq_t = yaal::freq_t;
    using yaal::freq_t;

    enum OpMode {
        OM_NORMAL,
        OM_CTC,
        OM_FAST_PWM,
        OM_PHASE_CORR_PWM,
        OM_PHASE_FREQ_CORR_PWM,
    };

    namespace timer_traits {
        template<typename T>
        static const uint8_t bits;

        template<typename T, OpMode om>
        static bool has_opmode;
#if 0
        static const struct {
            bool has_normal;
            bool has_ctc;
            bool has_fast_pwm;
            bool has_phase_correct_pwm;
            bool has_phase_freq_correct_pwm;
        } opmode_traits;
#endif
    };

    enum CTC_Top_Reg {
        CTC_TOP_OCR,
        CTC_TOP_ICR,
    };

    struct clock_params_16 {
        freq_t freq;
        uint16_t top;
        uint16_t prescaler_factor;
    };

    constexpr freq_t min_freq(freq_t a, freq_t b) {
        return a < b ? a : b;
    }

    constexpr freq_t abs_sub_freq(freq_t a, freq_t b) {
        return a > b ? (a - b) : (b - a);
    }

    template<
        OpMode opmode,
        CTC_Top_Reg top_reg,
        enable_if_t<opmode == OM_CTC, OpMode>* = nullptr,
        enable_if_t<top_reg == CTC_TOP_OCR, OpMode>* = nullptr
        >
    constexpr clock_params_16 calc_params_16(freq_t freq) {
        //target_freq = freq;
#if 0
        if (!__builtin_constant_p(freq))
            return clock_params_16 {};
#endif
        /*
         * Frequency:
         * f = (f_clk_I/O)/(2 * prescale_factor * (1 + TOP))
         * 2 * prescale_factor * (1 + TOP) = (f_clk_I/O)/f
         * prescale_factor * (1 + TOP) = (f_clk_I/O)/(2 * f)
         * prescale_factor = (f_clk_I/O)/(2 * f * (1 + TOP))
         * TOP = (f_clk_I/O)/(2 * f * prescale_factor) - 1
         *
         * prescale_factor * (1 + TOP) = target_val
         * TOP = target_val/prescale_factor - 1
         *
         * f = (f_clk_I/O)/(2 * target_val);
         *
         * prescale_factor is determined by CSn{1,2,3} bits.
         */
        const freq_t f_io = yaal::cpu.clock.get();
        const freq_t target_val_floor = f_io / (2 * freq);
        const freq_t target_val_ceil =
            (f_io + 2 * freq - 1) / (2 * freq);
        clock_params_16 params_opt =
            { (freq_t) -1, (uint16_t) -1, (uint16_t) -1 };
        freq_t freq_opt = (freq_t) -1;
        bool found_solution = false;

        //constexpr uint16_t prescale_factors[] = { 1, 8, 64, 256, 1024 };
        constexpr uint16_t prescale_factors[] = { 1024, 256, 64, 8, 1 };
        for (const auto factor : prescale_factors) {
            /*
             * Max frequency attainable (TOP == 0):
             *
             * f_max = (f_clk_I/O)/(2 * prescale_factor)
             *
             * Let f_clk_I/O := 1000000.
             *
             * prescale_factor |  f_max (Hz)   | f_min (Hz) | granularity (Hz)
             * ----------------+---------------+------------+-----------------
             *        1        | 500'000       |   7.630    | max 250'000
             *        8        |  62'500       |   0.954    | max  31'250
             *       64        |   7'812.5     |   0.119    | max   3'906.25
             *      256        |   1'953.125   |   0.0298   | max     976.57
             *     1024        |     488.28125 |   0.00745  | max     244.14
             *
             * Granularity at given freq:
             * f_gran(f) = (f_clk_I/O)/(2 * prescale_factor * (1 + TOP))
             *
             */
            const freq_t freq_max = f_io/(2 * factor);
            if (freq_max < freq)
                continue;

            const freq_t cur_top_floor =
                target_val_floor / (freq_t) factor - 1;
            const freq_t cur_top_ceil =
                target_val_ceil / (freq_t) factor - 1;
            if (cur_top_floor >= (1ull << 16ull) ||
                cur_top_ceil >= (1ull << 16ull)
#if 0
            ||
                    (((f_io/(freq_t) factor) / 128) < (1ull << 16ull) &&
                    cur_top_ceil <= (f_io/(freq_t) factor) / 128)
#endif
                )
            {
                continue;
            }

            const freq_t freq_ceil =
                cur_top_floor < (1ull << 16ull) ?
                    f_io / (2 * target_val_floor) : (freq_t)-1;
            const freq_t freq_floor =
                cur_top_ceil < (1ull << 16ull) ?
                    f_io / (2 * target_val_ceil) : (freq_t)-1;

            const freq_t f_ceil_diff = abs_sub_freq(freq, freq_ceil);
            const freq_t f_floor_diff = abs_sub_freq(freq, freq_floor);
            const freq_t f_old_min_diff = abs_sub_freq(freq, freq_opt);
            const freq_t f_min_diff =
                min_freq(min_freq(f_ceil_diff, f_floor_diff), f_old_min_diff);

            if (f_min_diff == f_old_min_diff)
                continue;
            else if (f_min_diff == f_floor_diff) {
                params_opt = { freq, (uint16_t) cur_top_ceil, factor };
                freq_opt = freq_floor;
                found_solution = true;
            }
            else {
                params_opt = { freq, (uint16_t) cur_top_floor, factor };
                freq_opt = freq_ceil;
                found_solution = true;
            }
        }

        if (found_solution)
            return params_opt;
    }

    template<typename TccrA, typename TccrB, typename TccrC,
        typename TCnt, typename OcrA, typename OcrB, typename Icr,
        typename Timsk, typename Tifr>
    class Timer16 {
        typedef
            Timer16<TccrA, TccrB, TccrC, TCnt, OcrA, OcrB, Icr, Timsk, Tifr>
            self_type;
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

        freq_t target_freq;
        freq_t real_freq;
        uint16_t top_val;
        uint16_t prescaler_factor;

        bool valid;

        template<OpMode opmode, freq_t freq>
        constexpr void set_clock() {
            set_clock_<opmode, freq>();
        }

private:

#if 0
        template<OpMode opmode,
            freq_t freq,
            enable_if_t<timer_traits::has_opmode<self_type, opmode>, self_type>* = nullptr,
            enable_if_t<opmode == OM_NORMAL, OpMode>* = nullptr>
        void set_clock_() {
            ;
        }
#endif

        template<OpMode opmode,
            freq_t freq,
            enable_if_t<
                timer_traits::has_opmode<self_type, opmode>,
                self_type>* = nullptr,
            enable_if_t<opmode == OM_CTC, OpMode>* = nullptr>
        inline void set_clock_() {
            constexpr freq_t f_io = yaal::cpu.clock.get();
            constexpr clock_params_16 parm =
                calc_params_16<opmode, CTC_TOP_OCR>(freq);
            target_freq = freq;
            top_val = parm.top;
            prescaler_factor = parm.prescaler_factor;
            real_freq = f_io / (2 * prescaler_factor * (top_val + 1));
        }

#if 0
        template<>
        void set_clock_<self_type, OM_CRC, true>(uint32_t freq) {

        }

        template<>
        void set_clock_<self_type, OM_FAST_PWM, true>(uint32_t freq) {

        }

        template<>
        void set_clock_<self_type, OM_PHASE_CORR_PWM, true>(uint32_t freq) {

        }

        template<>
        void set_clock_<self_type, OM_PHASE_FREQ_CORR_PWM, true>(uint32_t freq) {

        }
#endif
    };

    namespace internal {
        using namespace yaal::internal;
        using Timer1 =
            Timer16<Tccr1a, Tccr1b, Tccr1c, Tcnt1, Ocr1a, Ocr1b,
            Icr1, Timsk1, Tifr1>;
    };
    using internal::Timer1;

    namespace timer_traits {
        template<>
        constexpr uint8_t bits<Timer1> = 16;
#if 0
        template<OpMode opmode>
        static constexpr bool has_opmode<Timer1> = has_opmode_<opmode>;
#endif
        template<>
        constexpr bool has_opmode<Timer1, OM_NORMAL> = true;
        template<>
        constexpr bool has_opmode<Timer1, OM_CTC> = true;
        template<>
        constexpr bool has_opmode<Timer1, OM_FAST_PWM> = true;
        template<>
        constexpr bool has_opmode<Timer1, OM_PHASE_CORR_PWM> = true;
        template<>
        constexpr bool has_opmode<Timer1, OM_PHASE_FREQ_CORR_PWM> = true;
    };

    static Timer1 timer1;

    template<typename T>
    constexpr T constexprify(T val) {
        return val;
    }

    inline void timertest() {
        timer1.set_clock<OM_CTC, constexprify(1337)>();
    }

    inline void timertest2() {
        timer1.set_clock<OM_CTC, 0x4B00B>();
    }
}
#endif // __YAAL__
#endif // KORYUU_TIMER_HH
