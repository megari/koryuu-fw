#ifndef KORYUU_DEBOUNCE_HH
#define KORYUU_DEBOUNCE_HH
#include <yaal/requirements.hh>

#ifdef __YAAL__
#include <yaal/io/ports.hh>

#include <avr/interrupt.h>

namespace koryuu {
        template<typename ButtonPort>
        class DebouncedButton {
                ButtonPort button;
                uint8_t counter;
                const uint8_t debounce_factor;
                bool state;
                volatile bool is_down;
        public:
                DebouncedButton(uint8_t db_factor = 16)
                        : button(), counter(0), debounce_factor(db_factor),
                        state(false), is_down(false)
                {}

                YAAL_INLINE("DBButton::set_mode()")
                void set_mode(yaal::Mode mode)
                {
                        button.mode = mode;
                }

                YAAL_INLINE("DBButton::debounce()")
                void debounce()
                {
                        bool curr_state = !button;

                        if (curr_state != state) {
                            if (++counter == debounce_factor) {
                                state = curr_state;
                                if (state) {
                                    // Will be reset to false when read
                                    // in read().
                                    is_down = true;
                                }
                                counter = 0;
                            }
                        }
                        else {
                            counter = 0;
                        }
                }

                YAAL_INLINE("DBButton::read()")
                bool read()
                {
                        cli();
                        const bool ret = is_down;
                        is_down = false;
                        sei();
                        return ret;
                }
        };
}
#endif // __YAAL__
#endif // KORYUU_DEBOUNCE_HH
