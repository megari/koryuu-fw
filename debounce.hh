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
                bool state;
                volatile bool is_down;
        public:
                DebouncedButton()
                        : button(), counter(0), state(false), is_down(false)
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
                            if (++counter == 4) {
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
