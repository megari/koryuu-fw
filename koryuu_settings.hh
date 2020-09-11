#ifndef KORYUU_SETTINGS_HH
#define KORYUU_SETTINGS_HH

#include <yaal/requirements.hh>
#include <yaal/types/autounion.hh>

#ifdef __YAAL__
#include <avr/eeprom.h>
#include <string.h>
#include "crc32.hh"

namespace koryuu_settings {
    using yaal::autounion;
    using yaal::internal::enable_if_t;
    using yaal::internal::typeof_t;
#define typeof_(expr) typeof_t<decltype(expr)>

    constexpr char SETTINGS_MAGIC[8] =
        { 'K', 'R', 'Y', 'U', 'C', 'O', 'N', 'S' };
    constexpr uint16_t CURR_VERSION = 0x0002;
    constexpr uint16_t MIN_READ_VERSION = 0x0001;

    enum Input : uint8_t {
        CVBS = 0,
        CVBS_PEDESTAL = 1,
        SVIDEO = 2,
        SVIDEO_PEDESTAL = 3,
    };

    enum PhysInput : uint8_t {
        INPUT_CVBS = 0,
        INPUT_SVIDEO = 1,
    };

    PhysInput input_to_phys[] = {
        [CVBS] = INPUT_CVBS,
        [CVBS_PEDESTAL] = INPUT_CVBS,
        [SVIDEO] = INPUT_SVIDEO,
        [SVIDEO_PEDESTAL] = INPUT_SVIDEO,
    };

    bool input_to_pedestal[] = {
        [CVBS] = false,
        [CVBS_PEDESTAL] = true,
        [SVIDEO] = false,
        [SVIDEO_PEDESTAL] = true,
    };

    // The layout of this struct must remain compatible!
    // This means that the any changes to the layout can
    // only be done past the field min_read_version.
    struct SettingsHeader {
        char magic[8];
        uint32_t length;
        uint16_t version;
        uint16_t min_read_version;
        uint32_t checksum;
    } __attribute__((packed));
    static_assert(sizeof(SettingsHeader) == 20,
        "SettingsHeader size is wrong!");

    struct ConvSettings {
        SettingsHeader hdr;
        Input default_input;
        uint8_t smoothing;
        uint8_t disable_free_run;
        uint8_t padding;
        uint32_t checksum;
    } __attribute__((packed));
    static_assert(sizeof(ConvSettings) == 28, "ConvSettings size is wrong!");

    class KoryuuSettings {
    public:
        ConvSettings settings;
    private:
        /* EEMEM */ ConvSettings *const eeprom_settings;
        bool dirty;

    public:
        KoryuuSettings(/* EEMEM */ ConvSettings *const eep_s)
                : // settings({ { { '\0' }, 0, 0, 0 }, 0, 0, 0 }),
                eeprom_settings(eep_s), dirty(false)
        {
            SettingsHeader &tmp_hdr = settings.hdr;
            bool valid = true;
            eeprom_read_block(&tmp_hdr, eep_s, sizeof(tmp_hdr));
            for (uint8_t i = 0; i < sizeof(SETTINGS_MAGIC); ++i)
                if (tmp_hdr.magic[i] != SETTINGS_MAGIC[i]) {
                    valid = false;
                    break;
                }

            if (valid && CURR_VERSION < tmp_hdr.min_read_version)
                valid = false;

            if (valid) {
                uint32_t hdr_checksum =
                    crc::crc32(&tmp_hdr, offsetof(typeof_(tmp_hdr), checksum));
                if (hdr_checksum != tmp_hdr.checksum)
                    valid = false;
            }

            if (valid) {
                uint32_t checksum;
                size_t checksum_ofs;
                autounion<ConvSettings, true, 2 * sizeof(ConvSettings)> s_u;

                // Check for ridiculous length
                if (tmp_hdr.length > 2 * sizeof(ConvSettings))
                    valid = false;
                else
                    eeprom_read_block(&s_u[0], eep_s, tmp_hdr.length);

                if (valid && tmp_hdr.length == sizeof(ConvSettings)) {
                    settings = s_u;
                    checksum_ofs = offsetof(typeof_(settings), checksum);
                    checksum = settings.checksum;
                }
                else if (valid) {
                    const uint8_t *const p = &s_u[0];
                    checksum_ofs = tmp_hdr.length - sizeof(uint32_t);
                    memcpy(&checksum, p + checksum_ofs, sizeof(uint32_t));

                    // This intentionally "slices" a read-compatible
                    // settings struct.
                    settings = s_u;
                }

                if (valid) {
                    uint32_t checksum_expected =
                        crc::crc32(&s_u[0], checksum_ofs);
                    if (checksum != checksum_expected)
                        valid = false;
                    if (checksum_ofs != offsetof(typeof_(settings), checksum)) {
                        // Set the checksum of the in-memory struct
                        // in case it had a different, while
                        // read-compatible layout.
                        settings.checksum = checksum_expected;
                        dirty = true;
                    }
                }
            }

            if (!valid) {
                /* Reinitialize */
                memcpy(&settings.hdr.magic,
                    SETTINGS_MAGIC, sizeof(SETTINGS_MAGIC));
                settings.hdr.length = sizeof(settings);
                settings.hdr.version = CURR_VERSION;
                settings.hdr.min_read_version = MIN_READ_VERSION;
                settings.default_input = CVBS;
                settings.smoothing = 0x00;
                settings.disable_free_run = 0x00;
                settings.padding = 0x00;
                dirty = true;
            }
            else {
                // Validate known fields.
                if (settings.default_input > SVIDEO_PEDESTAL) {
                    settings.default_input = CVBS;
                    dirty = true;
                }
                if (settings.smoothing > 0x01) {
                    settings.smoothing = 0x00;
                    dirty = true;
                }
                if (settings.disable_free_run > 0x01) {
                    settings.disable_free_run = 0x00;
                    dirty = true;
                }
                if (settings.padding != 0x00) {
                    settings.padding = 0x00;
                    dirty = true;
                }
            }
        }

        YAAL_INLINE("KoryuuSettings::is_dirty()")
        bool is_dirty() const {
            return dirty;
        }

        YAAL_INLINE("KoryuuSettings::set_dirty()")
        void set_dirty() {
            dirty = true;
        }

        void write() {
            if (dirty) {
                settings.hdr.length = sizeof(settings);
                settings.hdr.version = CURR_VERSION;
                settings.hdr.min_read_version = MIN_READ_VERSION;
                settings.hdr.checksum =
                    crc::crc32(&settings.hdr,
                        offsetof(typeof_(settings.hdr), checksum));
                settings.checksum =
                    crc::crc32(&settings,
                        offsetof(typeof_(settings), checksum));
                eeprom_update_block(&settings,
                    eeprom_settings, sizeof(settings));
                dirty = false;
            }
        }
    };
#ifdef typeof_
#undef typeof_
#endif
}
#endif // __YAAL__
#endif // KORYUU_SETTINGS_HH
