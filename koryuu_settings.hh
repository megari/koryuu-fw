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
    constexpr uint16_t CURR_VERSION = 0x0002u;
    constexpr uint16_t MIN_READ_VERSION = 0x0001u;

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

    // The layout of this struct must not change!
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
        bool downgrade;

    public:
        KoryuuSettings(/* EEMEM */ ConvSettings *const eep_s)
                : eeprom_settings(eep_s), dirty(false), downgrade(false)
        {
            static autounion<ConvSettings, true, 2 * sizeof(ConvSettings)> s_u;
            SettingsHeader &tmp_hdr = settings.hdr;
            static constexpr size_t def_checksum_ofs =
                offsetof(typeof_(settings), checksum);
            bool valid = true;
            eeprom_read_block(&tmp_hdr, eep_s, sizeof(tmp_hdr));
            for (uint8_t i = 0; i < sizeof(SETTINGS_MAGIC); ++i)
                if (tmp_hdr.magic[i] != SETTINGS_MAGIC[i]) {
                    valid = false;
                    break;
                }

            if (valid && CURR_VERSION < tmp_hdr.min_read_version) {
                valid = false;
                downgrade = true;
            }
            else if (valid && CURR_VERSION != tmp_hdr.version) {
                // Even if there are no changes needed to the data,
                // we need to update the version fields on next write.
                dirty = true;
                if (CURR_VERSION < tmp_hdr.version)
                    // If the original settings were of a newer version,
                    // set a flag indicating this to help the firmware make
                    // an informed decision of when to write the settings.
                    downgrade = true;
            }

            if (valid) {
                uint32_t hdr_checksum =
                    crc::crc32(&tmp_hdr, offsetof(typeof_(tmp_hdr), checksum));
                if (hdr_checksum != tmp_hdr.checksum)
                    valid = false;
            }

            if (valid) {
                uint32_t checksum;
                size_t checksum_ofs;

                // Check for ridiculous length
                if (tmp_hdr.length > 2 * sizeof(ConvSettings))
                    valid = false;
                else
                    eeprom_read_block(&s_u[0], eep_s, tmp_hdr.length);

                if (valid && tmp_hdr.length == sizeof(ConvSettings)) {
                    settings = s_u.value();
                    checksum_ofs = def_checksum_ofs;
                    checksum = settings.checksum;
                }
                else if (valid) {
                    const uint8_t *const p = &s_u[0];
                    checksum_ofs = tmp_hdr.length - sizeof(uint32_t);
                    memcpy(&checksum, p + checksum_ofs, sizeof(uint32_t));

                    // This intentionally "slices" a read-compatible
                    // settings struct.
                    settings = s_u.value();
                }

                if (valid) {
                    uint32_t checksum_expected =
                        crc::crc32(&s_u[0], checksum_ofs);
                    if (checksum != checksum_expected) {
                        valid = false;
                    }
                    if (checksum_ofs != def_checksum_ofs) {
                        // Re-calculate the checksum of the settings
                        // struct read from the EEPROM if it had a
                        // different, while read-compatible layout.
                        settings.checksum =
                            crc::crc32(&settings, def_checksum_ofs);
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
                settings.smoothing = 0x00u;
                settings.disable_free_run = 0x00u;
                settings.padding = 0x00u;
                dirty = true;
            }
            else {
                // Validate known fields.
                if (settings.default_input > SVIDEO_PEDESTAL) {
                    settings.default_input = CVBS;
                    dirty = true;
                }
                if (settings.smoothing > 0x01u) {
                    settings.smoothing = 0x00u;
                    dirty = true;
                }
                if (settings.disable_free_run > 0x01u) {
                    settings.disable_free_run = 0x00u;
                    dirty = true;
                }
                if (settings.padding != 0x00u) {
                    settings.padding = 0x00u;
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

        YAAL_INLINE("KoryuuSettings::is_downgrading()")
        bool is_downgrading() const {
            return downgrade;
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
                downgrade = false;
            }
        }
    };

#ifdef typeof_
#undef typeof_
#endif
}
#endif // __YAAL__
#endif // KORYUU_SETTINGS_HH
