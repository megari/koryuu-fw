#ifndef KORYUU_SETTINGS_HH
#define KORYUU_SETTINGS_HH

#include <yaal/requirements.hh>
#include <yaal/types/autounion.hh>

#ifdef __YAAL__
#include <avr/eeprom.h>
#include <string.h>
#include "crc32.hh"

namespace koryuu_settings {
    constexpr char SETTINGS_MAGIC[8] =
        { 'K', 'R', 'Y', 'U', 'C', 'O', 'N', 'S' };
    constexpr uint16_t CURR_VERSION = 0x0001;

    enum Input : uint8_t {
        CVBS,
        CVBS_PEDESTAL,
        SVIDEO,
        SVIDEO_PEDESTAL,
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

    struct SettingsHeader {
        char magic[8];
        uint32_t length;
        uint16_t version;
        uint16_t min_read_version;
        uint32_t checksum;
    };

    struct ConvSettings {
        SettingsHeader hdr;
        Input default_input;
        uint8_t smoothing;
        uint32_t checksum;
    };

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
            SettingsHeader tmp_hdr;
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
                    crc::crc32(&tmp_hdr,
                        offsetof(decltype(tmp_hdr), checksum));
                if (hdr_checksum != tmp_hdr.checksum)
                    valid = false;
            }
            if (valid) {
                uint32_t checksum, checksum_expected;
                size_t checksum_ofs;
                // TODO: check for ridiculous length
                eeprom_read_block(&settings, eep_s, tmp_hdr.length);
                if (tmp_hdr.length == sizeof(ConvSettings)) {
                    checksum_ofs = offsetof(decltype(settings), checksum);
                    checksum = settings.checksum;
                }
                else {
                    const uintptr_t p_ =
                        reinterpret_cast<uintptr_t>(&settings);
                    const uint8_t *const p =
                        reinterpret_cast<const uint8_t*>(p_);
                    checksum_ofs = tmp_hdr.length - sizeof(uint32_t);
                    memcpy(&checksum, p + checksum_ofs, sizeof(uint32_t));
                }
                checksum_expected =
                    crc::crc32(&settings, checksum_ofs);
                if (checksum != checksum_expected)
                    valid = false;
            }

            if (!valid) {
                /* Reinitialize */
                memcpy(&settings.hdr.magic,
                    SETTINGS_MAGIC, sizeof(SETTINGS_MAGIC));
                settings.hdr.length = sizeof(settings);
                settings.hdr.version = CURR_VERSION;
                settings.hdr.min_read_version = CURR_VERSION;
                settings.hdr.checksum =
                    crc::crc32(&settings.hdr,
                        offsetof(decltype(settings.hdr), checksum));
                settings.default_input = CVBS;
                settings.smoothing = 0;
                settings.checksum =
                    crc::crc32(&settings,
                        offsetof(decltype(settings), checksum));
                dirty = true;
            }
        }

        bool is_dirty() const {
            return dirty;
        }

        void set_dirty() {
                dirty = true;
        }

        void write() {
            if (dirty) {
                eeprom_update_block(&settings,
                    eeprom_settings,
                    sizeof(settings));
                dirty = false;
            }
        }
    };
}
#endif // __YAAL__
#endif // KORYUU_SETTINGS_HH
