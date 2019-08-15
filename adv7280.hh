#ifndef ADV7280_HH
#define ADV7280_HH
#include <yaal/requirements.hh>

#ifdef __YAAL__
#include <yaal/io/ports.hh>
#include <yaal/communication/i2c_hw.hh>

#include "i2c_helpers.hh"

#define AVGLPF 0

namespace ad_decoder {
    using namespace yaal;
    using namespace i2c_helpers;

    enum InputSelection : uint8_t {
        INSEL_CVBS_Ain1      = 0x00,
        INSEL_CVBS_Ain2      = 0x01,
        INSEL_CVBS_Ain3      = 0x02,
        INSEL_CVBS_Ain4      = 0x03,
        INSEL_YC_Ain1_2      = 0x08,
        INSEL_YC_Ain3_4      = 0x09,
        INSEL_YPbPr_Ain1_2_3 = 0x0c,
    };

    enum AutoDetectSelection : uint8_t {
        AD_PALBGHID_NTSCJ_SECAM = 0x04,
        AD_PALBGHID_NTSCM_SECAM = 0x14,
        AD_PALN_NTSCJ_SECAM     = 0x24,
        AD_PALN_NTSCM_SECAM     = 0x34,
        AD_NTSCJ                = 0x44,
        AD_NTSCM                = 0x54,
        AD_PAL60                = 0x64,
        AD_NTSC443              = 0x74,
        AD_PALBGHID             = 0x84,
        AD_PALN                 = 0x94,
        AD_PALM                 = 0xa4,
        AD_PALM_P               = 0xb4,
        AD_PAL_COMBI_N          = 0xc4,
        AD_PAL_COMBI_N_P        = 0xd4,
        AD_SECAM                = 0xe4,
        AD_SECAM2               = 0xf4,
    };

    enum DecoderSubmap : uint8_t {
        DEC_SUBMAP_USER     = 0x00,
        DEC_SUBMAP_INTR_VDP = 0x20,
        DEC_SUBMAP_USER2    = 0x40,
        DEC_SUBMAP_0x80     = 0x80,
    };

    enum AlphaBlend : uint8_t {
        AB_SHARPEST  = 0x00,
        AB_SHARP     = 0x04,
        AB_SMOOTH    = 0x08,
        AB_SMOOTHEST = 0x0c,
    };

    enum I2P_Algorithm : uint8_t {
        I2P_ALG_LINEDOUBLE  = 0x00,
        I2P_ALG_DEINTERLACE = 0x18,
    };

    enum InterruptDriveLevel : uint8_t {
        IDL_OPEN_DRAIN  = 0x00,
        IDL_ACTIVE_LOW  = 0x01,
        IDL_ACTIVE_HIGH = 0x02,
    };

    enum InterruptDuration : uint8_t {
        ID_3_XTAL     = 0x00,
        ID_15_XTAL    = 0x40,
        ID_63_XTAL    = 0x80,
        ID_MUST_CLEAR = 0xc0,
    };

    constexpr uint8_t OUTC_TOD    = 0x40;
    constexpr uint8_t OUTC_VBI_EN = 0x80;

    constexpr uint8_t EOUTC_RANGE_LIM   = 0x00;
    constexpr uint8_t EOUTC_RANGE_FULL  = 0x01;
    constexpr uint8_t EOUTC_SFL_EN      = 0x02;
    constexpr uint8_t EOUTC_BLANK_C_VBI = 0x04;
    constexpr uint8_t EOUTC_TIM_OE      = 0x08;
    constexpr uint8_t EOUTC_BT656_4     = 0x80;

    constexpr uint8_t PWRM_PWRDWN = 0x20;
    constexpr uint8_t PWRM_RESET  = 0x80;

    constexpr uint8_t CTIDNRC_CTI_EN    = 0x01;
    constexpr uint8_t CTIDNRC_CTI_AB_EN = 0x02;
    constexpr uint8_t CTIDNRC_DNR_EN    = 0x20;

    constexpr uint8_t AFEC_F1_EN   = 0x01;
    constexpr uint8_t AFEC_F2_EN   = 0x02;
    constexpr uint8_t AFEC_F3_EN   = 0x04;
    constexpr uint8_t AFEC_F4_EN   = 0x08;
    constexpr uint8_t AFEC_MAN_OVR = 0x10;

    template<typename INTRQ, typename RESET, typename PWRDWN>
    class ADV7280A {
    public:
        INTRQ intrq;
        RESET reset;
        PWRDWN pwrdwn;

        uint8_t address;
        uint8_t vpp_address;

        ADV7280A(uint8_t addr, uint8_t vpp_addr = 0x84)
            : address(addr), vpp_address(vpp_addr) {
            intrq.mode = INPUT_PULLUP;
            reset.mode = OUTPUT;
            reset = false;
            pwrdwn.mode = OUTPUT;
            pwrdwn = false;
        }

        void select_input(InputSelection input) {
            I2C_WRITE(address, 0x00, (uint8_t)input);
        }

        void select_autodetection(AutoDetectSelection ad) {
            I2C_WRITE(address, 0x02, (uint8_t)ad);
        }

        void select_submap(DecoderSubmap sm) {
            I2C_WRITE(address, 0x0e, (uint8_t)sm);
        }

        void set_output_control(bool tristate_outputs, bool enable_vbi) {
            uint8_t outc = 0x0c;
            if (tristate_outputs)
                outc |= OUTC_TOD;
            if (enable_vbi)
                outc |= OUTC_VBI_EN;
            I2C_WRITE(address, 0x03, outc);
        }

        void set_ext_output_control(bool full_range, bool enable_sfl,
                bool blank_chroma_vbi, bool enable_timing_out, bool bt656_4) {
            uint8_t ext_outc = 0x30;
            if (full_range)
                ext_outc |= EOUTC_RANGE_FULL;
            if (enable_sfl)
                ext_outc |= EOUTC_SFL_EN;
            if (blank_chroma_vbi)
                ext_outc |= EOUTC_BLANK_C_VBI;
            if (enable_timing_out)
                ext_outc |= EOUTC_TIM_OE;
            if (bt656_4)
                ext_outc |= EOUTC_BT656_4;
            I2C_WRITE(address, 0x04, ext_outc);
        }

        void set_power_management(bool powerdown, bool reset) {
            uint8_t pwr_mgmt = 0x00;
            if (powerdown)
                pwr_mgmt |= PWRM_PWRDWN;
            if (reset)
                pwr_mgmt |= PWRM_RESET;

            // An I2C failure is expected here, as the chip resets.
            I2C_WRITE<false>(address, 0x0f, pwr_mgmt);
        }

        void set_cti_dnr_control(bool enable_cti, bool enable_cti_ab, AlphaBlend ab, bool enable_dnr) {
            // CTI DNR control
            uint8_t cti_dnr[] = { 0x4d, 0xc0 };
            if (enable_cti)
                cti_dnr[1] |= CTIDNRC_CTI_EN;
            if (enable_cti_ab)
                cti_dnr[1] |= CTIDNRC_CTI_AB_EN;
            cti_dnr[1] |= ab;
            if (enable_dnr)
                cti_dnr[1] |= CTIDNRC_DNR_EN;
            I2c_HW.write(address, cti_dnr, cti_dnr + sizeof(cti_dnr));
        }

        void deinterlace_reset() {
            // TODO
            // VPP slave address: User sub map, subaddress 0xfd, bits 6:0
            // VPP Map, subaddress 0x41, bit 0
        }

        void deinterlace_control(bool enable,
                I2P_Algorithm alg = I2P_ALG_DEINTERLACE) {
            // TODO
            // VPP slave address: User sub map, subaddress 0xfd, bits 6:0
            // Deinterlace enable: VPP Map, subaddress 0x55, bit 7
            // Algorithm selection (undocumented): VPP Map, subaddress 0x5a, bits 4:3
            //     0b00: Simple line doubling
            //     0b11: Proprietary deinterlacing algorithm
            // Advanced timing mode: VPP Map, subaddress 0x5b, bit 7 (negative)
        }

#if AVGLPF
        void set_lpf(bool enable_lpf, uint8_t cutoff) {
            uint8_t lpf[] = { 0xe6, 0x00 };
            if (enable_lpf)
                lpf[1] |= 0x02;
            lpf[1] |= (cutoff & 0x07) << 2;
            select_submap(DEC_SUBMAP_USER2);
            I2c_HW.write(address, lpf, lpf + sizeof(lpf));
            select_submap(DEC_SUBMAP_USER);
        }
#endif
        void set_aa_filters(bool man_ovr, bool f1, bool f2, bool f3, bool f4) {
            uint8_t afec[] = { 0xf3, 0x00 };
            if (man_ovr)
                afec[1] |= AFEC_MAN_OVR;
            if (f1)
                afec[1] |= AFEC_F1_EN;
            if (f2)
                afec[1] |= AFEC_F2_EN;
            if (f3)
                afec[1] |= AFEC_F3_EN;
            if (f4)
                afec[1] |= AFEC_F4_EN;
            I2c_HW.write(address, afec, afec + sizeof(afec));
        }

        void set_interrupt_config(InterruptDriveLevel idl, bool manual_mode,
                uint8_t mvirq_sel, InterruptDuration duration,
                bool setup_submap = true)
        {
            if (setup_submap)
                select_submap(DEC_SUBMAP_INTR_VDP);

            uint8_t intrcfg[] = { 0x40, 0x00 };
            intrcfg[1] |= idl | duration | mvirq_sel;
            if (manual_mode)
                intrcfg[1] |= 0x04;
            I2c_HW.write(address, intrcfg, intrcfg + sizeof(intrcfg));

            if (setup_submap)
                select_submap(DEC_SUBMAP_USER);
        }

        void interrupt_clear1(bool clear_sd_lock, bool clear_sd_unlock,
                bool clear_freerun_change, bool clear_mv_ps_cs,
                bool setup_submap = true)
        {
            if (setup_submap)
                select_submap(DEC_SUBMAP_INTR_VDP);

            uint8_t iclr1[] = { 0x43, 0x00 };
            if (clear_sd_lock)
                iclr1[1] |= 0x01;
            if (clear_sd_unlock)
                iclr1[1] |= 0x02;
            if (clear_freerun_change)
                iclr1[1] |= 0x20;
            if (clear_mv_ps_cs)
                iclr1[1] |= 0x40;
            I2c_HW.write(address, iclr1, iclr1 + sizeof(iclr1));

            if (setup_submap)
                select_submap(DEC_SUBMAP_USER);
        }

        void set_interrupt_mask1(bool unmask_sd_lock, bool unmask_sd_unlock,
                bool unmask_freerun_change, bool unmask_mv_ps_cs,
                bool setup_submap = true)
        {
            if (setup_submap)
                select_submap(DEC_SUBMAP_INTR_VDP);

            uint8_t imsk1[] = { 0x44, 0x00 };
            if (unmask_sd_lock)
                imsk1[1] |= 0x01;
            if (unmask_sd_unlock)
                imsk1[1] |= 0x02;
            if (unmask_freerun_change)
                imsk1[1] |= 0x20;
            if (unmask_mv_ps_cs)
                imsk1[1] |= 0x40;
            I2c_HW.write(address, imsk1, imsk1 + sizeof(imsk1));

            if (setup_submap)
                select_submap(DEC_SUBMAP_USER);
        }

        void interrupt_clear3(bool clear_sd_op_change,
                bool clear_sd_vsync_lock_change,
                bool clear_sd_hsync_lock_change,
                bool clear_sd_ad_result_change,
                bool clear_secam_lock_change,
                bool clear_pal_sw_lock_change,
                bool setup_submap = true)
        {
            if (setup_submap)
                select_submap(DEC_SUBMAP_INTR_VDP);

            uint8_t iclr3[] = { 0x4b, 0x00 };
            if (clear_sd_op_change)
                iclr3[1] |= 0x01;
            if (clear_sd_vsync_lock_change)
                iclr3[1] |= 0x02;
            if (clear_sd_hsync_lock_change)
                iclr3[1] |= 0x04;
            if (clear_sd_ad_result_change)
                iclr3[1] |= 0x08;
            if (clear_secam_lock_change)
                iclr3[1] |= 0x10;
            if (clear_pal_sw_lock_change)
                iclr3[1] |= 0x20;
            I2c_HW.write(address, iclr3, iclr3 + sizeof(iclr3));

            if (setup_submap)
                select_submap(DEC_SUBMAP_USER);
        }

        void set_interrupt_mask3(bool unmask_sd_op_change,
                bool unmask_sd_vsync_lock_change,
                bool unmask_sd_hsync_lock_change,
                bool unmask_sd_ad_result_change,
                bool unmask_secam_lock_change,
                bool unmask_pal_sw_lock_change,
                bool setup_submap = true)
        {
            if (setup_submap)
                select_submap(DEC_SUBMAP_INTR_VDP);

            uint8_t imsk3[] = { 0x4c, 0x00 };
            if (unmask_sd_op_change)
                imsk3[1] |= 0x01;
            if (unmask_sd_vsync_lock_change)
                imsk3[1] |= 0x02;
            if (unmask_sd_hsync_lock_change)
                imsk3[1] |= 0x04;
            if (unmask_sd_ad_result_change)
                imsk3[1] |= 0x08;
            if (unmask_secam_lock_change)
                imsk3[1] |= 0x10;
            if (unmask_pal_sw_lock_change)
                imsk3[1] |= 0x20;
            I2c_HW.write(address, imsk3, imsk3 + sizeof(imsk3));

            if (setup_submap)
                select_submap(DEC_SUBMAP_USER);
        }
    };
}

#endif
#endif
