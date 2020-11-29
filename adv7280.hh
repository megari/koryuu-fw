#ifndef ADV7280_HH
#define ADV7280_HH

#include <yaal/io/ports.hh>
#include <yaal/communication/i2c_hw.hh>

#define AVGLPF 0

namespace ad_decoder {
    using namespace yaal;

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

    enum DetectedVideoType : uint8_t {
        NTSC_MJ   = 0x00,
        NTSC_443  = 0x01,
        PAL_M     = 0x02,
        PAL_60    = 0x03,
        PAL_BGHID = 0x04,
        SECAM     = 0x05,
        PAL_CN    = 0x06,
        SECAM_525 = 0x07,
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
            : address(addr), vpp_address(vpp_addr >> 1) {
            intrq.mode = INPUT_PULLUP;
            reset.mode = OUTPUT;
            reset = false;
            pwrdwn.mode = OUTPUT;
            pwrdwn = false;
        }

        void select_input(InputSelection input) {
            uint8_t insel[] = { 0x00, input };
            I2c_HW.write_multi(address, insel, insel + sizeof(insel));
        }

        void select_autodetection(AutoDetectSelection ad) {
            uint8_t autodetect[] = { 0x02, ad };
            I2c_HW.write_multi(address, autodetect, autodetect + sizeof(autodetect));
        }

        void select_submap(DecoderSubmap sm) {
            uint8_t submap[] = { 0x0e, sm };
            I2c_HW.write_multi(address, submap, submap + sizeof(submap));
        }

        void set_output_control(bool tristate_outputs, bool enable_vbi) {
            uint8_t outc[] = { 0x03, 0x0c };
            if (tristate_outputs)
                outc[1] |= OUTC_TOD;
            if (enable_vbi)
                outc[1] |= OUTC_VBI_EN;
            I2c_HW.write_multi(address, outc, outc + sizeof(outc));
        }

        void set_ext_output_control(bool full_range, bool enable_sfl,
                bool blank_chroma_vbi, bool enable_timing_out, bool bt656_4) {
            //uint8_t ext_outc[] = { 0x04, 0x30 };
            uint8_t ext_outc[] = { 0x04, 0x00 };
            if (full_range)
                ext_outc[1] |= EOUTC_RANGE_FULL;
            if (enable_sfl)
                ext_outc[1] |= EOUTC_SFL_EN;
            if (blank_chroma_vbi)
                ext_outc[1] |= EOUTC_BLANK_C_VBI;
            if (enable_timing_out)
                ext_outc[1] |= EOUTC_TIM_OE;
            if (bt656_4)
                ext_outc[1] |= EOUTC_BT656_4;
            I2c_HW.write_multi(address, ext_outc, ext_outc + sizeof(ext_outc));
        }

        void set_power_management(bool powerdown, bool reset) {
            uint8_t pwr_mgmt[] = { 0x0f, 0x00 };
            if (powerdown)
                pwr_mgmt[1] |= PWRM_PWRDWN;
            if (reset)
                pwr_mgmt[1] |= PWRM_RESET;
            I2c_HW.write_multi(address, pwr_mgmt, pwr_mgmt + sizeof(pwr_mgmt));
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
            I2c_HW.write_multi(address, cti_dnr, cti_dnr + sizeof(cti_dnr));
        }

        void deinterlace_reset() {
            // VPP slave address: User sub map, subaddress 0xfd, bits 7:1
            uint8_t vpp_sla[] = { 0xfd, vpp_address << 1 };
            I2c_HW.write_multi(address, vpp_sla, vpp_sla + sizeof(vpp_sla));

            // VPP Map, subaddress 0x41, bit 0
            uint8_t vpp_rst[] = { 0x41, 0x01 };
            I2c_HW.write_multi(vpp_address, vpp_rst, vpp_rst + sizeof(vpp_rst));
        }

        void deinterlace_control(bool enable,
                I2P_Algorithm alg = I2P_ALG_DEINTERLACE) {

            // VPP slave address: User sub map, subaddress 0xfd, bits 7:1
            uint8_t vpp_sla[] = { 0xfd, vpp_address << 1 };
            I2c_HW.write_multi(address, vpp_sla, vpp_sla + sizeof(vpp_sla));

            // ADI required write
            uint8_t adi_req[] = { 0xa3, 0x00 };
            I2c_HW.write_multi(vpp_address, adi_req, adi_req + sizeof(adi_req));

            // Advanced timing mode: VPP Map, subaddress 0x5b, bit 7 (negative)
            uint8_t atm_enable[] = { 0x5b, enable ? 0x00 : 0x80 };
            I2c_HW.write_multi(vpp_address, atm_enable, atm_enable + sizeof(atm_enable));

            // Deinterlace enable: VPP Map, subaddress 0x55, bit 7
            uint8_t i2p_enable[] = { 0x55, enable ? 0x80 : 0x00 };
            I2c_HW.write_multi(vpp_address, i2p_enable, i2p_enable + sizeof(i2p_enable));

            // Algorithm selection (undocumented): VPP Map, subaddress 0x5a, bits 4:3
            //     0b00: Simple line doubling
            //     0b11: Proprietary deinterlacing algorithm
#if 1
            uint8_t i2p_alg[] = { 0x5a, alg };
            I2c_HW.write_multi(vpp_address, i2p_alg, i2p_alg + sizeof(i2p_alg));
#endif
        }

#if AVGLPF
        void set_lpf(bool enable_lpf, uint8_t cutoff) {
            uint8_t lpf[] = { 0xe6, 0x00 };
            if (enable_lpf)
                lpf[1] |= 0x02;
            lpf[1] |= (cutoff & 0x07) << 2;
            select_submap(DEC_SUBMAP_USER2);
            I2c_HW.write_multi(address, lpf, lpf + sizeof(lpf));
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
            I2c_HW.write_multi(address, afec, afec + sizeof(afec));
        }

        void set_interrupt_config(InterruptDriveLevel idl, bool manual_mode,
                InterruptDuration duration, bool setup_submap = true)
        {
            if (setup_submap)
                select_submap(DEC_SUBMAP_INTR_VDP);

            uint8_t intrcfg[] = { 0x40, 0x10 };
            intrcfg[1] |= idl | duration;
            if (manual_mode)
                intrcfg[1] |= 0x04;
            I2c_HW.write_multi(address, intrcfg, intrcfg + sizeof(intrcfg));

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
            I2c_HW.write_multi(address, iclr1, iclr1 + sizeof(iclr1));

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
            I2c_HW.write_multi(address, imsk1, imsk1 + sizeof(imsk1));

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
            I2c_HW.write_multi(address, iclr3, iclr3 + sizeof(iclr3));

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
            I2c_HW.write_multi(address, imsk3, imsk3 + sizeof(imsk3));

            if (setup_submap)
                select_submap(DEC_SUBMAP_USER);
        }
    };
}

#endif
