/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Driver for the zl30260 Clock Generator as used
 * in the Thunderscope.  Datasheet reference at 
 * https://ww1.microchip.com/downloads/aemDocuments/documents/TCG/ProductDocuments/DataSheets/ZL30260-1-2-3-1-APLL-6-or-10-Output-Any-to-Any-Clock-Multiplier-and-Frequency-Synthesizer-DS20006554.pdf
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#include "mcp_zl3026x.h"
#include "mcp_clkgen.h"

#include "util.h"
#include "ts_common.h"

#include <math.h>


#define MCP_ADD_REG_WRITE(conf, reg, val)   { (conf)->action = MCP_CLKGEN_WRITE_REG; \
                                              (conf)->addr = (reg); \
                                              (conf)->value = (val); }

#define MCP_ADD_DELAY(conf, delay)          { (conf)->action = MCP_CLKGEN_DELAY; \
                                              (conf)->delay_us = (delay); }

static const uint8_t g_outClkGroups[ZL3026X_NUM_OUTPUT_CLK] = {0, 0, 1, 2, 2, 3, 3, 4, 5, 5};

static uint64_t mcp_zl3026x_selected_input_freq(zl3026x_clk_config_t *conf);

int32_t mcp_zl3026x_build_config(mcp_clkgen_conf_t* confData, uint32_t len, zl3026x_clk_config_t conf)
{
    int32_t calLen = 0;
    uint32_t scaled_in_freq = 0;
    // Reference App Note ZLAN-590
    // https://ww1.microchip.com/downloads/aemDocuments/documents/TCG/ApplicationNotes/ApplicationNotes/ConfigurationSequenceZLAN-590.pdf

    //ZL3026x Init
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0423, 0x08);
    calLen++;

    // Enable Outputs by setting OCEN1 and OCEN2
    uint16_t out_ch_bitmap = 0;
    for(uint8_t ch = 0; ch < ZL3026X_NUM_OUTPUT_CLK; ch++)
    {
        if(conf.out_clks[ch].enable)
        {
            out_ch_bitmap |= (1UL << ch);
        }
    }
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0005, (out_ch_bitmap & 0xff));
    calLen++;
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0006, ((out_ch_bitmap >> 8) & 0xFF));
    calLen++;

    // Stop Outputs
    for(uint8_t ch = 0; ch < ZL3026X_NUM_OUTPUT_CLK; ch++)
    {
        if(conf.out_clks[ch].enable)
        {
            MCP_ADD_REG_WRITE(&confData[calLen], (0x20A + (0x10*ch)), 0x0C);
            calLen++;
        }
    }
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0009, (out_ch_bitmap & 0xff));
    calLen++;
    MCP_ADD_REG_WRITE(&confData[calLen], 0x000A, ((out_ch_bitmap >> 8) & 0xFF));
    calLen++;

    // Configure Input Clock Source:
    // Enable/Disable XA/XB Inputs
    if(conf.in_xo.enable)
    {
        // TODO
    }
    // Enable/Disable Input Clocks
    else
    {
        uint8_t in_ch_bitmap = 0;
        for(uint8_t ch = 0; ch < ZL3026X_NUM_INPUT_CLK; ch++)
        {
            if(conf.in_clks[ch].enable)
            {
                LOG_DEBUG("Input Clock %d: Freq %u Hz, Div %d", ch, conf.in_clks[ch].input_freq, conf.in_clks[ch].input_divider);
                MCP_ADD_REG_WRITE(&confData[calLen], (0x0303+ch), (uint8_t)conf.in_clks[ch].input_divider | ZL3026X_VALTIME_DEFAULT);
                calLen++;
                in_ch_bitmap |= (1 << ch);
                scaled_in_freq = conf.in_clks[ch].input_freq / (1 << conf.in_clks[ch].input_divider);
                break;
            }
        }
        if(in_ch_bitmap == 0)
        {
            //ERROR
            LOG_ERROR("Invalid Clock Config: No Input Clocks Enabled");
            return TS_STATUS_ERROR;
        }
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0004, in_ch_bitmap);
        calLen++;
    }


    // Select Source for Output Multiplexers
    uint8_t clkMuxABC = 0, clkMuxDEF = 0;
    for(uint8_t chA = 0; chA < ZL3026X_NUM_OUTPUT_CLK; chA++)
    {
        if(conf.out_clks[chA].enable)
        {
            for(uint8_t chB=0; chB < ZL3026X_NUM_OUTPUT_CLK; chB++)
            {
                if((chA != chB) && (conf.out_clks[chB].enable)
                    && (g_outClkGroups[chA] == g_outClkGroups[chB])
                    && (conf.out_clks[chA].output_pll_select != conf.out_clks[chB].output_pll_select))
                {
                    //Error
                    LOG_ERROR("Clock Configuration error: Out %d and %d are in the same group but have different PLL Configs", chA, chB);
                    return TS_STATUS_ERROR;
                }
            }
            uint8_t pll_mux = 0; // ZL3026X_PLL_INT_DIV
            if (conf.out_clks[chA].output_pll_select == ZL3026X_PLL_FRAC_DIV ||
                conf.out_clks[chA].output_pll_select == ZL3026X_PLL_BYPASS)
            {
                pll_mux = 1;
            }
            else if( conf.out_clks[chA].output_pll_select == ZL3026X_PLL_BYPASS_2)
            {
                pll_mux = 3;
            }

            if(g_outClkGroups[chA] < 3)
            {
                clkMuxABC |= (pll_mux << (2*g_outClkGroups[chA]));
            }
            else
            {
                clkMuxDEF |= (pll_mux << (2*g_outClkGroups[chA]));
            }
        }
    }
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0007, clkMuxABC);
    calLen++;
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0008, clkMuxDEF);
    calLen++;

    // Configure GPIO Pins
    //TODO

    // Determine PLL Configuration
    /** 
     * Find a common ratio for the selected output frequencies
     */
    uint64_t pll_out = 1;
    uint64_t clk_max = 1;
    uint8_t pll_frac_bypass = 0;
    for(uint8_t ch = 0; ch < ZL3026X_NUM_OUTPUT_CLK; ch++)
    {
        if(conf.out_clks[ch].enable)
        {
            if (conf.out_clks[ch].output_pll_select == ZL3026X_PLL_INT_DIV)
            {
                //Find the least-common multiple
                uint64_t freq_mult = pll_out;
                uint64_t freq_mod = conf.out_clks[ch].output_freq;
                while(freq_mod != 0)
                {
                    uint64_t freq_temp = freq_mult;
                    freq_mult = freq_mod;
                    freq_mod = freq_temp % freq_mod;
                }
                pll_out = (pll_out * conf.out_clks[ch].output_freq) / freq_mult;
            }
            else if (conf.out_clks[ch].output_pll_select == ZL3026X_PLL_BYPASS)
            {
                pll_frac_bypass = 1;
            }
        }
    }

    //Scale pll_out to be in a good range
    while(pll_out < ZL3026X_MIN_PLL_OUT)
    {
        pll_out *= 2;
    }

    // Scale pll_out to be good for the MSDIV if needed
    for(uint8_t ch = 0; ch < ZL3026X_NUM_OUTPUT_CLK; ch++)
    {
        if(conf.out_clks[ch].enable &&
            (conf.out_clks[ch].output_pll_select == ZL3026X_PLL_INT_DIV))
        {
            if((pll_out != conf.out_clks[ch].output_freq) &&
                (pll_out < ZL3026X_OUT_MDIV_MIN_CLK))
            {
                pll_out *= 2;
                break;
            }
        }
    }

    LOG_DEBUG("Calculated APLL Output Freq: %llu Hz", pll_out);

    uint64_t pll_vco = 4200000000;
    uint32_t pll_int_div = pll_vco/pll_out;
    //validate pll_int_div 4-15
    if(pll_int_div < 4 || pll_int_div > 15)
    {
        LOG_ERROR("Invalid APLL Configuration: Int Div %u", pll_int_div);
        return TS_INVALID_PARAM;
    }

    //Get exact VCO frequency
    pll_vco = pll_out * pll_int_div;
    //validate VCO between 3.7GHz and 4.2GHz
    if(pll_vco < 3700000000 || pll_vco > 4200000000)
    {
        LOG_ERROR("Invalid APLL Configuration: VCO Freq %llu", pll_vco);
        return TS_INVALID_PARAM;
    }

    double multiplier = pll_vco / mcp_zl3026x_selected_input_freq(&conf);
    int64_t pll_numerator = 0;
    int64_t pll_denominator = 1;
    //TODO - Determine N & D parameters

    // Configure APLL1
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0003, 0x01); //Enable PLL
    calLen++;

    //TODO Enable useage of the fractional divider
    uint8_t pll_conf_1 = (0x2 * pll_frac_bypass);
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0100, 0x40 | pll_conf_1); //Enable PLL
    calLen++;

    if(pll_int_div < 8)
    {
        pll_int_div = (pll_int_div - 4)*2;
    }
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0101, pll_int_div & 0x0F); //PLL Output Integer Divide
    calLen++;

    uint8_t apll_cr3 = (conf.input_select & 0x7); //PLL Input
    if(conf.alternate_select != ZL3026X_INPUT_NONE)
    {
        apll_cr3 |= 0x80 | ((conf.alternate_select & 0x07) << 3); //Enable Input Monitoring and Alternate PLL Input
    }
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0102, apll_cr3); //PLL Input
    calLen++;

    //PLL AFBDIV
    /* The PLL configures the multiplier using a fixed-precision value,
     * consisting of 9 integer bits and 33 fractional bits.
     * Split the float value into the whole number and decimal number.
     */
    uint16_t mult_int = (uint16_t)multiplier;
    uint64_t mult_frac = (uint64_t)((multiplier - (double)mult_int) * pow(2,33));
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0106, (mult_frac & 0xFF));
    calLen++;
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0107, ((mult_frac >> 8) & 0xFF));
    calLen++;
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0108, ((mult_frac >> 16) & 0xFF));
    calLen++;
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0109, ((mult_frac >> 24) & 0xFF));
    calLen++;
    MCP_ADD_REG_WRITE(&confData[calLen], 0x010A, (((mult_int << 1) & 0xFE) | ((mult_frac >> 32) & 0x01)));
    calLen++;
    MCP_ADD_REG_WRITE(&confData[calLen], 0x010B, ((mult_int >> 7) & 0x03));
    calLen++;

    //TODO: PLL AFBDEN
    //TODO: PLL AFBREM

    // Configure APLL1 Analog
    //Assume no spread-spectrum
    if(mult_frac == 0)
    {
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0120, 0xC7);
        calLen++;
        if(mcp_zl3026x_selected_input_freq(&conf) < 50000000)
        {
            MCP_ADD_REG_WRITE(&confData[calLen], 0x0121, 0x60);
        }
        else
        {
            MCP_ADD_REG_WRITE(&confData[calLen], 0x0121, 0x40);
        }
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0122, 0x7F);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0123, 0x00);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0124, 0x04);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0125, 0xB3);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0126, 0xD8);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0127, 0x90);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x423, 0x08);
        calLen++;
    }
    else if(mult_frac == 0x100000000)
    {
        if(mcp_zl3026x_selected_input_freq(&conf) < 50000000)
        {
            MCP_ADD_REG_WRITE(&confData[calLen], 0x0120, 0xC7);
        }
        else
        {
            MCP_ADD_REG_WRITE(&confData[calLen], 0x0120, 0xC3);
        }
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0121, 0x40);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0122, 0x7F);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0123, 0x00);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0124, 0x04);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0125, 0xB3);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0126, 0xD8);
        calLen++;
        
        if(mcp_zl3026x_selected_input_freq(&conf) < 50000000)
        {
            MCP_ADD_REG_WRITE(&confData[calLen], 0x0127, 0x50);
        }
        else
        {
            MCP_ADD_REG_WRITE(&confData[calLen], 0x0127, 0x90);
        }
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0423, 0x08);
        calLen++;
    }
    else
    {
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0120, 0xC7);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0121, 0x40);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0122, 0x5F);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0123, 0x00);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0124, 0x04);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0125, 0xB3);
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0126, 0x98);
        calLen++;
        
        if(mcp_zl3026x_selected_input_freq(&conf) < 80000000)
        {
            MCP_ADD_REG_WRITE(&confData[calLen], 0x0127, 0x50);
        }
        else
        {
            MCP_ADD_REG_WRITE(&confData[calLen], 0x0127, 0x90);
        }
        calLen++;
        MCP_ADD_REG_WRITE(&confData[calLen], 0x0423, 0x1B);
        calLen++;
    }

    // Configure APLL1 Fractional Output Divider
    //TODO

    // Configure APLL1 Fractional Output Divider Analog
    //TODO

    // Configure Output Divider Registers
    for(uint8_t ch = 0; ch < ZL3026X_NUM_OUTPUT_CLK; ch++)
    {
        if(conf.out_clks[ch].enable)
        {
            LOG_DEBUG("Setting Output CLK %d to %llu Hz", ch, conf.out_clks[ch].output_freq);
            /**
             * The frequency of the clock from the medium-speed divider is divided by LSDIV+1.
             *
             * The divisor is MSDIV+1. Note that if MSDIV is not set to 0 (bypass) then the maximum
             * input clock frequency to the medium-speed divider is 750 MHz and the maximum output
             * clock frequency from the medium-speed divider is 375 MHz. When MSDIV=0, the medium-speed
             * divider, phase adjust, low-speed divider, start/stop and output duty cycle adjustment
             * circuits are bypassed and the high-frequency clock signal is sent to the directly output driver. 
             */
            uint32_t out_divide = 1, out_div_low = 1;
            uint8_t out_div, out_div_med = 1;
            out_div = 0x80; //Phase align output

            uint32_t clksrc_freq = 0;
            if(conf.out_clks[ch].output_pll_select == ZL3026X_PLL_INT_DIV)
            {
                clksrc_freq = pll_out;
            }
            else if(conf.out_clks[ch].output_pll_select == ZL3026X_PLL_BYPASS)
            {
                clksrc_freq = scaled_in_freq;
            }

            if(conf.out_clks[ch].output_freq != clksrc_freq)
            {
                out_divide = clksrc_freq / conf.out_clks[ch].output_freq;
                // Note: output_freq = clksrc_freq / (div_med * div_low)
                while(out_divide >= (1 << 7))
                {
                    if(out_divide & 0x1)
                    {
                        LOG_ERROR("======== TODO: FIX OUTPUT DIVIDE =======");
                        return TS_STATUS_ERROR;
                    }
                    out_divide >>= 1;
                    out_div_low <<= 1;
                }

                out_div_med = out_divide;
                
                out_div |= (out_div_med-1) & 0x7F;
            }
            LOG_DEBUG("Setting Output CLK %d LDIV to %02X", ch, out_div_low);

            LOG_DEBUG("Setting Output CLK %d CR1 to %02X", ch, out_div);
            MCP_ADD_REG_WRITE(&confData[calLen], 0x0200+(ch*0x10), out_div);
            calLen++;
            
            LOG_DEBUG("Setting Output CLK %d mode to %02X", ch, conf.out_clks[ch].output_mode);
            MCP_ADD_REG_WRITE(&confData[calLen], 0x0201+(ch*0x10), conf.out_clks[ch].output_mode);
            calLen++;

            // Output Low-Speed Divide (25-bit)
            if(out_div_low > 1)
            {
                MCP_ADD_REG_WRITE(&confData[calLen], 0x0204+(ch*0x10), (0x10) | ((out_div_low-1) >> 24) & 0x01);
                calLen++;
                MCP_ADD_REG_WRITE(&confData[calLen], 0x0205+(ch*0x10), (out_div_low-1) & 0xFF);
                calLen++;
                MCP_ADD_REG_WRITE(&confData[calLen], 0x0206+(ch*0x10), ((out_div_low-1) >> 8) & 0xFF);
                calLen++;
                MCP_ADD_REG_WRITE(&confData[calLen], 0x0207+(ch*0x10), ((out_div_low-1) >> 16) & 0xFF);
                calLen++;
            }
        }
    }

    // APLL1 Relock Sequence
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0430, 0x0C);
    calLen++;
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0430, 0x00);
    calLen++;

    // Delay 2ms
    MCP_ADD_DELAY(&confData[calLen], 2000);
    calLen++;

    // Align APLL Outputs
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0100, pll_conf_1 & ~(0x40));
    calLen++;
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0100, 0x40 | pll_conf_1);
    calLen++;
    
    // Disable Ring Oscillator
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0001, 0x28);
    calLen++;

    //Set Interrupt Bits
    // N/A

    // Start Outputs
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0001, 0x08);
    calLen++;
   
    MCP_ADD_REG_WRITE(&confData[calLen], 0x0009, (~out_ch_bitmap & 0xff));
    calLen++;
    MCP_ADD_REG_WRITE(&confData[calLen], 0x000A, ((~out_ch_bitmap >> 8) & 0xFF));
    calLen++;

    // Delay 2ms
    MCP_ADD_DELAY(&confData[calLen], 2000);
    calLen++;

    return calLen;
}

static uint64_t mcp_zl3026x_selected_input_freq(zl3026x_clk_config_t *conf)
{
    uint64_t freq = 0;
    switch(conf->input_select) {
    case ZL3026X_INPUT_IC1:
    case ZL3026X_INPUT_IC2:
    case ZL3026X_INPUT_IC3:
        freq = conf->in_clks[conf->input_select].input_freq / (1 << conf->in_clks[conf->input_select].input_divider);
        break;
    case ZL3026X_INPUT_XO:
        freq = conf->in_xo.xo_freq;
        break;
    case ZL3026X_INPUT_XO_DBL:
        freq = conf->in_xo.xo_freq*2;
        break;
    }
    return freq;
}
