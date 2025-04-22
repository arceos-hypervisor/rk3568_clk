//!  Clock and Reset Unit(CRU)
//!
//! # Overview
//!
//! The CRU is an APB slave module that is designed for generating all of the internal and 
//! system clocks, resets in the chip. CRU generates system clocks from PLL output clock or 
//! external clock source, and generates system reset from external power-on-reset, watchdog 
//! timer reset or software reset or temperature sensor.
//! The CRU comprises with: 
//! - PLL 
//! - Register configuration unit 
//! - Clock generate unit 
//! - Reset generate unit 
//! 
//! # Function Description
//! 
//! There are 6 fractional PLLs in RK3568: APLL, PPLL, HPLL, DPLL, CPLL and GPLL.There are 
//! also 3 integer PLLs: MPLL, NPLL and VPLL. Each PLL can only receive 24MHz oscillator as 
//! input reference clock and can be set to three work modes: normal mode, slow mode and 
//! deep slow mode. When power on or changing PLL setting, we must program PLL into slow 
//! mode or deep slow mode. 
//! To maximize the flexibility, some of clocks can select divider source from multiple PLLs.To 
//! provide some specific frequency, another solution is integrated: fractional divider. Divfree50
//! divider and divfreeNP5 divider are also provided for some modules.All clocks can be gated 
//! by software.

// the bit feild for CRU_APLL_CON0
pub const CRU_APLL_WR_EN_POS: u32 = 16;
pub const CRU_APLL_WR_EN_MASK: u32 = 0x01 << CRU_APLL_WR_EN_POS;
pub const CRU_APLL_WR_EN: u32 = CRU_APLL_WR_EN_MASK;
pub const CRU_APLL_BYPASS_POS: u32 = 15;
pub const CRU_APLL_BYPASS_MASK: u32 = 0x01 << CRU_APLL_BYPASS_POS;
pub const CRU_APLL_BYPASS: u32 = CRU_APLL_BYPASS_MASK;
pub const CRU_APLL_POSTDIV1_POS: u32 = 12;
pub const CRU_APLL_POSTDIV1_MASK: u32 = 0x03 << CRU_APLL_POSTDIV1_POS;
pub const CRU_APLL_POSTDIV1: u32 = CRU_APLL_POSTDIV1_MASK;
pub const CRU_APLL_FBDIV_POS: u32 = 0x00;
pub const CRU_APLL_FBDIV_MASK: u32 = 0xfff << CRU_APLL_POSTDIV1_POS;
pub const CRU_APLL_FBDIV: u32 = CRU_APLL_FBDIV_MASK;
// the bit feild for CRU_APLL_CON1
// TODO
// the bit feild for CRU_CLKSEL_CON28
pub const CRU_CLKSEL_WR_EN_POS: u32 = 16;
pub const CRU_CLKSEL_WR_EN_MASK: u32 = 0x01 << CRU_CLKSEL_WR_EN_POS;
pub const CRU_CLKSEL_WR_EN: u32 = CRU_CLKSEL_WR_EN_MASK;
pub const CRU_CLKSEL_CCLK_EMMC_POS: u32 = 12;
pub const CRU_CLKSEL_CCLK_EMMC_MASK: u32 = 0x07 << CRU_CLKSEL_CCLK_EMMC_POS;
pub const CRU_CLKSEL_CCLK_EMMC: u32 = CRU_CLKSEL_CCLK_EMMC_MASK;
pub const CRU_CLKSEL_CCLK_EMMC_XIN_SOC0_MUX: u32 = 0x000 << CRU_CLKSEL_CCLK_EMMC_POS;
pub const CRU_CLKSEL_CCLK_EMMC_GPL_DIV_200M: u32 = 0x001 << CRU_CLKSEL_CCLK_EMMC_POS;
pub const CRU_CLKSEL_CCLK_EMMC_GPL_DIV_150M: u32 = 0x002 << CRU_CLKSEL_CCLK_EMMC_POS;
pub const CRU_CLKSEL_CCLK_EMMC_CPL_DIV_100M: u32 = 0x003 << CRU_CLKSEL_CCLK_EMMC_POS;
pub const CRU_CLKSEL_CCLK_EMMC_CPL_DIV_50M: u32 = 0x004 << CRU_CLKSEL_CCLK_EMMC_POS;
pub const CRU_CLKSEL_CCLK_EMMC_SOC0_375K: u32 = 0x005 << CRU_CLKSEL_CCLK_EMMC_POS;
pub const CRU_CLKSEL_BCLK_EMMC_POS: u32 = 8;
pub const CRU_CLKSEL_BCLK_EMMC_MASK: u32 = 0x03 << CRU_CLKSEL_BCLK_EMMC_POS;
pub const CRU_CLKSEL_BCLK_EMMC: u32 = CRU_CLKSEL_BCLK_EMMC_MASK;
pub const CRU_CLKSEL_BCLK_EMMC_GPL_DIV_200M: u32 = 0x000 << CRU_CLKSEL_BCLK_EMMC_POS;
pub const CRU_CLKSEL_BCLK_EMMC_GPL_DIV_150M: u32 = 0x001 << CRU_CLKSEL_BCLK_EMMC_POS;
pub const CRU_CLKSEL_BCLK_EMMC_CPL_DIV_125M: u32 = 0x002 << CRU_CLKSEL_BCLK_EMMC_POS;
pub const CRU_CLKSEL_SCLK_SFC_POS: u32 = 4;
pub const CRU_CLKSEL_SCLK_SFC_MASK: u32 = 0x07 << CRU_CLKSEL_SCLK_SFC_POS;
pub const CRU_CLKSEL_SCLK_SFC: u32 = CRU_CLKSEL_SCLK_SFC_MASK;
pub const CRU_CLKSEL_SCLK_SFC_XIN_SOC0_MUX: u32 = 0x000 << CRU_CLKSEL_SCLK_SFC_POS;
pub const CRU_CLKSEL_SCLK_SFC_CPL_DIV_50M: u32 = 0x001 << CRU_CLKSEL_SCLK_SFC_POS;
pub const CRU_CLKSEL_SCLK_SFC_GPL_DIV_75M: u32 = 0x002 << CRU_CLKSEL_SCLK_SFC_POS;
pub const CRU_CLKSEL_SCLK_SFC_GPL_DIV_100M: u32 = 0x003 << CRU_CLKSEL_SCLK_SFC_POS;
pub const CRU_CLKSEL_SCLK_SFC_CPL_DIV_125M: u32 = 0x004 << CRU_CLKSEL_SCLK_SFC_POS;
pub const CRU_CLKSEL_SCLK_SFC_GPL_DIV_150M: u32 = 0x005 << CRU_CLKSEL_SCLK_SFC_POS;
pub const CRU_CLKSEL_NCLK_NANDC_POS: u32 = 0;
pub const CRU_CLKSEL_NCLK_NANDC_MASK: u32 = 0x03 << CRU_CLKSEL_NCLK_NANDC_POS;
pub const CRU_CLKSEL_NCLK_NANDC: u32 = CRU_CLKSEL_NCLK_NANDC_MASK;
pub const CRU_CLKSEL_NCLK_NANDC_GPL_DIV_200M: u32 = 0x000 << CRU_CLKSEL_NCLK_NANDC_POS;
pub const CRU_CLKSEL_NCLK_NANDC_GPL_DIV_150M: u32 = 0x001 << CRU_CLKSEL_NCLK_NANDC_POS;
pub const CRU_CLKSEL_NCLK_NANDC_CPL_DIV_125M: u32 = 0x002 << CRU_CLKSEL_NCLK_NANDC_POS;
pub const CRU_CLKSEL_NCLK_NANDC_XIN_SOC0_MUX: u32 = 0x003 << CRU_CLKSEL_NCLK_NANDC_POS;
