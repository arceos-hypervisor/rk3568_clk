//!  Basic interface for reading and writing CRU registers
//!
//! # Clock and Reset Unit(CRU)
//!
//! The CRU is an APB slave module that is designed for generating all of the internal and 
//! system clocks, resets in the chip. CRU generates system clocks from PLL output clock or 
//! external clock source, and generates system reset from external power-on-reset, watchdog 
//! timer reset or software reset or temperature sensor.The CRU islocated at several addresses. 
//! - PMUCRU, used for always on system, with base address 0xFDD00000 
//! - PMUSCRU, used for always on secure system, with base address 0xFDD30000 
//! - CRU, used for general system except always on system, with base address 0xFDD20000 
//! - SCRU, used for general secure system except always on system, with base address 0xFDD10000
//! 
//! **NOTE:** Currently, only CUR is supported.
//! 
//! ## Block Diagram
//! 
//! The CRU comprises with: 
//! - PLL 
//! - Register configuration unit 
//! - Clock generate unit 
//! - Reset generate unit 
//! 
//! ## Function Description
//! 
//! There are 6 fractional PLLs in RK3568: APLL, PPLL, HPLL, DPLL, CPLL and GPLL. There are 
//! also 3 integer PLLs: MPLL, NPLL and VPLL. Each PLL can only receive 24MHz oscillator as 
//! input reference clock and can be set to three work modes: normal mode, slow mode and 
//! deep slow mode.
//! 
//! To maximize the flexibility, some of clocks can select divider source from multiple PLLs. To 
//! provide some specific frequency, another solution is integrated: fractional divider. Divfree50
//! divider and divfreeNP5 divider are also provided for some modules.All clocks can be gated 
//! by software.
//! 
//! # About the driver
//! 
//! The driver is designed to be used in a no_std environment, and does not depend on any
//! external libraries or crates. The driver is designed to be used in a no_std environment,
//! and does not depend on any external libraries or crates. The driver is designed to be
//! used in a no_std environment, and does not depend on any external libraries or crates.
//! 
//! ## Register map
//! 
//! There are a total of 240 registers in the CRU, each 4 bytes wide. The register map is 
//! defined in the `RegMap` struct. The register map is used to access
//! the CRU registers in a safe and efficient manner. The register map is defined as a
//! `#[repr(C)]` struct, which means that the layout of the struct is the same as the
//! layout of the registers in memory. 
//! 
//! ## Register access
//! 
//! The driver provides a safe interface to the CRU registers. It allows reading and
//! writing to the registers, as well as setting and getting specific bits in the
//! registers. 
//! 
//! The driver uses the `read_volatile()` and `write_volatile()` functions to
//! read and write to the registers. These functions are used to ensure that the
//! compiler does not optimize away the read and write operations. 
//! 
//! The driver also provides functions to set and clear specific bits in the registers. 
//! These functions are used to ensure that the compiler does not optimize away 
//! the read and write operations.
//! 
//! For one register, the follwing modules are defined:
//! - `pub mod cru_*_bits`：define the bit field definitions for the CRU registers.
//! - `impl CRU {}` : implemented read, write, and other operation interfaces corresponding to each register.
//! 
//! **It is particularly important to note that** some registers can not be accessed by the driver 
//! due to soc work mode. Otherwise, Reading and writing certain registers requires special processing procedures. 
//! For example,When power on or changing PLL setting, we must program PLL into slow mode or deep slow mode.
//! 
//! This module provides a basic interface for reading and writing CRU registers. DO NOT include
//! special processing procedures, instead, the user should implement the special processing procedures.

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicPtr, Ordering};

/// Clock and Reset Unit (CRU) register map
/// this struct defines the layout of the CRU registers in memory.
/// It is used to access the CRU registers in a safe and efficient manner.
/// There are a total of 240 registers in the CRU, each 4 bytes wide.
#[repr(C)]
struct RegMap {
    cru_apll_con: [u32; 5],             // APLL 寄存器 /* 0x0000 ~ 0x0014 */
    reserved0: [u32; 3],                // 保留
    cru_dpll_con: [u32; 5],             // GPLL 寄存器 /* 0x0020 ~ 0x0034 */
    reserved1: [u32; 3],                // 保留
    cru_gpll_con: [u32; 5],             // CPLL 寄存器 /* 0x0040 ~ 0x0054 */
    reserved2: [u32; 3],                // 保留
    cru_cpll_con: [u32; 5],             // DPLL 寄存器 /* 0x0060 ~ 0x0074 */
    reserved3: [u32; 3],                // 保留
    cru_npll_con: [u32; 2],             // NPLL 寄存器 /* 0x0080 ~ 0x0088 */
    reserved4: [u32; 6],                // 保留
    cru_vpll_con: [u32; 2],             // VPLL 寄存器 /* 0x00A0 ~ 0x00A8 */
    reserved5: [u32; 6],                // 保留
    
    cru_mode_con00: u32,                // 模式控制寄存器   /* 0x00C0 */
    cru_misc_con: [u32; 3],             // 杂项控制寄存器 
    cru_glb_cnt_th: u32,                // 全局计数阈值     /*  */
    cru_glb_srst_fst: u32,              // 全局软复位
    cru_glb_srsr_snd: u32,              // 全局软复位
    cru_glb_rst_con: u32,               // 全局软复位阈值
    cru_glb_rst_st: u32,                // 全局软复位状态
    
    reserved6: [u32; 7],                // 保留
    clksel_con: [u32; 85],              // 时钟选择寄存器
    reserved7: [u32; 43],               // 保留
    clk_gate_con: [u32; 36],            // 时钟门控寄存器
    reserved8: [u32; 28],               // 保留
    
    cru_softrst_con: [u32; 30],         // 软复位寄存器
    reserved9: [u32; 2],                // 保留
    cru_ssgtbl: [u32; 32],              // SSG表寄存器

    cru_autocs_core_con: [u32; 2],
    cru_autocs_gpu_con: [u32; 2],
    cru_autocs_bus_con: [u32; 2],
    cru_autocs_top_con: [u32; 2],
    cru_autocs_rkvdec_con: [u32; 2],
    cru_autocs_rkvenc_con: [u32; 2],
    cru_autocs_vpu_con: [u32; 2],
    cru_autocs_peri_con: [u32; 2],
    cru_autocs_gpll_con: [u32; 2],
    cru_autocs_cpll_con: [u32; 2],

    reserved10: [u32; 12],              // 保留
    sdmmc0_con: [u32; 2],               // SDMMC0 控制寄存器
    sdmmc1_con: [u32; 2],               // SDMMC1 控制寄存器
    sdmmc2_con: [u32; 2],               // SDMMC2 控制寄存器
    emmc_con: [u32; 2],                 // eMMC 控制寄存器
}

/// Clock and Reset Unit (CRU)
/// This struct provides a safe interface to the CRU registers.
/// It allows reading and writing to the registers, as well as setting and getting
/// specific bits in the registers.
pub struct CRU {
    reg: AtomicPtr<RegMap>,
}

/// Implementing the most basic interface
/// - Creates a new instance of the CRU struct.
/// - Based function to read and write registers.
impl CRU {
    /// Creates a new instance with the given base address.
    ///
    /// # Arguments
    /// * `base_addr` - The base memory address of the CRU (Control Register Unit) registers.
    ///                Must be a valid, aligned address for hardware access.
    ///
    /// # Safety
    /// This is unsafe because:
    /// - The caller must ensure `base_addr` points to valid CRU hardware registers
    /// - The address must be properly aligned for `RegMap` type access
    /// - Concurrent access to the same hardware registers may cause UB
    ///
    /// # Example
    /// ```no_run
    /// // Safe wrapper would verify address validity
    /// let cru = unsafe { new(0xFF00_0000) };
    /// ```
    pub fn new(base_addr: u64) -> Self {
        Self {
            reg: AtomicPtr::new(base_addr as *mut RegMap),
        }
    }

    fn get_reg_ptr(&self) -> *mut RegMap {
        self.reg.load(Ordering::Acquire)
    }

    fn with_reg<F, R>(&self, f: F) -> R
    where
        F: FnOnce(*mut RegMap) -> R,
    {
        let ptr = self.get_reg_ptr();
        f(ptr)
    }

    /// Reads a 32-bit register at the given address.
    ///
    /// # Arguments
    /// * None
    /// 
    /// # Returns
    /// - The value read from the register.
    fn read_reg(&self, addr: u64) -> u32 {
        unsafe { read_volatile(addr as *const u32) as u32 }
    }

    /// Writes a 32-bit value to the register at the given address.
    ///
    /// # Arguments
    /// * `value` - The value to write to the register.
    /// * `addr` - The address of the register to write to.
    /// 
    /// # Returns
    /// - None
    fn write_reg(&self, addr: u64, value: u32) {
        unsafe { write_volatile(addr as *mut u32, value); }
    }
}

/// This module contains the bit field definitions for the `CRU_CLKSEL_CON28` register.
/// It defines the positions and masks for various clock selection bits.
pub mod cru_clksel_con28_bits {
    pub const CRU_CLKSEL_CON28_WR_EN_POS: u32 = 16;
    pub const CRU_CLKSEL_CON28_WR_EN_MASK: u32 = 0x01 << CRU_CLKSEL_CON28_WR_EN_POS;
    pub const CRU_CLKSEL_CON28_WR_EN: u32 = CRU_CLKSEL_CON28_WR_EN_MASK;
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
}

use cru_clksel_con28_bits::*;

/// Implemented read, write, and other operation interfaces corresponding to each bit in the `CRU_CLKSEL_CON28` register.
/// - The definition of the bit is in the `cru_clksel_con28_bits` module.
/// - The `CRU_CLKSEL_CON28` register is used to select the clock source for various peripherals.
impl CRU {
    /// Set cclk_emmc clock
    ///
    /// # Arguments
    /// One of the following values
    /// - CRU_CLKSEL_CCLK_EMMC_XIN_SOC0_MUX
    /// - CRU_CLKSEL_CCLK_EMMC_GPL_DIV_200M
    /// - CRU_CLKSEL_CCLK_EMMC_GPL_DIV_150M
    /// - CRU_CLKSEL_CCLK_EMMC_CPL_DIV_100M
    /// - CRU_CLKSEL_CCLK_EMMC_CPL_DIV_50M
    /// - CRU_CLKSEL_CCLK_EMMC_SOC0_375K
    ///
    /// # Returns
    /// - None
    pub fn cru_clksel_set_cclk_emmc(&self, sel: u32) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clksel_con[28] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(
                addr, 
                (current & !CRU_CLKSEL_CCLK_EMMC_MASK) | 
                (CRU_CLKSEL_CCLK_EMMC_MASK << CRU_CLKSEL_CON28_WR_EN_POS) | 
                sel
            );
        });
    }

    /// Get cclk_emmc clock
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// One of the following values
    /// - CRU_CLKSEL_CCLK_EMMC_XIN_SOC0_MUX
    /// - CRU_CLKSEL_CCLK_EMMC_GPL_DIV_200M
    /// - CRU_CLKSEL_CCLK_EMMC_GPL_DIV_150M
    /// - CRU_CLKSEL_CCLK_EMMC_CPL_DIV_100M
    /// - CRU_CLKSEL_CCLK_EMMC_CPL_DIV_50M
    /// - CRU_CLKSEL_CCLK_EMMC_SOC0_375K
    pub fn cru_clksel_get_cclk_emmc(&self) -> u32 {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clksel_con[28] as *const u32 as u64;
            self.read_reg(addr) & CRU_CLKSEL_CCLK_EMMC
        })
    }

    /// Set bclk_emmc clock
    ///
    /// # Arguments
    /// One of the following values
    /// - CRU_CLKSEL_BCLK_EMMC_GPL_DIV_200M
    /// - CRU_CLKSEL_BCLK_EMMC_GPL_DIV_150M
    /// - CRU_CLKSEL_BCLK_EMMC_CPL_DIV_125M
    ///
    /// # Returns
    /// - None
    pub fn cru_clksel_set_bclk_emmc(&self, sel: u32) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clksel_con[28] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current & !CRU_CLKSEL_BCLK_EMMC_MASK) | (CRU_CLKSEL_BCLK_EMMC_MASK << CRU_CLKSEL_CON28_WR_EN_POS ) | sel);
        });
    }

    /// Get bclk_emmc clock
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// One of the following values
    /// - CRU_CLKSEL_BCLK_EMMC_GPL_DIV_200M
    /// - CRU_CLKSEL_BCLK_EMMC_GPL_DIV_150M
    /// - CRU_CLKSEL_BCLK_EMMC_CPL_DIV_125M
    pub fn cru_clksel_get_bclk_emmc(&self) -> u32 {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clksel_con[28] as *const u32 as u64;
            self.read_reg(addr) & CRU_CLKSEL_BCLK_EMMC
        })
    }

    /// Set sclk_sfc clock
    ///
    /// # Arguments
    /// One of the following values
    /// - CRU_CLKSEL_SCLK_SFC_XIN_SOC0_MUX
    /// - CRU_CLKSEL_SCLK_SFC_CPL_DIV_50M
    /// - CRU_CLKSEL_SCLK_SFC_GPL_DIV_75M
    /// - CRU_CLKSEL_SCLK_SFC_GPL_DIV_100M
    /// - CRU_CLKSEL_SCLK_SFC_CPL_DIV_125M
    /// - CRU_CLKSEL_SCLK_SFC_GPL_DIV_150M
    ///
    /// # Returns
    /// - None
    pub fn cru_clksel_set_sclk_sfc(&self, sel: u32) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clksel_con[28] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current & !CRU_CLKSEL_SCLK_SFC_MASK) | (CRU_CLKSEL_SCLK_SFC_MASK << CRU_CLKSEL_CON28_WR_EN_POS ) | sel);
        });
    }

    /// Get sclk_sfc clock
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// One of the following values
    /// - CRU_CLKSEL_SCLK_SFC_XIN_SOC0_MUX
    /// - CRU_CLKSEL_SCLK_SFC_CPL_DIV_50M
    /// - CRU_CLKSEL_SCLK_SFC_GPL_DIV_75M
    /// - CRU_CLKSEL_SCLK_SFC_GPL_DIV_100M
    /// - CRU_CLKSEL_SCLK_SFC_CPL_DIV_125M
    /// - CRU_CLKSEL_SCLK_SFC_GPL_DIV_150M
    pub fn cru_clksel_get_sclk_sfc(&self) -> u32 {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clksel_con[28] as *const u32 as u64;
            self.read_reg(addr) & CRU_CLKSEL_SCLK_SFC
        })
    }

    /// set nclk_nandc clock
    ///
    /// # Arguments
    /// One of the following values
    /// - CRU_CLKSEL_NCLK_NANDC_GPL_DIV_200M
    /// - CRU_CLKSEL_NCLK_NANDC_GPL_DIV_150M
    /// - CRU_CLKSEL_NCLK_NANDC_CPL_DIV_125M
    /// - CRU_CLKSEL_NCLK_NANDC_XIN_SOC0_MUX
    ///
    /// # Returns
    /// - None
    pub fn cru_clksel_set_nclk_nanc(&self, sel: u32) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clksel_con[28] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current & !CRU_CLKSEL_NCLK_NANDC_MASK) | (CRU_CLKSEL_NCLK_NANDC_MASK << CRU_CLKSEL_CON28_WR_EN_POS ) | sel);
        })
    }

    /// Get nclk_nandc clock
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// One of the following values
    /// - CRU_CLKSEL_NCLK_NANDC_GPL_DIV_200M
    /// - CRU_CLKSEL_NCLK_NANDC_GPL_DIV_150M
    /// - CRU_CLKSEL_NCLK_NANDC_CPL_DIV_125M
    /// - CRU_CLKSEL_NCLK_NANDC_XIN_SOC0_MUX
    pub fn cru_clksel_get_nclk_nanc(&self) -> u32 {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clksel_con[28] as *const u32 as u64;
            self.read_reg(addr) & CRU_CLKSEL_NCLK_NANDC
        })
    }
}

/// This module contains the bit field definitions for the `CRU_GATE_CON09` register.
/// It defines the positions and masks for various clock selection bits.
pub mod cru_gate_con09_bits {
    pub const CRU_GATE_CON09_WR_EN_POS: u32 = 16;
    pub const CRU_GATE_CON09_WR_EN_MASK: u32 = 0x01 << CRU_GATE_CON09_WR_EN_POS;
    pub const CRU_GATE_CON09_WR_EN: u32 = CRU_GATE_CON09_WR_EN_MASK;
    pub const CRU_GATE_CLK_TRNG_NS_POS: u32 = 11;
    pub const CRU_GATE_CLK_TRNG_NS_MASK: u32 = 0x01 << CRU_GATE_CLK_TRNG_NS_POS;
    pub const CRU_GATE_CLK_TRNG_NS: u32 = CRU_GATE_CLK_TRNG_NS_MASK;
    pub const CRU_GATE_HCLK_TRNG_NS_POS: u32 = 10;
    pub const CRU_GATE_HCLK_TRNG_NS_MASK: u32 = 0x01 << CRU_GATE_HCLK_TRNG_NS_POS;
    pub const CRU_GATE_HCLK_TRNG_NS: u32 = CRU_GATE_HCLK_TRNG_NS_MASK;
    pub const CRU_GATE_TCLK_EMMC_POS: u32 = 9;
    pub const CRU_GATE_TCLK_EMMC_MASK: u32 = 0x01 << CRU_GATE_TCLK_EMMC_POS;
    pub const CRU_GATE_TCLK_EMMC: u32 = CRU_GATE_TCLK_EMMC_MASK;
    pub const CRU_GATE_CCLK_EMMC_POS: u32 = 8;
    pub const CRU_GATE_CCLK_EMMC_MASK: u32 = 0x01 << CRU_GATE_CCLK_EMMC_POS;
    pub const CRU_GATE_CCLK_EMMC: u32 = CRU_GATE_CCLK_EMMC_MASK;
    pub const CRU_GATE_BCLK_EMMC_POS: u32 = 7;
    pub const CRU_GATE_BCLK_EMMC_MASK: u32 = 0x01 << CRU_GATE_BCLK_EMMC_POS;
    pub const CRU_GATE_BCLK_EMMC: u32 = CRU_GATE_BCLK_EMMC_MASK;
    pub const CRU_GATE_HCLK_EMMC_POS: u32 = 6;
    pub const CRU_GATE_HCLK_EMMC_MASK: u32 = 0x01 << CRU_GATE_HCLK_EMMC_POS;
    pub const CRU_GATE_HCLK_EMMC: u32 = CRU_GATE_HCLK_EMMC_MASK;
    pub const CRU_GATE_ACLK_EMMC_POS: u32 = 5;
    pub const CRU_GATE_ACLK_EMMC_MASK: u32 = 0x01 << CRU_GATE_ACLK_EMMC_POS;
    pub const CRU_GATE_ACLK_EMMC: u32 = CRU_GATE_ACLK_EMMC_MASK;
    pub const CRU_GATE_SCLK_SFC_POS: u32 = 4;
    pub const CRU_GATE_SCLK_SFC_MASK: u32 = 0x01 << CRU_GATE_SCLK_SFC_POS;
    pub const CRU_GATE_SCLK_SFC: u32 = CRU_GATE_SCLK_SFC_MASK;
    pub const CRU_GATE_HCLK_SFC_XIP_POS: u32 = 3;
    pub const CRU_GATE_HCLK_SFC_XIP_MASK: u32 = 0x01 << CRU_GATE_HCLK_SFC_XIP_POS;
    pub const CRU_GATE_HCLK_SFC_XIP: u32 = CRU_GATE_HCLK_SFC_XIP_MASK;
    pub const CRU_GATE_HCLK_SFC_POS: u32 = 2;
    pub const CRU_GATE_HCLK_SFC_MASK: u32 = 0x01 << CRU_GATE_HCLK_SFC_POS;
    pub const CRU_GATE_HCLK_SFC: u32 = CRU_GATE_HCLK_SFC_MASK;
    pub const CRU_GATE_NCLK_NANDC_POS: u32 = 1;
    pub const CRU_GATE_NCLK_NANDC_MASK: u32 = 0x01 << CRU_GATE_NCLK_NANDC_POS;
    pub const CRU_GATE_NCLK_NANDC: u32 = CRU_GATE_NCLK_NANDC_MASK;
    pub const CRU_GATE_HCLK_NANDC_POS: u32 = 0;
    pub const CRU_GATE_HCLK_NANDC_MASK: u32 = 0x01 << CRU_GATE_HCLK_NANDC_POS;
    pub const CRU_GATE_HCLK_NANDC: u32 = CRU_GATE_HCLK_NANDC_MASK;
}

use cru_gate_con09_bits::*;

/// Implemented read, write, and other operation interfaces corresponding to each bit in the `CRU_GATE_CON09` register.
/// - The definition of the bit is in the `cru_gate_con09_bits` module.
/// - The `CRU_GATE_CON09` register is used to control the clock gating for various peripherals.
impl CRU {
    /// Enable clk_trng_ns
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_clk_trng_ns(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_GATE_CLK_TRNG_NS_MASK << CRU_GATE_CON09_WR_EN_POS )) & !CRU_GATE_CLK_TRNG_NS);
        })
    }

    /// Disable clk_trng_ns
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_clk_trng_ns(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_GATE_CLK_TRNG_NS_MASK << CRU_GATE_CON09_WR_EN_POS ) | CRU_GATE_CLK_TRNG_NS);
        })
    }

    /// Check the clk_trng_ns is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if clk_trng_ns is enabled
    /// - false if clk_trng_ns is disabled
    pub fn cru_clk_trng_ns_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            self.read_reg(addr) & CRU_GATE_CLK_TRNG_NS == 0
        })
    }

    /// Enable hclk_trng_ns
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_hclk_trng_ns(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_GATE_HCLK_TRNG_NS_MASK << CRU_GATE_CON09_WR_EN_POS )) & !CRU_GATE_HCLK_TRNG_NS);
        })
    }

    /// Disable hclk_trng_ns
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_hclk_trng_ns(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_GATE_HCLK_TRNG_NS_MASK << CRU_GATE_CON09_WR_EN_POS ) | CRU_GATE_HCLK_TRNG_NS);
        })
    }

    /// Check the hclk_trng_ns is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if hclk_trng_ns is enabled
    /// - false if hclk_trng_ns is disabled
    pub fn cru_hclk_trng_ns_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            self.read_reg(addr) & CRU_GATE_HCLK_TRNG_NS == 0
        })
    }

    /// Enable tclk_emmc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_tclk_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_GATE_TCLK_EMMC_MASK << CRU_GATE_CON09_WR_EN_POS )) & !CRU_GATE_TCLK_EMMC);
        })
    }

    /// Disable tclk_emmc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_tclk_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_GATE_TCLK_EMMC_MASK << CRU_GATE_CON09_WR_EN_POS ) | CRU_GATE_TCLK_EMMC);
        })
    }

    /// Check the tclk_emmc is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if tclk_emmc is enabled
    /// - false if tclk_emmc is disabled
    pub fn cru_tclk_emmc_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            self.read_reg(addr) & CRU_GATE_TCLK_EMMC == 0
        })
    }

    /// Enable cclk_emmc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_cclk_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_GATE_CCLK_EMMC_MASK << CRU_GATE_CON09_WR_EN_POS )) & !CRU_GATE_CCLK_EMMC);
        })
    }

    /// Disable cclk_emmc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_cclk_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_GATE_CCLK_EMMC_MASK << CRU_GATE_CON09_WR_EN_POS ) | CRU_GATE_CCLK_EMMC);
        })
    }

    /// Check the cclk_emmc is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if cclk_emmc is enabled
    /// - false if cclk_emmc is disabled
    pub fn cru_cclk_emmc_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            self.read_reg(addr) & CRU_GATE_CCLK_EMMC == 0
        })
    }

    /// Enable bclk_emmc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_bclk_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_GATE_BCLK_EMMC_MASK << CRU_GATE_CON09_WR_EN_POS )) & !CRU_GATE_BCLK_EMMC);
        })
    }

    /// Disable bclk_emmc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_bclk_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_GATE_BCLK_EMMC_MASK << CRU_GATE_CON09_WR_EN_POS ) | CRU_GATE_BCLK_EMMC);
        })
    }

    /// Check the bclk_emmc is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if bclk_emmc is enabled
    /// - false if bclk_emmc is disabled
    pub fn cru_bclk_emmc_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            self.read_reg(addr) & CRU_GATE_BCLK_EMMC == 0
        })
    }

    /// Enable hclk_emmc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_hclk_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_GATE_HCLK_EMMC_MASK << CRU_GATE_CON09_WR_EN_POS )) & !CRU_GATE_HCLK_EMMC);
        })
    }

    /// Disable hclk_emmc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_hclk_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_GATE_HCLK_EMMC_MASK << CRU_GATE_CON09_WR_EN_POS ) | CRU_GATE_HCLK_EMMC);
        })
    }

    /// Check the hclk_emmc is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if hclk_emmc is enabled
    /// - false if hclk_emmc is disabled
    pub fn cru_hclk_emmc_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            self.read_reg(addr) & CRU_GATE_HCLK_EMMC == 0
        })
    }

    /// Enable aclk_emmc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_aclk_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_GATE_ACLK_EMMC_MASK << CRU_GATE_CON09_WR_EN_POS )) & !CRU_GATE_ACLK_EMMC);
        })
    }

    /// Disable aclk_emmc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_aclk_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_GATE_ACLK_EMMC_MASK << CRU_GATE_CON09_WR_EN_POS ) | CRU_GATE_ACLK_EMMC);
        })
    }

    /// Check the aclk_emmc is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if aclk_emmc is enabled
    /// - false if aclk_emmc is disabled
    pub fn cru_aclk_emmc_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            self.read_reg(addr) & CRU_GATE_ACLK_EMMC == 0
        })
    }

    /// Enable sclk_sfc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_sclk_sfc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_GATE_SCLK_SFC_MASK << CRU_GATE_CON09_WR_EN_POS )) & !CRU_GATE_SCLK_SFC);
        })
    }

    /// Disable sclk_sfc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_sclk_sfc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_GATE_SCLK_SFC_MASK << CRU_GATE_CON09_WR_EN_POS ) | CRU_GATE_SCLK_SFC);
        })
    }

    /// Check the sclk_sfc is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if sclk_sfc is enabled
    /// - false if sclk_sfc is disabled
    pub fn cru_sclk_sfc_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            self.read_reg(addr) & CRU_GATE_SCLK_SFC == 0
        })
    }

    /// Enable hclk_sfc_xip
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_hclk_sfc_xip(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_GATE_HCLK_SFC_XIP_MASK << CRU_GATE_CON09_WR_EN_POS )) & !CRU_GATE_HCLK_SFC_XIP);
        })
    }

    /// Disable hclk_sfc_xip
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_hclk_sfc_xip(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_GATE_HCLK_SFC_XIP_MASK << CRU_GATE_CON09_WR_EN_POS ) | CRU_GATE_HCLK_SFC_XIP);
        })
    }

    /// Check the hclk_sfc_xip is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if hclk_sfc_xip is enabled
    /// - false if hclk_sfc_xip is disabled
    pub fn cru_hclk_sfc_xip_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            self.read_reg(addr) & CRU_GATE_HCLK_SFC_XIP == 0
        })
    }

    /// Enable hclk_sfc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_hclk_sfc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_GATE_HCLK_SFC_MASK << CRU_GATE_CON09_WR_EN_POS )) & !CRU_GATE_HCLK_SFC);
        })
    }

    /// Disable hclk_sfc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_hclk_sfc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_GATE_HCLK_SFC_MASK << CRU_GATE_CON09_WR_EN_POS ) | CRU_GATE_HCLK_SFC);
        })
    }

    /// Check the hclk_sfc is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if hclk_sfc is enabled
    /// - false if hclk_sfc is disabled
    pub fn cru_hclk_sfc_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            self.read_reg(addr) & CRU_GATE_HCLK_SFC == 0
        })
    }

    /// Enable nclk_nandc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_nclk_nandc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_GATE_NCLK_NANDC_MASK << CRU_GATE_CON09_WR_EN_POS )) & !CRU_GATE_NCLK_NANDC);
        })
    }

    /// Disable nclk_nandc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_nclk_nandc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_GATE_NCLK_NANDC_MASK << CRU_GATE_CON09_WR_EN_POS ) | CRU_GATE_NCLK_NANDC);
        })
    }

    /// Check the nclk_nandc is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if nclk_nandc is enabled
    /// - false if nclk_nandc is disabled
    pub fn cru_nclk_nandc_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            self.read_reg(addr) & CRU_GATE_NCLK_NANDC == 0
        })
    }

    /// Enable hclk_nandc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_hclk_nandc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_GATE_HCLK_NANDC_MASK << CRU_GATE_CON09_WR_EN_POS )) & !CRU_GATE_HCLK_NANDC);
        })
    }

    /// Disable hclk_nandc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_hclk_nandc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_GATE_HCLK_NANDC_MASK << CRU_GATE_CON09_WR_EN_POS ) | CRU_GATE_HCLK_NANDC);
        })
    }

    /// Check the hclk_nandc is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if hclk_nandc is enabled
    /// - false if hclk_nandc is disabled
    pub fn cru_hclk_nandc_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).clk_gate_con[9] as *const u32 as u64;
            self.read_reg(addr) & CRU_GATE_HCLK_NANDC == 0
        })
    }
}

/// This module contains the bit field definitions for the `CRU_SOFTRST_CON07` register.
/// It defines the positions and masks for various clock selection bits.
pub mod cru_softrst_con07_bits {
    pub const CRU_SOFTRST_CON07_WR_EN_POS: u32 = 16;
    pub const CRU_SOFTRST_CON07_WR_EN_MASK: u32 = 0x01 << CRU_SOFTRST_CON07_WR_EN_POS;
    pub const CRU_SOFTRST_CON07_WR_EN: u32 = CRU_SOFTRST_CON07_WR_EN_MASK;
    pub const CRU_SOFTRST_TRESET_EMMC_POS: u32 = 9;
    pub const CRU_SOFTRST_TRESET_EMMC_MASK: u32 = 0x01 << CRU_SOFTRST_TRESET_EMMC_POS;
    pub const CRU_SOFTRST_TRESET_EMMC: u32 = CRU_SOFTRST_TRESET_EMMC_MASK;
    pub const CRU_SOFTRST_CRESET_EMMC_POS: u32 = 8;
    pub const CRU_SOFTRST_CRESET_EMMC_MASK: u32 = 0x01 << CRU_SOFTRST_CRESET_EMMC_POS;
    pub const CRU_SOFTRST_CRESET_EMMC: u32 = CRU_SOFTRST_CRESET_EMMC_MASK;
    pub const CRU_SOFTRST_BRESET_EMMC_POS: u32 = 7;
    pub const CRU_SOFTRST_BRESET_EMMC_MASK: u32 = 0x01 << CRU_SOFTRST_BRESET_EMMC_POS;
    pub const CRU_SOFTRST_BRESET_EMMC: u32 = CRU_SOFTRST_BRESET_EMMC_MASK;
    pub const CRU_SOFTRST_HRESET_EMMC_POS: u32 = 6;
    pub const CRU_SOFTRST_HRESET_EMMC_MASK: u32 = 0x01 << CRU_SOFTRST_HRESET_EMMC_POS;
    pub const CRU_SOFTRST_HRESET_EMMC: u32 = CRU_SOFTRST_HRESET_EMMC_MASK;
    pub const CRU_SOFTRST_ARESET_EMMC_POS: u32 = 5;
    pub const CRU_SOFTRST_ARESET_EMMC_MASK: u32 = 0x01 << CRU_SOFTRST_ARESET_EMMC_POS;
    pub const CRU_SOFTRST_ARESET_EMMC: u32 = CRU_SOFTRST_ARESET_EMMC_MASK;
    pub const CRU_SOFTRST_SRESET_SFC_POS: u32 = 4;
    pub const CRU_SOFTRST_SRESET_SFC_MASK: u32 = 0x01 << CRU_SOFTRST_SRESET_SFC_POS;
    pub const CRU_SOFTRST_SRESET_SFC: u32 = CRU_SOFTRST_SRESET_SFC_MASK;
    pub const CRU_SOFTRST_HRESET_SFC_XIP_POS: u32 = 3;
    pub const CRU_SOFTRST_HRESET_SFC_XIP_MASK: u32 = 0x01 << CRU_SOFTRST_HRESET_SFC_XIP_POS;
    pub const CRU_SOFTRST_HRESET_SFC_XIP: u32 = CRU_SOFTRST_HRESET_SFC_XIP_MASK;
    pub const CRU_SOFTRST_HRESET_SFC_POS: u32 = 2;
    pub const CRU_SOFTRST_HRESET_SFC_MASK: u32 = 0x01 << CRU_SOFTRST_HRESET_SFC_POS;
    pub const CRU_SOFTRST_HRESET_SFC: u32 = CRU_SOFTRST_HRESET_SFC_MASK;
    pub const CRU_SOFTRST_NRESET_NANDC_POS: u32 = 1;
    pub const CRU_SOFTRST_NRESET_NANDC_MASK: u32 = 0x01 << CRU_SOFTRST_NRESET_NANDC_POS;
    pub const CRU_SOFTRST_NRESET_NANDC: u32 = CRU_SOFTRST_NRESET_NANDC_MASK;
    pub const CRU_SOFTRST_HRESET_NANDC_POS: u32 = 0;
    pub const CRU_SOFTRST_HRESET_NANDC_MASK: u32 = 0x01 << CRU_SOFTRST_HRESET_NANDC_POS;
    pub const CRU_SOFTRST_HRESET_NANDC: u32 = CRU_SOFTRST_HRESET_NANDC_MASK;
}

use cru_softrst_con07_bits::*;

/// Implemented read, write, and other operation interfaces corresponding to each bit in the `CRU_SOFTRST_CON07` register.
/// - The definition of the bit is in the `cru_softrst_con07_bits` module.
/// - The `CRU_SOFTRST_CON07` register is used to control the soft reset for various peripherals.
impl CRU {
    /// Enable treset_emmc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_treset_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_SOFTRST_TRESET_EMMC_MASK << CRU_SOFTRST_CON07_WR_EN_POS )) & !CRU_SOFTRST_TRESET_EMMC);
        })
    }

    /// Disable treset_emmc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_treset_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_SOFTRST_TRESET_EMMC_MASK << CRU_SOFTRST_CON07_WR_EN_POS ) | CRU_SOFTRST_TRESET_EMMC);
        })
    }

    /// Check the treset_emmc is finished or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if treset_emmc is finished
    /// - false if treset_emmc is not finished
    pub fn cru_treset_emmc_is_finished(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            self.read_reg(addr) & CRU_SOFTRST_TRESET_EMMC == 0
        })
    }

    /// Enable creset_emmc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_creset_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_SOFTRST_CRESET_EMMC_MASK << CRU_SOFTRST_CON07_WR_EN_POS )) & !CRU_SOFTRST_CRESET_EMMC);
        })
    }

    /// Disable creset_emmc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_creset_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_SOFTRST_CRESET_EMMC_MASK << CRU_SOFTRST_CON07_WR_EN_POS ) | CRU_SOFTRST_CRESET_EMMC);
        })
    }

    /// Check the creset_emmc is finished or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if creset_emmc is finished
    /// - false if creset_emmc is not finished
    pub fn cru_creset_emmc_is_finished(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            self.read_reg(addr) & CRU_SOFTRST_CRESET_EMMC == 0
        })
    }

    /// Enable breset_emmc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_breset_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_SOFTRST_BRESET_EMMC_MASK << CRU_SOFTRST_CON07_WR_EN_POS )) & !CRU_SOFTRST_BRESET_EMMC);
        })
    }

    /// Disable breset_emmc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_breset_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_SOFTRST_BRESET_EMMC_MASK << CRU_SOFTRST_CON07_WR_EN_POS ) | CRU_SOFTRST_BRESET_EMMC);
        })
    }

    /// Check the breset_emmc is finished or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if breset_emmc is finished
    /// - false if breset_emmc is not finished
    pub fn cru_breset_emmc_is_finished(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            self.read_reg(addr) & CRU_SOFTRST_BRESET_EMMC == 0
        })
    }

    /// Enable hreset_emmc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_hreset_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_SOFTRST_HRESET_EMMC_MASK << CRU_SOFTRST_CON07_WR_EN_POS )) & !CRU_SOFTRST_HRESET_EMMC);
        })
    }

    /// Disable hreset_emmc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_hreset_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_SOFTRST_HRESET_EMMC_MASK << CRU_SOFTRST_CON07_WR_EN_POS ) | CRU_SOFTRST_HRESET_EMMC);
        })
    }

    /// Check the hreset_emmc is finished or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if hreset_emmc is finished
    /// - false if hreset_emmc is not finished
    pub fn cru_hreset_emmc_is_finished(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            self.read_reg(addr) & CRU_SOFTRST_HRESET_EMMC == 0
        })
    }

    /// Enable areset_emmc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_areset_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_SOFTRST_ARESET_EMMC_MASK << CRU_SOFTRST_CON07_WR_EN_POS )) & !CRU_SOFTRST_ARESET_EMMC);
        })
    }

    /// Disable areset_emmc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_areset_emmc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_SOFTRST_ARESET_EMMC_MASK << CRU_SOFTRST_CON07_WR_EN_POS ) | CRU_SOFTRST_ARESET_EMMC);
        })
    }

    /// Check the areset_emmc is finished or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if areset_emmc is finished
    /// - false if areset_emmc is not finished
    pub fn cru_areset_emmc_is_finished(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            self.read_reg(addr) & CRU_SOFTRST_ARESET_EMMC == 0
        })
    }

    /// Enable sreset_sfc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_sreset_sfc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_SOFTRST_SRESET_SFC_MASK << CRU_SOFTRST_CON07_WR_EN_POS )) & !CRU_SOFTRST_SRESET_SFC);
        })
    }

    /// Disable sreset_sfc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_sreset_sfc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_SOFTRST_SRESET_SFC_MASK << CRU_SOFTRST_CON07_WR_EN_POS ) | CRU_SOFTRST_SRESET_SFC);
        })
    }

    /// Check the sreset_sfc is finished or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if sreset_sfc is finished
    /// - false if sreset_sfc is not finished
    pub fn cru_sreset_sfc_is_finished(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            self.read_reg(addr) & CRU_SOFTRST_SRESET_SFC == 0
        })
    }

    /// Enable hreset_sfc_xip
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_hreset_sfc_xip(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_SOFTRST_HRESET_SFC_XIP_MASK << CRU_SOFTRST_CON07_WR_EN_POS )) & !CRU_SOFTRST_HRESET_SFC_XIP);
        })
    }

    /// Disable hreset_sfc_xip
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_hreset_sfc_xip(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_SOFTRST_HRESET_SFC_XIP_MASK << CRU_SOFTRST_CON07_WR_EN_POS ) | CRU_SOFTRST_HRESET_SFC_XIP);
        })
    }

    /// Check the hreset_sfc_xip is finished or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if hreset_sfc_xip is finished
    /// - false if hreset_sfc_xip is not finished
    pub fn cru_hreset_sfc_xip_is_finished(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            self.read_reg(addr) & CRU_SOFTRST_HRESET_SFC_XIP == 0
        })
    }

    /// Enable hreset_sfc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_hreset_sfc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_SOFTRST_HRESET_SFC_MASK << CRU_SOFTRST_CON07_WR_EN_POS )) & !CRU_SOFTRST_HRESET_SFC);
        })
    }

    /// Disable hreset_sfc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_hreset_sfc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_SOFTRST_HRESET_SFC_MASK << CRU_SOFTRST_CON07_WR_EN_POS ) | CRU_SOFTRST_HRESET_SFC);
        })
    }

    /// Check the hreset_sfc is finished or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if hreset_sfc is finished
    /// - false if hreset_sfc is not finished
    pub fn cru_hreset_sfc_is_finished(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            self.read_reg(addr) & CRU_SOFTRST_HRESET_SFC == 0
        })
    }

    /// Enable nreset_nandc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_nreset_nandc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_SOFTRST_NRESET_NANDC_MASK << CRU_SOFTRST_CON07_WR_EN_POS )) & !CRU_SOFTRST_NRESET_NANDC);
        })
    }

    /// Disable nreset_nandc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_nreset_nandc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_SOFTRST_NRESET_NANDC_MASK << CRU_SOFTRST_CON07_WR_EN_POS ) | CRU_SOFTRST_NRESET_NANDC);
        })
    }

    /// Check the nreset_nandc is finished or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if nreset_nandc is finished
    /// - false if nreset_nandc is not finished
    pub fn cru_nreset_nandc_is_finished(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            self.read_reg(addr) & CRU_SOFTRST_NRESET_NANDC == 0
        })
    }

    /// Enable hreset_nandc
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_hreset_nandc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_SOFTRST_HRESET_NANDC_MASK << CRU_SOFTRST_CON07_WR_EN_POS )) & !CRU_SOFTRST_HRESET_NANDC);
        })
    }

    /// Disable hreset_nandc
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_hreset_nandc(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_SOFTRST_HRESET_NANDC_MASK << CRU_SOFTRST_CON07_WR_EN_POS ) | CRU_SOFTRST_HRESET_NANDC);
        })
    }

    /// Check the hreset_nandc is finished or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if hreset_nandc is finished
    /// - false if hreset_nandc is not finished
    pub fn cru_hreset_nandc_is_finished(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).cru_softrst_con[7] as *const u32 as u64;
            self.read_reg(addr) & CRU_SOFTRST_HRESET_NANDC == 0
        })
    }
}

/// This module contains the bit field definitions for the `CRU_EMMC_CON0` register.
/// It defines the positions and masks for various clock selection bits.
pub mod cru_emmc_con0_bits {
    pub const CRU_EMMC_CON0_WR_EN_POS: u32 = 16;
    pub const CRU_EMMC_CON0_WR_EN_MASK: u32 = 0x01 << CRU_EMMC_CON0_WR_EN_POS;
    pub const CRU_EMMC_CON0_WR_EN: u32 = CRU_EMMC_CON0_WR_EN_MASK;
    pub const CRU_EMMC_DRV_POS: u32 = 11;
    pub const CRU_EMMC_DRV_MASK: u32 = 0x01 << CRU_EMMC_DRV_POS;
    pub const CRU_EMMC_DRV: u32 = CRU_EMMC_DRV_MASK;
    pub const CRU_EMMC_DRV_DELAYNUM_POS: u32 = 3;
    pub const CRU_EMMC_DRV_DELAYNUM_MASK: u32 = 0xff << CRU_EMMC_DRV_DELAYNUM_POS;
    pub const CRU_EMMC_DRV_DELAYNUM: u32 = CRU_EMMC_DRV_DELAYNUM_MASK;
    pub const CRU_EMMC_DRV_DEGREE_POS: u32 = 1;
    pub const CRU_EMMC_DRV_DEGREE_MASK: u32 = 0x03 << CRU_EMMC_DRV_DEGREE_POS;
    pub const CRU_EMMC_DRV_DEGREE: u32 = CRU_EMMC_DRV_DEGREE_MASK;
    pub const CRU_EMMC_INIT_STATE_POS: u32 = 0;
    pub const CRU_EMMC_INIT_STATE_MASK: u32 = 0x01 << CRU_EMMC_INIT_STATE_POS;
    pub const CRU_EMMC_INIT_STATE: u32 = CRU_EMMC_INIT_STATE_MASK;
}

use cru_emmc_con0_bits::*;

/// Implemented read, write, and other operation interfaces corresponding to each bit in the `CRU_EMMC_CON0` register.
/// - The definition of the bit is in the `cru_emmc_con0_bits` module.
/// - The `CRU_EMMC_CON0` register is used to control some functions for EMMC.
impl CRU {
    /// Enable emmc_drv
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_emmc_drv(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[0] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_EMMC_DRV_MASK << CRU_EMMC_CON0_WR_EN_POS ) | CRU_EMMC_DRV);
        })
    }

    /// Disable emmc_drv
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_emmc_drv(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[0] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_EMMC_DRV_MASK << CRU_EMMC_CON0_WR_EN_POS )) & !CRU_EMMC_DRV);
        })
    }

    /// Check the emmc_drv is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if emmc_drv is enabled
    /// - false if emmc_drv is disabled
    pub fn cru_emmc_drv_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[0] as *const u32 as u64;
            (self.read_reg(addr) & CRU_EMMC_DRV) == CRU_EMMC_DRV
        })
    }

    /// Set drv_delaynum
    ///
    /// # Arguments
    /// - value: the value to set for drv_delaynum
    ///
    /// # Returns
    /// - None
    pub fn cru_set_emmc_drv_delaynum(&self, value: u32) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[0] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current & !CRU_EMMC_DRV_DELAYNUM_MASK) | (CRU_EMMC_DRV_DELAYNUM_MASK << CRU_EMMC_CON0_WR_EN_POS ) | (value << CRU_EMMC_DRV_DELAYNUM_POS));
        })
    }

    /// Get drv_delaynum
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - the value of drv_delaynum
    pub fn cru_get_emmc_drv_delaynum(&self) -> u32 {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[0] as *const u32 as u64;
            (self.read_reg(addr) & CRU_EMMC_DRV_DELAYNUM_MASK) >> CRU_EMMC_DRV_DELAYNUM_POS
        })
    }

    /// Set drv_degree
    ///
    /// # Arguments
    /// - value: the value to set for drv_degree
    ///
    /// # Returns
    /// - None
    pub fn cru_set_emmc_drv_degree(&self, value: u32) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[0] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current & !CRU_EMMC_DRV_DEGREE_MASK) | (CRU_EMMC_DRV_DEGREE_MASK << CRU_EMMC_CON0_WR_EN_POS ) | (value << CRU_EMMC_DRV_DEGREE_POS));
        })
    }

    /// Get drv_degree
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - the value of drv_degree
    pub fn cru_get_emmc_drv_degree(&self) -> u32 {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[0] as *const u32 as u64;
            (self.read_reg(addr) & CRU_EMMC_DRV_DEGREE_MASK) >> CRU_EMMC_DRV_DEGREE_POS
        })
    }

    /// Enable emmc_init_state
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_emmc_init_state(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[0] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_EMMC_INIT_STATE_MASK << CRU_EMMC_CON0_WR_EN_POS ) | CRU_EMMC_INIT_STATE);
        })
    }

    /// Disable emmc_init_state
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_emmc_init_state(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[0] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_EMMC_INIT_STATE_MASK << CRU_EMMC_CON0_WR_EN_POS )) & !CRU_EMMC_INIT_STATE);
        })
    }

    /// Check the emmc_init_state is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if emmc_init_state is enabled
    /// - false if emmc_init_state is disabled
    pub fn cru_emmc_init_state_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[0] as *const u32 as u64;
            (self.read_reg(addr) & CRU_EMMC_INIT_STATE) == CRU_EMMC_INIT_STATE
        })
    }


}

/// This module contains the bit field definitions for the `CRU_EMMC_CON1` register.
/// It defines the positions and masks for various clock selection bits.
pub mod cru_emmc_con1_bits {
    pub const CRU_EMMC_CON1_WR_EN_POS: u32 = 16;
    pub const CRU_EMMC_CON1_WR_EN_MASK: u32 = 0x01 << CRU_EMMC_CON1_WR_EN_POS;
    pub const CRU_EMMC_CON1_WR_EN: u32 = CRU_EMMC_CON1_WR_EN_MASK;
    pub const CRU_EMMC_SAMPLE_POS: u32 = 10;
    pub const CRU_EMMC_SAMPLE_MASK: u32 = 0x01 << CRU_EMMC_SAMPLE_POS;
    pub const CRU_EMMC_SAMPLE: u32 = CRU_EMMC_SAMPLE_MASK;
    pub const CRU_EMMC_SAMPLE_DELAYNUM_POS: u32 = 2;
    pub const CRU_EMMC_SAMPLE_DELAYNUM_MASK: u32 = 0xff << CRU_EMMC_SAMPLE_DELAYNUM_POS;
    pub const CRU_EMMC_SAMPLE_DELAYNUM: u32 = CRU_EMMC_SAMPLE_DELAYNUM_MASK;
    pub const CRU_EMMC_SAMPLE_DEGREE_POS: u32 = 0;
    pub const CRU_EMMC_SAMPLE_DEGREE_MASK: u32 = 0x03 << CRU_EMMC_SAMPLE_DEGREE_POS;
    pub const CRU_EMMC_SAMPLE_DEGREE: u32 = CRU_EMMC_SAMPLE_DEGREE_MASK;
}

use cru_emmc_con1_bits::*;

/// Implemented read, write, and other operation interfaces corresponding to each bit in the `CRU_EMMC_CON1` register.
/// - The definition of the bit is in the `cru_emmc_con1_bits` module.
/// - The `CRU_EMMC_CON1` register is used to control some functions for EMMC.
impl CRU {
    /// Enable emmc_sample
    ///
    /// # Arguments
    ///
    /// # Returns
    /// - None
    pub fn cru_enable_emmc_sample(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[1] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, current | (CRU_EMMC_SAMPLE << CRU_EMMC_CON1_WR_EN_POS) | CRU_EMMC_SAMPLE);
        })
    }

    /// Disable emmc_sample
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - None
    pub fn cru_disable_emmc_sample(&self) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[1] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current | (CRU_EMMC_SAMPLE << CRU_EMMC_CON1_WR_EN_POS)) & !CRU_EMMC_SAMPLE);
        })
    }

    /// Check the emmc_sample is enabled or not
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - true if emmc_sample is enabled
    /// - false if emmc_sample is disabled
    pub fn cru_emmc_sample_is_enabled(&self) -> bool {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[1] as *const u32 as u64;
            (self.read_reg(addr) & CRU_EMMC_SAMPLE) == CRU_EMMC_SAMPLE
        })
    }

    /// Set sample_delaynum
    ///
    /// # Arguments
    /// - value: the value to set for sample_delaynum
    ///
    /// # Returns
    /// - None
    pub fn cru_set_emmc_sample_delaynum(&self, value: u32) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[1] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current & !CRU_EMMC_SAMPLE_DELAYNUM_MASK) | (CRU_EMMC_SAMPLE_DELAYNUM_MASK << CRU_EMMC_CON1_WR_EN_POS ) | (value << CRU_EMMC_SAMPLE_DELAYNUM_POS));
        })
    }

    /// Get sample_delaynum
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - the value of sample_delaynum
    pub fn cru_get_emmc_sample_delaynum(&self) -> u32 {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[1] as *const u32 as u64;
            (self.read_reg(addr) & CRU_EMMC_SAMPLE_DELAYNUM_MASK) >> CRU_EMMC_SAMPLE_DELAYNUM_POS
        })
    }

    /// Set sample_degree
    ///
    /// # Arguments
    /// - value: the value to set for sample_degree
    ///
    /// # Returns
    /// - None
    pub fn cru_set_emmc_sample_degree(&self, value: u32) {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[1] as *const u32 as u64;
            let current = self.read_reg(addr);
            self.write_reg(addr, (current & !CRU_EMMC_SAMPLE_DEGREE_MASK) | (CRU_EMMC_SAMPLE_DEGREE_MASK << CRU_EMMC_CON1_WR_EN_POS ) | (value << CRU_EMMC_SAMPLE_DEGREE_POS));
        })
    }

    /// Get sample_degree
    ///
    /// # Arguments
    /// - None
    ///
    /// # Returns
    /// - the value of sample_degree
    pub fn cru_get_emmc_sample_degree(&self) -> u32 {
        self.with_reg(|ptr| unsafe {
            let addr = &(*ptr).emmc_con[1] as *const u32 as u64;
            (self.read_reg(addr) & CRU_EMMC_SAMPLE_DEGREE_MASK) >> CRU_EMMC_SAMPLE_DEGREE_POS
        })
    }
}
