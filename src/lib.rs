//! Clock driver for RK3568
//!
//! # Overview
//!
//! Clock is the heart of synchronous digital systems. All the events in an SoC are 
//! controlled by the active edge of the clock and clock frequency is 
//! often synonymous with throughput and performance.
//! 
//! ## Clock tree
//! The clock tree is a hierarchical structure that distributes the clock signal
//! from a single source to various components in the system. The clock tree is
//! designed to minimize skew and ensure that all components receive the clock signal
//! at the same time. The clock tree is typically implemented using a combination of
//! buffers, inverters, and multiplexers. The clock tree is also responsible for
//! generating different clock frequencies for different components in the system.
//! 
//! ## CRU
//! The Clock Reset Unit (CRU) is responsible for managing the clock and reset signals
//! for the various components in the RK3568 SoC. The CRU is responsible for generating
//! the clock signals for the CPU, GPU, DSP, and other peripherals. The CRU is also
//! responsible for managing the reset signals for the various components in the RK3568 SoC.
//!
//! # About the driver
//! 
//! The driver is designed to be used in a no_std environment, and does not depend on any
//! external libraries or crates. The driver is designed to be used in a no_std environment,
//! and does not depend on any external libraries or crates. The driver is designed to be
//! used in a no_std environment, and does not depend on any external libraries or crates.
//! 
//! ## Usage
//! 
//! ```rust
//! use rk3568_clk::CRU;
//! use rk3568_clk::cru_clksel_con28_bits::{*};
//! 
//! let clock = CRU::new(clk_addr as u64);
//! clock.cru_clksel_set_cclk_emmc(CRU_CLKSEL_CCLK_EMMC_GPL_DIV_200M);
//! clock.cru_enable_tclk_emmc();
//! ```

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

/// By re-exporting, the cru-level Module is shielded.
mod cru;
pub use cru::*;
