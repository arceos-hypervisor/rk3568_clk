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
//! # RK3568
//! The RK3568 is a high-performance SoC designed for a wide range of applications,
//! including AI, machine learning, and multimedia processing. The RK3568 features
//! a powerful CPU, GPU, and DSP, as well as a variety of peripherals and interfaces.
//! The RK3568 is designed to be highly configurable and scalable, making it suitable
//! for a wide range of applications.
//! 
//! ## CRU
//! The Clock Reset Unit (CRU) is responsible for managing the clock and reset signals
//! for the various components in the RK3568 SoC. The CRU is responsible for generating
//! the clock signals for the CPU, GPU, DSP, and other peripherals. The CRU is also
//! responsible for managing the reset signals for the various components in the RK3568 SoC.
//!

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

pub mod cru;
