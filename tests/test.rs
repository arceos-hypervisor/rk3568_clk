#![no_std]
#![no_main]
#![feature(used_with_arg)]

extern crate alloc;

#[bare_test::tests]
mod tests {
    use bare_test::{
        globals::{PlatformInfoKind, global_val},
        mem::iomap,
        println,
    };
    use log::info;
    use rk3568_clk::CRU;
    use rk3568_clk::cru_clksel_con28_bits::*;
    // use rk3568_clk::cru::cru_gate_con09_bits::{*};

    #[test]
    fn test_platform() {
        let PlatformInfoKind::DeviceTree(fdt) = &global_val().platform_info;
        let fdt_parser = fdt.get();

        // Detect platform type by searching for compatible strings
        if fdt_parser
            .find_compatible(&["rockchip,rk3568-dwcmshc"])
            .next()
            .is_some()
        {
            // Rockchip platform detected, run uboot test
            info!("Rockchip platform detected, running uboot test");
            test_uboot(&fdt_parser);
        } else if fdt_parser
            .find_compatible(&["pci-host-ecam-generic"])
            .next()
            .is_some()
        {
            // QEMU platform detected, run qemu test
            info!("QEMU platform detected, running qemu test");
            // test_qemu(&fdt_parser);
        } else {
            // Unknown platform, output debug information
            println!("Unknown platform, no compatible devices found");
        }
    }

    fn test_uboot(fdt: &fdt_parser::Fdt) {
        let emmc = fdt
            .find_compatible(&["rockchip,dwcmshc-sdhci"])
            .next()
            .unwrap();
        let clock = fdt
            .find_compatible(&["rockchip,rk3568-cru"])
            .next()
            .unwrap();
        let syscon = fdt
            .find_compatible(&["rockchip,rk3568-grf"])
            .next()
            .unwrap();

        info!(
            "EMMC: {} Clock: {}, Syscon {}",
            emmc.name, clock.name, syscon.name
        );

        let emmc_reg = emmc.reg().unwrap().next().unwrap();
        let clk_reg = clock.reg().unwrap().next().unwrap();
        let syscon_reg = syscon.reg().unwrap().next().unwrap();

        println!(
            "EMMC reg {:#x}, {:#x}",
            emmc_reg.address,
            emmc_reg.size.unwrap()
        );
        println!(
            "Clock reg {:#x}, {:#x}",
            clk_reg.address,
            clk_reg.size.unwrap()
        );
        println!(
            "Syscon reg {:#x}, {:#x}",
            syscon_reg.address,
            syscon_reg.size.unwrap()
        );

        let emmc_addr_ptr = iomap((emmc_reg.address as usize).into(), emmc_reg.size.unwrap());
        let clk_add_ptr = iomap((clk_reg.address as usize).into(), clk_reg.size.unwrap());
        let syscon_addr_ptr = iomap(
            (syscon_reg.address as usize).into(),
            syscon_reg.size.unwrap(),
        );

        let emmc_addr = emmc_addr_ptr.as_ptr() as usize;
        let clk_addr = clk_add_ptr.as_ptr() as usize;
        let syscon_addr = syscon_addr_ptr.as_ptr() as usize;
        info!(
            "EMMC addr: {:#x}, Clock addr: {:#x}, Syscon addr: {:#x}",
            emmc_addr, clk_addr, syscon_addr
        );

        let clock = CRU::new(clk_addr as *mut _);
        clock.cru_clksel_set_cclk_emmc(CRU_CLKSEL_CCLK_EMMC_GPL_DIV_200M);
        info!(
            "clock.cru_clksel_get_cclk_emmc(): {:#x}",
            clock.cru_clksel_get_cclk_emmc()
        );
        clock.cru_clksel_set_cclk_emmc(CRU_CLKSEL_CCLK_EMMC_GPL_DIV_150M);
        info!(
            "clock.cru_clksel_get_cclk_emmc(): {:#x}",
            clock.cru_clksel_get_cclk_emmc()
        );
        clock.cru_clksel_set_cclk_emmc(CRU_CLKSEL_CCLK_EMMC_CPL_DIV_100M);
        info!(
            "clock.cru_clksel_get_cclk_emmc(): {:#x}",
            clock.cru_clksel_get_cclk_emmc()
        );

        clock.cru_enable_tclk_emmc();
        info!(
            "clock.cru_enable_tclk_emmc(): { }",
            clock.cru_tclk_emmc_is_enabled()
        );
        clock.cru_disable_tclk_emmc();
        info!(
            "clock.cru_disable_tclk_emmc(): { }",
            clock.cru_tclk_emmc_is_enabled()
        );

        clock.cru_enable_treset_emmc();
        info!(
            "clock.cru_enable_treset_emmc(): { }",
            clock.cru_treset_emmc_is_finished()
        );
        clock.cru_disable_treset_emmc();
        info!(
            "clock.cru_disable_treset_emmc(): { }",
            clock.cru_treset_emmc_is_finished()
        );

        clock.cru_enable_emmc_drv();
        info!(
            "clock.cru_enable_emmc_drv(): { }",
            clock.cru_emmc_drv_is_enabled()
        );
        clock.cru_disable_emmc_drv();
        info!(
            "clock.cru_disable_emmc_drv(): { }",
            clock.cru_emmc_drv_is_enabled()
        );
        clock.cru_set_emmc_drv_delaynum(0x0f);
        info!(
            "clock.cru_set_emmc_drv_delaynum(): {:#x}",
            clock.cru_get_emmc_drv_delaynum()
        );
        clock.cru_set_emmc_drv_delaynum(0x00);
        info!(
            "clock.cru_set_emmc_drv_delaynum(): {:#x}",
            clock.cru_get_emmc_drv_delaynum()
        );

        clock.cru_enable_emmc_sample();
        info!(
            "clock.cru_enable_emmc_sample(): { }",
            clock.cru_emmc_sample_is_enabled()
        );
        clock.cru_disable_emmc_sample();
        info!(
            "clock.cru_disable_emmc_sample(): { }",
            clock.cru_emmc_sample_is_enabled()
        );
    }
}
