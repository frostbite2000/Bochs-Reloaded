/////////////////////////////////////////////////////////////////////////
// $Id$
/////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2023-2025  The Bochs Project
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA

// Intel i6300ESB southbridge emulation

#define BX_PLUGGABLE

#include "iodev.h"

#if BX_SUPPORT_PCI

#include "pci.h"
#include "i6300esb.h"

#define LOG_THIS thei6300esb_lpc->

bx_i6300esb_lpc_c *thei6300esb_lpc = NULL;
bx_i6300esb_wdog_c *thei6300esb_wdog = NULL;

// Device plugin entries

PLUGIN_ENTRY_FOR_MODULE(i6300esb_lpc)
{
  if (mode == PLUGIN_INIT) {
    thei6300esb_lpc = new bx_i6300esb_lpc_c();
    BX_REGISTER_DEVICE_DEVMODEL(plugin, type, thei6300esb_lpc, BX_PLUGIN_I6300ESB_LPC);
    return 0;
  } else if (mode == PLUGIN_FINI) {
    delete thei6300esb_lpc;
    thei6300esb_lpc = NULL;
    return 0;
  } else if (mode == PLUGIN_PROBE) {
    return PLUGTYPE_STANDARD;
  } else {
    return 0;
  }
}

PLUGIN_ENTRY_FOR_MODULE(i6300esb_wdog)
{
  if (mode == PLUGIN_INIT) {
    thei6300esb_wdog = new bx_i6300esb_wdog_c();
    BX_REGISTER_DEVICE_DEVMODEL(plugin, type, thei6300esb_wdog, BX_PLUGIN_I6300ESB_WDOG);
    return 0;
  } else if (mode == PLUGIN_FINI) {
    delete thei6300esb_wdog;
    thei6300esb_wdog = NULL;
    return 0;
  } else if (mode == PLUGIN_PROBE) {
    return PLUGTYPE_OPTIONAL;
  } else {
    return 0;
  }
}

//
// i6300ESB LPC/ISA Bridge
//

bx_i6300esb_lpc_c::bx_i6300esb_lpc_c()
{
  put("i6300esb_lpc", "I6300ESB_LPC");
}

bx_i6300esb_lpc_c::~bx_i6300esb_lpc_c()
{
  BX_DEBUG(("Exit"));
}

void bx_i6300esb_lpc_c::init(void)
{
  // Register 82801 LPC bridge (i6300ESB southbridge is based on 82801) (device 1:31.0)
  init_pci_conf(0x8086, 0x25a1, 0x01, 0x060100, 0x80);
  
  // Register I/O handlers for port access
  DEV_register_iowrite_handler(this, write_handler, 0x00B2, "i6300ESB LPC bridge", 1);
  DEV_register_iowrite_handler(this, write_handler, 0x00B3, "i6300ESB LPC bridge", 1);
  DEV_register_iowrite_handler(this, write_handler, 0x04D0, "i6300ESB LPC bridge", 1);
  DEV_register_iowrite_handler(this, write_handler, 0x04D1, "i6300ESB LPC bridge", 1);
  DEV_register_iowrite_handler(this, write_handler, 0x0CF9, "i6300ESB LPC bridge", 1);

  DEV_register_ioread_handler(this, read_handler, 0x00B2, "i6300ESB LPC bridge", 1);
  DEV_register_ioread_handler(this, read_handler, 0x00B3, "i6300ESB LPC bridge", 1);
  DEV_register_ioread_handler(this, read_handler, 0x04D0, "i6300ESB LPC bridge", 1);
  DEV_register_ioread_handler(this, read_handler, 0x04D1, "i6300ESB LPC bridge", 1);
  DEV_register_ioread_handler(this, read_handler, 0x0CF9, "i6300ESB LPC bridge", 1);

  BX_INFO(("i6300ESB LPC bridge initialized"));
}

void bx_i6300esb_lpc_c::reset(unsigned type)
{
  pci_conf[0x04] = 0x07; // Command: I/O, MEM, BUS Master
  pci_conf[0x05] = 0x00;
  pci_conf[0x06] = 0x00;
  pci_conf[0x07] = 0x02; // Status
  pci_conf[0x4c] = 0x4d;
  pci_conf[0x4e] = 0x03;
  pci_conf[0x4f] = 0x00;
  pci_conf[0x60] = 0x80; // PIRQ routing registers
  pci_conf[0x61] = 0x80;
  pci_conf[0x62] = 0x80;
  pci_conf[0x63] = 0x80;
  pci_conf[0x69] = 0x02;
  pci_conf[0x72] = 0x02; // SMRAM control register
  pci_conf[0x80] = 0x00;
  pci_conf[0x82] = 0x00;

  BX_I6300ESB_THIS rtc_conf = 0x00;
  BX_I6300ESB_THIS pmbase = 0;
  BX_I6300ESB_THIS acpi_cntl = 0;
  BX_I6300ESB_THIS gpio_base = 0;
  BX_I6300ESB_THIS gpio_cntl = 0x00;
  BX_I6300ESB_THIS back_cntl = 0x0f;
  BX_I6300ESB_THIS lpc_if_com_range = 0x00;
  BX_I6300ESB_THIS lpc_if_fdd_lpt_range = 0x00;
  BX_I6300ESB_THIS lpc_if_sound_range = 0x00;
  BX_I6300ESB_THIS fwh_dec_en1 = 0xff;
  BX_I6300ESB_THIS fwh_dec_en2 = 0x0f;
  BX_I6300ESB_THIS gen1_dec = 0x0000;
  BX_I6300ESB_THIS lpc_en = 0x0000;
  BX_I6300ESB_THIS fwh_sel1 = 0x00112233;
  
  // Reset PCI reset register
  BX_I6300ESB_THIS pci_reset = 0;
  
  // Reset ELCR registers
  BX_I6300ESB_THIS elcr1 = 0;
  BX_I6300ESB_THIS elcr2 = 0;
  
  // Reset APM registers
  BX_I6300ESB_THIS apm_cnt = 0;
  BX_I6300ESB_THIS apm_sts = 0;
}

void bx_i6300esb_lpc_c::register_state(void)
{
  bx_list_c *list = new bx_list_c(SIM->get_bochs_root(), "i6300esb_lpc", "i6300ESB LPC Bridge State");
  
  register_pci_state(list);
  
  BXRS_HEX_PARAM_FIELD(list, pmbase, BX_I6300ESB_THIS pmbase);
  BXRS_HEX_PARAM_FIELD(list, acpi_cntl, BX_I6300ESB_THIS acpi_cntl);
  BXRS_HEX_PARAM_FIELD(list, gpio_base, BX_I6300ESB_THIS gpio_base);
  BXRS_HEX_PARAM_FIELD(list, gpio_cntl, BX_I6300ESB_THIS gpio_cntl);
  BXRS_HEX_PARAM_FIELD(list, back_cntl, BX_I6300ESB_THIS back_cntl);
  BXRS_HEX_PARAM_FIELD(list, rtc_conf, BX_I6300ESB_THIS rtc_conf);
  BXRS_HEX_PARAM_FIELD(list, lpc_if_com_range, BX_I6300ESB_THIS lpc_if_com_range);
  BXRS_HEX_PARAM_FIELD(list, lpc_if_fdd_lpt_range, BX_I6300ESB_THIS lpc_if_fdd_lpt_range);
  BXRS_HEX_PARAM_FIELD(list, lpc_if_sound_range, BX_I6300ESB_THIS lpc_if_sound_range);
  BXRS_HEX_PARAM_FIELD(list, fwh_dec_en1, BX_I6300ESB_THIS fwh_dec_en1);
  BXRS_HEX_PARAM_FIELD(list, fwh_dec_en2, BX_I6300ESB_THIS fwh_dec_en2);
  BXRS_HEX_PARAM_FIELD(list, gen1_dec, BX_I6300ESB_THIS gen1_dec);
  BXRS_HEX_PARAM_FIELD(list, lpc_en, BX_I6300ESB_THIS lpc_en);
  BXRS_HEX_PARAM_FIELD(list, fwh_sel1, BX_I6300ESB_THIS fwh_sel1);
  BXRS_HEX_PARAM_FIELD(list, elcr1, BX_I6300ESB_THIS elcr1);
  BXRS_HEX_PARAM_FIELD(list, elcr2, BX_I6300ESB_THIS elcr2);
  BXRS_HEX_PARAM_FIELD(list, pci_reset, BX_I6300ESB_THIS pci_reset);
  BXRS_HEX_PARAM_FIELD(list, apm_cnt, BX_I6300ESB_THIS apm_cnt);
  BXRS_HEX_PARAM_FIELD(list, apm_sts, BX_I6300ESB_THIS apm_sts);
  
  // PIRQ routing registration
  bx_list_c *pirq = new bx_list_c(list, "pirq_routing");
  for (unsigned i=0; i<8; i++) {
    char name[8];
    sprintf(name, "pirq%d", i+1);
    new bx_shadow_num_c(pirq, name, &BX_I6300ESB_THIS pirq_rout[i], BASE_HEX);
  }
}

void bx_i6300esb_lpc_c::after_restore_state(void)
{
  if (BX_I6300ESB_THIS acpi_cntl & 0x10)
    acpi_update_io_mapping(BX_I6300ESB_THIS pmbase);
    
  if (BX_I6300ESB_THIS gpio_cntl & 0x10)
    gpio_update_io_mapping(BX_I6300ESB_THIS gpio_base);
    
  map_bios_regions();
}

// static IO port read callback handler
// redirects to non-static class handler to avoid virtual functions

Bit32u bx_i6300esb_lpc_c::read_handler(void *this_ptr, Bit32u address, unsigned io_len)
{
  bx_i6300esb_lpc_c *class_ptr = (bx_i6300esb_lpc_c *) this_ptr;
  return class_ptr->read(address, io_len);
}

Bit32u bx_i6300esb_lpc_c::read(Bit32u address, unsigned io_len)
{
  switch (address) {
    case 0x00b2: // APM command
      return BX_I6300ESB_THIS apm_cnt;

    case 0x00b3: // APM status
      return BX_I6300ESB_THIS apm_sts;

    case 0x04d0: // ELCR1
      return BX_I6300ESB_THIS elcr1;

    case 0x04d1: // ELCR2
      return BX_I6300ESB_THIS elcr2;

    case 0x0cf9: // Reset control
      return BX_I6300ESB_THIS pci_reset;
  }

  return 0xffffffff;
}

// static IO port write callback handler
// redirects to non-static class handler to avoid virtual functions

void bx_i6300esb_lpc_c::write_handler(void *this_ptr, Bit32u address, Bit32u value, unsigned io_len)
{
  bx_i6300esb_lpc_c *class_ptr = (bx_i6300esb_lpc_c *) this_ptr;
  class_ptr->write(address, value, io_len);
}

void bx_i6300esb_lpc_c::write(Bit32u address, Bit32u value, unsigned io_len)
{
  switch (address) {
    case 0x00b2: // APM command
      BX_I6300ESB_THIS apm_cnt = value & 0xff;
      break;
      
    case 0x00b3: // APM status
      BX_I6300ESB_THIS apm_sts = value & 0xff;
      break;

    case 0x04d0: // ELCR1
      BX_I6300ESB_THIS elcr1 = value & 0xf8;
      BX_INFO(("write: ELCR1 = 0x%02x", BX_I6300ESB_THIS elcr1));
      DEV_pic_set_mode(1, BX_I6300ESB_THIS elcr1); // master PIC
      break;

    case 0x04d1: // ELCR2
      BX_I6300ESB_THIS elcr2 = value & 0xde;
      BX_INFO(("write: ELCR2 = 0x%02x", BX_I6300ESB_THIS elcr2));
      DEV_pic_set_mode(0, BX_I6300ESB_THIS elcr2); // slave PIC
      break;

    case 0x0cf9: // Reset control
      BX_INFO(("write: CPU reset register = 0x%02x", value));
      BX_I6300ESB_THIS pci_reset = value & 0x02;
      if (value & 0x04) {
        if (BX_I6300ESB_THIS pci_reset) {
          bx_pc_system.Reset(BX_RESET_HARDWARE);
        } else {
          bx_pc_system.Reset(BX_RESET_SOFTWARE);
        }
      }
      break;
  }
}

void bx_i6300esb_lpc_c::map_bios(Bit32u addr, bool readonly)
{
  // Map ROM/BIOS region at the specified address with
  // appropriate read/write permissions
  BX_DEBUG(("mapping BIOS at 0x%08x%s", addr, readonly ? " (read-only)" : ""));
  
  // Get the BIOS ROM mapped by the base firmware
  char *bios = (char*)SIM->get_param_string(BXPN_BIOS_ROM_PATH)->getptr();
  
  // Stub implementation - in a real implementation you'd actually map
  // the BIOS to the appropriate address in memory with the right permissions
  // For now, just log what we'd be doing
  BX_INFO(("i6300ESB: Would map BIOS '%s' at 0x%08x (access=%s)", bios, addr, 
          readonly ? "RO" : "RW"));
}

void bx_i6300esb_lpc_c::map_bios_regions()
{
  // Map BIOS regions according to the BIOS decode enable registers

  if(fwh_dec_en1 & 0x80) {
    map_bios(0xfff80000, true); // Read-only
    map_bios(0xffb80000, true);
    map_bios(0x000e0000, true);
  }
  if(fwh_dec_en1 & 0x40) {
    map_bios(0xfff00000, true);
    map_bios(0xffb00000, true);
  }
  // Additional regions could be mapped here based on other bits in fwh_dec_en*
}

void bx_i6300esb_lpc_c::acpi_update_io_mapping(Bit32u new_pmbase)
{
  // This function would handle ACPI I/O remapping
  BX_INFO(("ACPI base address changed to 0x%04x", new_pmbase));
}

void bx_i6300esb_lpc_c::gpio_update_io_mapping(Bit32u new_gpio_base)
{
  // This function would handle GPIO I/O remapping
  BX_INFO(("GPIO base address changed to 0x%04x", new_gpio_base));
}

// pci configuration space write callback handler
void bx_i6300esb_lpc_c::pci_write_handler(Bit8u address, Bit32u value, unsigned io_len)
{
  Bit8u value8, oldval;
  bool remap_needed = false;

  if (((address >= 0x10) && (address < 0x20)) ||
      ((address > 0x23) && (address < 0x40)))
    return;

  BX_DEBUG_PCI_WRITE(address, value, io_len);
  for (unsigned i=0; i<io_len; i++) {
    value8 = (value >> (i*8)) & 0xFF;
    oldval = pci_conf[address+i];
    switch (address+i) {
      case 0x04:
        pci_conf[address+i] = value8 & 0x05;
        break;
        
      // IRQ routing registers
      case 0x60: case 0x61:
      case 0x62: case 0x63:
        value8 &= 0x8f;
        if (value8 != oldval) {
          pci_conf[address+i] = value8;
          unsigned irq = address+i - 0x60;
          BX_I6300ESB_THIS pirq_rout[irq] = value8;
          BX_INFO(("PCI IRQ routing: PIRQ%c# set to 0x%02x", irq+'A', value8));
        }
        break;
        
      case 0x40: // PMBASE - ACPI base address
        BX_I6300ESB_THIS pmbase = (BX_I6300ESB_THIS pmbase & 0xff00) | (value8 & 0x80);
        pci_conf[address+i] = value8;
        remap_needed = true;
        break;
        
      case 0x41: // PMBASE high byte
        BX_I6300ESB_THIS pmbase = (BX_I6300ESB_THIS pmbase & 0x80) | ((value8 & 0xff) << 8);
        pci_conf[address+i] = value8;
        remap_needed = true;
        break;
        
      case 0x44: // ACPI control
        BX_I6300ESB_THIS acpi_cntl = value8;
        pci_conf[address+i] = value8;
        remap_needed = true;
        break;
        
      case 0x58: // GPIO I/O base address
        BX_I6300ESB_THIS gpio_base = (BX_I6300ESB_THIS gpio_base & 0xff00) | (value8 & 0xc0);
        pci_conf[address+i] = value8;
        remap_needed = true;
        break;
        
      case 0x59: // GPIO I/O base address high byte
        BX_I6300ESB_THIS gpio_base = (BX_I6300ESB_THIS gpio_base & 0xc0) | ((value8 & 0xff) << 8);
        pci_conf[address+i] = value8;
        remap_needed = true;
        break;
        
      case 0x5C: // GPIO control
        BX_I6300ESB_THIS gpio_cntl = value8;
        pci_conf[address+i] = value8;
        remap_needed = true;
        break;
        
      case 0x72: // SMRAM control
        pci_conf[address+i] = value8;
        // Handle SMRAM control changes - this is just a stub
        BX_INFO(("SMRAM control register modified: 0x%02x", value8));
        break;
        
      case 0xd5: // BACK_CNTL (undocumented)
        BX_I6300ESB_THIS back_cntl = value8;
        pci_conf[address+i] = value8;
        break;
        
      case 0xd8: // RTC configuration
        BX_I6300ESB_THIS rtc_conf = value8;
        pci_conf[address+i] = value8;
        break;
        
      case 0xe0: // LPC_IF_COM_RANGE
        BX_I6300ESB_THIS lpc_if_com_range = value8;
        pci_conf[address+i] = value8;
        break;
        
      case 0xe1: // LPC_IF_FDD_LPT_RANGE
        BX_I6300ESB_THIS lpc_if_fdd_lpt_range = value8;
        pci_conf[address+i] = value8;
        break;
        
      case 0xe2: // LPC_IF_SOUND_RANGE
        BX_I6300ESB_THIS lpc_if_sound_range = value8;
        pci_conf[address+i] = value8;
        break;
        
      case 0xe3: // FWH_DEC_EN1
        BX_I6300ESB_THIS fwh_dec_en1 = value8 | 0x80;
        pci_conf[address+i] = BX_I6300ESB_THIS fwh_dec_en1;
        map_bios_regions();
        break;
        
      case 0xe4: case 0xe5: // GEN1_DEC
        BX_I6300ESB_THIS gen1_dec &= ~(0xff << (8 * (address+i - 0xe4)));
        BX_I6300ESB_THIS gen1_dec |= (value8 << (8 * (address+i - 0xe4)));
        pci_conf[address+i] = value8;
        break;
        
      case 0xe6: case 0xe7: // LPC_EN
        BX_I6300ESB_THIS lpc_en &= ~(0xff << (8 * (address+i - 0xe6)));
        BX_I6300ESB_THIS lpc_en |= (value8 << (8 * (address+i - 0xe6)));
        pci_conf[address+i] = value8;
        break;
        
      case 0xe8: case 0xe9: case 0xea: case 0xeb: // FWH_SEL1
        {
          int shift = 8 * (address+i - 0xe8);
          BX_I6300ESB_THIS fwh_sel1 &= ~(0xff << shift);
          BX_I6300ESB_THIS fwh_sel1 |= (value8 << shift);
          pci_conf[address+i] = value8;
        }
        break;
        
      case 0xf0: // FWH_DEC_EN2
        BX_I6300ESB_THIS fwh_dec_en2 = value8;
        pci_conf[address+i] = value8;
        map_bios_regions();
        break;
        
      default:
        pci_conf[address+i] = value8;
        BX_DEBUG(("i6300ESB LPC write register 0x%02x value 0x%02x", address+i, value8));
    }
  }
  
  // Update I/O mappings if needed
  if (remap_needed) {
    if (BX_I6300ESB_THIS acpi_cntl & 0x10) {
      acpi_update_io_mapping(BX_I6300ESB_THIS pmbase);
    }
    
    if (BX_I6300ESB_THIS gpio_cntl & 0x10) {
      gpio_update_io_mapping(BX_I6300ESB_THIS gpio_base);
    }
  }
}

//
// i6300ESB Watchdog Timer implementation
//

bx_i6300esb_wdog_c::bx_i6300esb_wdog_c()
{
  put("i6300esb_wdog", "I6300ESB_WDOG");
  wdt_conf_reg = 0;
  wdt_count_reg = 0;
  wdt_reload_reg = 0;
  wdt_status = 0;
}

bx_i6300esb_wdog_c::~bx_i6300esb_wdog_c()
{
  BX_DEBUG(("Exit"));
}

void bx_i6300esb_wdog_c::init(void)
{
  // Register i6300ESB watchdog timer (device 1:5.0)
  init_pci_conf(0x8086, 0x25ab, 0x02, 0x088000, 0x00);
  
  // Reserve memory space for watchdog registers
  pci_conf[0x10] = 0x10; // Base address register
  pci_conf[0x14] = 0x10; // Base address register
  
  BX_INFO(("i6300ESB Watchdog Timer initialized"));
}

void bx_i6300esb_wdog_c::reset(unsigned type)
{
  pci_conf[0x04] = 0x00; // Command register
  pci_conf[0x05] = 0x00;
  pci_conf[0x06] = 0x80; // Status
  pci_conf[0x07] = 0x02;
  pci_conf[0x08] = 0x02; // Revision ID
  pci_conf[0x0D] = 0x00; // Master latency timer
  
  wdt_conf_reg = 0;
  wdt_count_reg = 0;
  wdt_reload_reg = 0;
  wdt_status = 0;
}

void bx_i6300esb_wdog_c::register_state(void)
{
  bx_list_c *list = new bx_list_c(SIM->get_bochs_root(), "i6300esb_wdog", "i6300ESB Watchdog Timer State");
  
  register_pci_state(list);
  
  BXRS_HEX_PARAM_FIELD(list, wdt_conf_reg, wdt_conf_reg);
  BXRS_HEX_PARAM_FIELD(list, wdt_count_reg, wdt_count_reg);
  BXRS_HEX_PARAM_FIELD(list, wdt_reload_reg, wdt_reload_reg);
  BXRS_HEX_PARAM_FIELD(list, wdt_status, wdt_status);
}

void bx_i6300esb_wdog_c::pci_write_handler(Bit8u address, Bit32u value, unsigned io_len)
{
  Bit8u value8;

  if ((address >= 0x10) && (address < 0x20)) 
    return;

  BX_DEBUG_PCI_WRITE(address, value, io_len);
  for (unsigned i=0; i<io_len; i++) {
    value8 = (value >> (i*8)) & 0xFF;
    switch (address+i) {
      case 0x04:  // Command register
        pci_conf[address+i] = value8 & 0x02;
        break;
      default:
        pci_conf[address+i] = value8;
        BX_DEBUG(("Watchdog register write to offset 0x%02x: 0x%02x", address+i, value8));
    }
  }
}

#endif /* BX_SUPPORT_PCI */