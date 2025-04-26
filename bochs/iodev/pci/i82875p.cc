/////////////////////////////////////////////////////////////////////////
// $Id$
/////////////////////////////////////////////////////////////////////////
//
// Intel i82875P (northbridge) emulation
// Intel i82875P (northbridge) AGP bridge
//
// Copyright (c) 2023-2025 The Bochs Project
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA

// Define BX_PLUGGABLE in files that can be compiled into plugins.  For
// platforms that require a special tag on exported symbols, BX_PLUGGABLE
// is used to know when we are exporting symbols and when we are importing.

#define BX_PLUGGABLE

#include "iodev.h"

#if BX_SUPPORT_PCI

#include "pci.h"
#include "i82875p.h"

#define LOG_THIS thei82875p->

bx_i82875p_c *thei82875p = NULL;
bx_i82875p_agp_c *thei82875p_agp = NULL;

// Host bridge device plugin entry point

PLUGIN_ENTRY_FOR_MODULE(i82875p)
{
  if (mode == PLUGIN_INIT) {
    thei82875p = new bx_i82875p_c();
    BX_REGISTER_DEVICE_DEVMODEL(plugin, type, thei82875p, BX_PLUGIN_I82875P);

    thei82875p_agp = new bx_i82875p_agp_c();
    BX_REGISTER_DEVICE_DEVMODEL(plugin, type, thei82875p_agp, BX_PLUGIN_I82875P_AGP);
  } else if (mode == PLUGIN_FINI) {
    delete thei82875p;
    delete thei82875p_agp;
  } else if (mode == PLUGIN_PROBE) {
    return (int)PLUGTYPE_CORE;
  }
  return 0; // Success
}

//
// i82875P Host bridge implementation
//

bx_i82875p_c::bx_i82875p_c()
{
  put("i82875p", "I82875P");
}

bx_i82875p_c::~bx_i82875p_c()
{
  BX_DEBUG(("Exit"));
}

void bx_i82875p_c::init(void)
{
  unsigned i;
  // Register the host bridge (device 0:0.0)
  BX_PCI_THIS init_pci_conf(0x8086, 0x2578, 0x02, 0x060000, 0x00);

  // Initialize the RAM mapping
  ram_size = SIM->get_param_num(BXPN_MEM_SIZE)->get();
  
  // Initialize DRAM module setup
  for (i = 0; i < 8; i++)
    BX_I82875P_THIS DRBA[i] = 0x0;
  
  // Set up memory mapping based on RAM size
  if (ram_size > 1024) ram_size = 1024;  // Max supported is 1GB
  Bit8u drbval = 0;
  unsigned row = 0;
  
  // Map memory by 128MB, 32MB, or 8MB chunks based on total size
  const Bit8u type[3] = {128, 32, 8};
  unsigned ti = 0;
  while ((ram_size > 0) && (row < 8) && (ti < 3)) {
    unsigned mc = ram_size / type[ti];
    ram_size = ram_size % type[ti];
    for (i = 0; i < mc; i++) {
      drbval += (type[ti] >> 3);
      BX_I82875P_THIS DRBA[row++] = drbval;
      if (row == 8) break;
    }
    ti++;
  }
  while (row < 8) {
    BX_I82875P_THIS DRBA[row++] = drbval;
  }
  
  for (i = 0; i < 8; i++)
    BX_PCI_THIS pci_conf[0x60 + i] = BX_I82875P_THIS DRBA[i];
  
  BX_I82875P_THIS dram_detect = 0;

  BX_INFO(("i82875P Host bridge initialized"));
}

void bx_i82875p_c::reset(unsigned type)
{
  unsigned i;

  BX_PCI_THIS pci_conf[0x04] = 0x06; // command register - enable I/O, MEM, master
  BX_PCI_THIS pci_conf[0x05] = 0x00;
  BX_PCI_THIS pci_conf[0x06] = 0x80; // status
  BX_PCI_THIS pci_conf[0x07] = 0x02;
  BX_PCI_THIS pci_conf[0x0d] = 0x00; // master latency timer
  BX_PCI_THIS pci_conf[0x0f] = 0x00;
  BX_PCI_THIS pci_conf[0x50] = 0x00;
  BX_PCI_THIS pci_conf[0x52] = 0x00;
  BX_PCI_THIS pci_conf[0x53] = 0x80;
  BX_PCI_THIS pci_conf[0x54] = 0x00;
  BX_PCI_THIS pci_conf[0x55] = 0x00;
  BX_PCI_THIS pci_conf[0x56] = 0x00;
  BX_PCI_THIS pci_conf[0x57] = 0x01;
  BX_PCI_THIS pci_conf[0x51] = 0x01;
  BX_PCI_THIS pci_conf[0x58] = 0x10;

  // Reset all PAM registers (disable all device memory)
  for (i=0x59; i<=0x5f; i++) {
    BX_PCI_THIS pci_conf[i] = 0x00;
    BX_I82875P_THIS pam[i-0x59] = 0x00;
  }
  
  BX_I82875P_THIS smram = 0x02;
  BX_I82875P_THIS esmramc = 0x38;
  BX_I82875P_THIS agpm = 0x00;
  BX_I82875P_THIS fpllcont = 0x00;
  BX_I82875P_THIS apsize = 0x00;
  BX_I82875P_THIS amtt = 0x10;
  BX_I82875P_THIS lptt = 0x10;
  BX_I82875P_THIS toud = 0x0400;
  BX_I82875P_THIS mchcfg = 0x0000;
  BX_I82875P_THIS errcmd = 0x0000;
  BX_I82875P_THIS smicmd = 0x0000;
  BX_I82875P_THIS scicmd = 0x0000;
  BX_I82875P_THIS skpd = 0x0000;
  BX_I82875P_THIS agpctrl = 0x00000000;
  BX_I82875P_THIS attbase = 0x00000000;
  
  BX_I82875P_THIS remap_memory();
}

void bx_i82875p_c::register_state(void)
{
  unsigned i;

  bx_list_c *list = new bx_list_c(SIM->get_bochs_root(), "i82875p", "i82875P Host Bridge State");
  
  bx_pci_device_c::register_state_pci(list);
  
  BXRS_PARAM_BOOL(list, dram_detect, BX_I82875P_THIS dram_detect);
  
  bx_list_c *pam = new bx_list_c(list, "PAM");
  for (i=0; i<8; i++) {
    char name[4];
    sprintf(name, "%d", i);
    new bx_shadow_num_c(pam, name, &BX_I82875P_THIS pam[i], BASE_HEX);
  }
  
  BXRS_HEX_PARAM_FIELD(list, smram, BX_I82875P_THIS smram);
  BXRS_HEX_PARAM_FIELD(list, esmramc, BX_I82875P_THIS esmramc);
  BXRS_HEX_PARAM_FIELD(list, agpm, BX_I82875P_THIS agpm);
  BXRS_HEX_PARAM_FIELD(list, fpllcont, BX_I82875P_THIS fpllcont);
  BXRS_HEX_PARAM_FIELD(list, apsize, BX_I82875P_THIS apsize);
  BXRS_HEX_PARAM_FIELD(list, amtt, BX_I82875P_THIS amtt);
  BXRS_HEX_PARAM_FIELD(list, lptt, BX_I82875P_THIS lptt);
  BXRS_HEX_PARAM_FIELD(list, toud, BX_I82875P_THIS toud);
  BXRS_HEX_PARAM_FIELD(list, mchcfg, BX_I82875P_THIS mchcfg);
  BXRS_HEX_PARAM_FIELD(list, errcmd, BX_I82875P_THIS errcmd);
  BXRS_HEX_PARAM_FIELD(list, smicmd, BX_I82875P_THIS smicmd);
  BXRS_HEX_PARAM_FIELD(list, scicmd, BX_I82875P_THIS scicmd);
  BXRS_HEX_PARAM_FIELD(list, skpd, BX_I82875P_THIS skpd);
  BXRS_HEX_PARAM_FIELD(list, agpctrl, BX_I82875P_THIS agpctrl);
  BXRS_HEX_PARAM_FIELD(list, attbase, BX_I82875P_THIS attbase);
}

void bx_i82875p_c::after_restore_state(void)
{
  BX_I82875P_THIS remap_memory();
}

void bx_i82875p_c::pci_write_handler(Bit8u address, Bit32u value, unsigned io_len)
{
  Bit8u value8, oldval;
  unsigned area;

  if ((address >= 0x10) && (address < 0x34))
    return;

  BX_DEBUG_PCI_WRITE(address, value, io_len);
  for (unsigned i=0; i<io_len; i++) {
    value8 = (value >> (i*8)) & 0xFF;
    oldval = BX_PCI_THIS pci_conf[address+i];
    switch (address+i) {
      case 0x04:
        BX_PCI_THIS pci_conf[address+i] = (value8 & 0x40) | 0x06;
        break;
      case 0x05:
        BX_PCI_THIS pci_conf[address+i] = (value8 & 0x01);
        break;
      case 0x07:
        BX_PCI_THIS pci_conf[address+i] &= ~(value8 & 0xf9);
        break;
      case 0x0d:
        BX_PCI_THIS pci_conf[address+i] = (value8 & 0xf8);
        break;
      case 0x50:
        BX_PCI_THIS pci_conf[address+i] = (value8 & 0xec);
        break;
      case 0x51:
        BX_PCI_THIS pci_conf[address+i] = (value8 & 0x8f) | 0x20;
        break;
      case 0x59:
      case 0x5A:
      case 0x5B:
      case 0x5C:
      case 0x5D:
      case 0x5E:
      case 0x5F:
        if (value8 != oldval) {
          BX_PCI_THIS pci_conf[address+i] = value8;
          BX_I82875P_THIS pam[address+i-0x59] = value8;
          BX_I82875P_THIS map_pam_memory(value8);
          BX_INFO(("i82875P: PAM register %x (TLB Flush)", address+i));
          bx_pc_system.MemoryMappingChanged();
        }
        break;
      case 0x60:
      case 0x61:
      case 0x62:
      case 0x63:
      case 0x64:
      case 0x65:
      case 0x66:
      case 0x67:
        BX_PCI_THIS pci_conf[address+i] = value8;
        break;
      case 0x72:
        BX_I82875P_THIS smram_control(value8); // SMRAM control register
        break;
      case 0x73:
        // Extended SMRAM control register (not fully implemented)
        BX_PCI_THIS pci_conf[address+i] = (value8 | 0x38);
        break;
      case 0x90: // PAM0 mapped to 0x59
        if (value8 != oldval) {
          BX_PCI_THIS pci_conf[0x59] = value8;
          BX_I82875P_THIS pam[0] = value8;
          BX_I82875P_THIS map_pam_memory(value8);
          BX_INFO(("i82875P: PAM register 0x59 (TLB Flush)"));
          bx_pc_system.MemoryMappingChanged();
        }
        break;
      default:
        BX_PCI_THIS pci_conf[address+i] = value8;
        BX_DEBUG(("i82875P PCI write register 0x%02x value 0x%02x", address+i, value8));
    }
  }
}

void bx_i82875p_c::smram_control(Bit8u value)
{
  //
  // [7:7] Reserved.
  // [6:6] SMM Space Open (DOPEN), when DOPEN=1 and DLCK=0, SMM space DRAM
  //       became visible even CPU not indicte SMM mode access. This is
  //       indended to help BIOS to initialize SMM space.
  // [5:5] SMM Space Closed (DCLS), when DCLS=1, SMM space is not accessible
  //       for data references, even if CPU indicates SMM mode access. Code
  //       references may still access SMM space DRAM.
  // [4:4] SMM Space Locked (DLCK), when DLCK=1, DOPEN is set to 0 and
  //       both DLCK and DOPEN became R/O. DLCK can only be cleared by
  //       a power-on reset.
  // [3:3] SMRAM Enable (SMRAME)
  // [2:0] SMM space base segment, program the location of SMM space
  //       reserved.
  //

  value = (value & 0x78) | 0x2; // ignore reserved bits (set to 0)

  if (BX_PCI_THIS pci_conf[0x72] & 0x10)
  {
    value &= 0xbf; // set DOPEN=0, DLCK=1
    value |= 0x10;
  }

  if ((value & 0x08) == 0) {
    bx_devices.mem->disable_smram();
  }
  else {
    bool DOPEN = (value & 0x40) > 0, DCLS = (value & 0x20) > 0;
    if(DOPEN && DCLS) BX_PANIC(("SMRAM control: DOPEN not mutually exclusive with DCLS !"));
    bx_devices.mem->enable_smram(DOPEN, DCLS);
  }

  BX_INFO(("SMRAM control register: 0x%02x", value));
  BX_PCI_THIS pci_conf[0x72] = value;
}

void bx_i82875p_c::map_pam_memory(Bit8u value)
{
  // PAM0 - BIOS area (0xF0000 - 0xFFFFF)
  // PAM1 - 0xC0000 - 0xC3FFF
  // PAM2 - 0xC4000 - 0xC7FFF
  // PAM3 - 0xC8000 - 0xCBFFF
  // PAM4 - 0xCC000 - 0xCFFFF
  // PAM5 - 0xD0000 - 0xD3FFF
  // PAM6 - 0xD4000 - 0xD7FFF
  // PAM7 - 0xD8000 - 0xDBFFF (not implemented in i82875P)

  // PAM(n) consists of two 4-bit values for the memory range:
  // bit 0-3: Lower 0 = DRAM disabled, 1 = Read-only, 2 = Write-only, 3 = Read/Write
  // bit 4-7: Upper 0 = DRAM disabled, 1 = Read-only, 2 = Write-only, 3 = Read/Write

  for (unsigned i = 0; i < 7; i++) { // PAM0 - PAM6
    unsigned char access_lower = BX_I82875P_THIS pam[i] & 0x0F;
    unsigned char access_upper = (BX_I82875P_THIS pam[i] >> 4) & 0x0F;
    
    // Convert PAM values to Bochs memory access bits
    unsigned read_l = (access_lower & 0x1) ? 1 : 0;
    unsigned write_l = (access_lower & 0x2) ? 1 : 0;
    unsigned read_u = (access_upper & 0x1) ? 1 : 0;
    unsigned write_u = (access_upper & 0x2) ? 1 : 0;

    if (i == 0) {
      // PAM0: F0000-FFFFF
      DEV_mem_set_memory_type(BX_MEM_AREA_F0000, read_l, write_l);
    } else {
      // PAM1-6: C0000-D7FFF in 16KB chunks
      unsigned area = (i - 1) * 2 + BX_MEM_AREA_C0000;
      DEV_mem_set_memory_type(area, read_l, write_l);
      DEV_mem_set_memory_type(area + 1, read_u, write_u);
    }
  }
}

void bx_i82875p_c::remap_memory(void)
{
  // Configure memory regions
  Bit32u top = BX_I82875P_THIS toud << 16;
  Bit32u ram_size = SIM->get_param_num(BXPN_MEM_SIZE)->get() * 1024 * 1024;

  if (BX_I82875P_THIS esmramc & 1) {
    switch((BX_I82875P_THIS esmramc >> 1) & 3) {
      case 2: top += 512*1024; break;
      case 3: top += 1024*1024; break;
    }
  }

  if(top > ram_size)
    top = ram_size;

  BX_INFO(("i82875P: Remapping memory, RAM top = 0x%08x", top));

  // Configure PAM areas
  for (unsigned i = 0; i < 7; i++) {
    map_pam_memory(BX_I82875P_THIS pam[i]);
  }

  // Set up SMRAM
  if (BX_I82875P_THIS smram & 0x40)
    bx_devices.mem->disable_smram();
  else
    smram_control(BX_I82875P_THIS smram); // Ensure SMRAM is properly configured
}

//
// i82875P AGP bridge implementation
//

bx_i82875p_agp_c::bx_i82875p_agp_c()
{
  put("i82875p_agp", "I82875P_AGP");
}

bx_i82875p_agp_c::~bx_i82875p_agp_c()
{
  BX_DEBUG(("Exit"));
}

void bx_i82875p_agp_c::init(void)
{
  // Register the PCI-AGP bridge (device 0:1.0)
  BX_PCI_THIS init_pci_conf(0x8086, 0x2579, 0x02, 0x060400, 0x01);
  BX_PCI_THIS pci_conf[0x06] = 0x20;
  BX_PCI_THIS pci_conf[0x07] = 0x02;
  BX_PCI_THIS pci_conf[0x1e] = 0xa0;

  BX_INFO(("i82875P AGP bridge initialized"));
}

void bx_i82875p_agp_c::reset(unsigned type)
{
  BX_PCI_THIS pci_conf[0x04] = 0x00;
  BX_PCI_THIS pci_conf[0x05] = 0x00;
  BX_PCI_THIS pci_conf[0x1c] = 0xf0;
  BX_PCI_THIS pci_conf[0x1f] = 0x02;
  BX_PCI_THIS pci_conf[0x20] = 0xf0;
  BX_PCI_THIS pci_conf[0x21] = 0xff;
  BX_PCI_THIS pci_conf[0x22] = 0x00;
  BX_PCI_THIS pci_conf[0x23] = 0x00;
  BX_PCI_THIS pci_conf[0x24] = 0xf0;
  BX_PCI_THIS pci_conf[0x25] = 0xff;
  BX_PCI_THIS pci_conf[0x26] = 0x00;
  BX_PCI_THIS pci_conf[0x27] = 0x00;
  BX_PCI_THIS pci_conf[0x3e] = 0x80;
}

void bx_i82875p_agp_c::register_state(void)
{
  bx_list_c *list = new bx_list_c(SIM->get_bochs_root(), "i82875p_agp", "i82875P AGP Bridge State");
  bx_pci_device_c::register_state_pci(list);
}

void bx_i82875p_agp_c::pci_write_handler(Bit8u address, Bit32u value, unsigned io_len)
{
  Bit8u value8, oldval;

  BX_DEBUG_PCI_WRITE(address, value, io_len);

  for (unsigned i=0; i<io_len; i++) {
    value8 = (value >> (i*8)) & 0xff;
    oldval = BX_PCI_THIS pci_conf[address+i];
    switch (address+i) {
      case 0x04: // PCICMD1
        value8 &= 0x3f;
        break;
      case 0x05:
        value8 &= 0x01;
        break;
      case 0x0d: // MLT1
      case 0x1b: // SMLT
        value8 &= 0xf8;
        break;
      case 0x1c: // IOBASE
      case 0x1d: // IOLIMIT
      case 0x20: // MBASE lo
      case 0x22: // MLIMIT lo
      case 0x24: // PMBASE lo
      case 0x26: // PMLIMIT lo
        value8 &= 0xf0;
        break;
      case 0x1f: // SSTS hi
        value8 = (oldval & ~value8) | 0x02;
        break;
      case 0x3e: // BCTRL
        value8 = (value8 & 0xc9) | 0x80;
        break;
      case 0x19: // SBUSN - all bits r/w
      case 0x1a: // SUBUSN
      case 0x21: // MBASE hi
      case 0x23: // MLIMIT hi
      case 0x25: // PMBASE hi
      case 0x27: // PMLIMIT hi
        break;
      case 0x06: // PCISTS1 - all bits r/o
      case 0x07:
      case 0x18: // PBUSN
      case 0x1e: // SSTS lo
      default:
        value8 = oldval;
    }
    BX_PCI_THIS pci_conf[address+i] = value8;
  }
}

#endif /* BX_SUPPORT_PCI */