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

#ifndef BX_I82875P_H
#define BX_I82875P_H

#if BX_USE_I82875P_SMF
#  define BX_I82875P_SMF static
#  define BX_I82875P_THIS thei82875p->
#else
#  define BX_I82875P_SMF
#  define BX_I82875P_THIS this->
#endif

// Host bridge (Device 0:0.0)
class bx_i82875p_c : public bx_pci_device_c {
public:
  bx_i82875p_c();
  virtual ~bx_i82875p_c();
  virtual void init(void);
  virtual void reset(unsigned type);
  virtual void register_state(void);
  virtual void after_restore_state(void);

  virtual void pci_write_handler(Bit8u address, Bit32u value, unsigned io_len);

private:
  Bit8u  pam[8];        // PAM0 ... PAM7 register
  Bit8u  smram;         // SMRAM control register
  Bit8u  esmramc;       // Extended SMRAM control
  Bit8u  agpm;          // AGP command register
  Bit8u  fpllcont;      // FPLL control register
  Bit8u  apsize;        // AGP aperture size register
  Bit8u  amtt;
  Bit8u  lptt;
  Bit16u toud;          // Top of usable DRAM
  Bit16u mchcfg;        // MCHCFG Register
  Bit16u errcmd;        // Error Command Register
  Bit16u smicmd;        // SMI Command Register
  Bit16u scicmd;        // SCI Command Register
  Bit16u skpd;          // Scratch Pad Register
  
  Bit32u agpctrl;       // AGP control register
  Bit32u attbase;       // Aperture Translation Table base
  
  unsigned ram_size;
  Bit8u dram_detect;
  Bit8u DRBA[8];        // DRAM Row Boundary Address

  BX_I82875P_SMF void smram_control(Bit8u value);
  BX_I82875P_SMF void map_pam_memory(Bit8u value);
  BX_I82875P_SMF void remap_memory(void);
};

// PCI-AGP bridge (Device 0:1.0)
class bx_i82875p_agp_c : public bx_pci_device_c {
public:
  bx_i82875p_agp_c();
  virtual ~bx_i82875p_agp_c();
  virtual void init(void);
  virtual void reset(unsigned type);
  virtual void register_state(void);
  
  virtual void pci_write_handler(Bit8u address, Bit32u value, unsigned io_len);
};

#endif