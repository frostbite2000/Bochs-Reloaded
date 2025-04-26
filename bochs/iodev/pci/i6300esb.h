/////////////////////////////////////////////////////////////////////////
// $Id$
/////////////////////////////////////////////////////////////////////////
//
// Intel i6300ESB southbridge emulation
// i6300ESB LPC/ISA bridge
// i6300ESB watchdog timer
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

#ifndef BX_I6300ESB_H
#define BX_I6300ESB_H

#if BX_USE_I6300ESB_SMF
#  define BX_I6300ESB_SMF static
#  define BX_I6300ESB_THIS thei6300esb->
#else
#  define BX_I6300ESB_SMF
#  define BX_I6300ESB_THIS this->
#endif

// Watchdog device (Device 1:5.0)
class bx_i6300esb_wdog_c : public bx_pci_device_c {
public:
  bx_i6300esb_wdog_c();
  virtual ~bx_i6300esb_wdog_c();
  virtual void init(void);
  virtual void reset(unsigned type);
  virtual void register_state(void);

  virtual void pci_write_handler(Bit8u address, Bit32u value, unsigned io_len);

private:
  // empty for now
};

// LPC/ISA bridge (Device 1:31.0)
class bx_i6300esb_lpc_c : public bx_pci_device_c {
public:
  bx_i6300esb_lpc_c();
  virtual ~bx_i6300esb_lpc_c();
  virtual void init(void);
  virtual void reset(unsigned type);
  virtual void register_state(void);

  virtual void pci_write_handler(Bit8u address, Bit32u value, unsigned io_len);
  virtual void after_restore_state(void);

private:
  Bit32u pmbase, gpio_base, fwh_sel1, gen_cntl, etr1, rst_cnt2, gpi_rout;
  Bit16u bios_cntl, pci_dma_cfg, gen1_dec, lpc_en, gen2_dec, fwh_sel2, func_dis, gen_pmcon_1;
  Bit16u mon_trp_rng[4], mon_trp_msk;
  Bit8u pirq_rout[8];
  Bit8u acpi_cntl, tco_cntl, gpio_cntl, serirq_cntl, d31_err_cfg, d31_err_sts, gen_sta, back_cntl, rtc_conf;
  Bit8u lpc_if_com_range, lpc_if_fdd_lpt_range, lpc_if_sound_range, fwh_dec_en1, fwh_dec_en2, siu_config_port;
  Bit8u gen_pmcon_2, gen_pmcon_3, apm_cnt, apm_sts, mon_fwd_en, nmi_sc;
  int siu_config_state;

  BX_I6300ESB_SMF void map_bios(Bit32u addr, bool readonly);
};

#endif