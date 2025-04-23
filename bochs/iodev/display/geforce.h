/////////////////////////////////////////////////////////////////////////
// $Id$
/////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2025  The Bochs Project
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
//
/////////////////////////////////////////////////////////////////////////

#if BX_SUPPORT_GEFORCE

#if BX_USE_GEFORCE_SMF
#  define BX_GEFORCE_SMF  static
#  define BX_GEFORCE_THIS theSvga->
#  define BX_GEFORCE_THIS_PTR theSvga
#else
#  define BX_GEFORCE_SMF
#  define BX_GEFORCE_THIS this->
#  define BX_GEFORCE_THIS_PTR this
#endif // BX_USE_GEFORCE_SMF

// 0x3b4,0x3d4
#define VGA_CRTC_MAX 0x18
#define GEFORCE_CRTC_MAX 0x9F

// NV20-specific definitions
#define NV20_3D_CLASS 0x0097
#define NV20_VERTEX_PROGRAM_MAX_INSTRUCTIONS 256
#define NV20_VERTEX_PROGRAM_MAX_CONSTANTS 192
#define NV20_TEXTURE_UNITS 4
#define NV20_REGISTER_COMBINERS_STAGES 8
#define NV20_MAX_VERTEX_ATTRIBS 16

#define GEFORCE_CHANNEL_COUNT 32
#define GEFORCE_SUBCHANNEL_COUNT 8
#define GEFORCE_CACHE1_SIZE 64

#define BX_ROP_PATTERN 0x01

// Define maximum work threads for rasterizer
#define WORK_MAX_THREADS 8

// Define vertex structure (vertex processor I/O)
struct vertex_nv {
  union {
    float fv[4];
    Bit32u iv[4];
  } attribute[NV20_MAX_VERTEX_ATTRIBS];
};

// Define rasterizer vertex structure
struct nv2avertex_t {
  // Position
  float x, y, z, w;
  
  // Color (primary)
  float r, g, b, a;
  
  // Color (secondary)
  float sr, sg, sb;
  
  // Fog
  float fog;
  
  // Texture coordinates for up to 4 texture units
  float s[4], t[4], r[4], q[4];
};

// Forward declaration for poly_manager
class nv2a_rasterizer;
struct nvidia_object_data;

// For vertex transform parameter indices
enum VERTEX_PARAMETER {
  PARAM_COLOR_B = 0,
  PARAM_COLOR_G,
  PARAM_COLOR_R,
  PARAM_COLOR_A,
  PARAM_TEXTURE0_S,
  PARAM_TEXTURE0_T,
  PARAM_TEXTURE0_R,
  PARAM_TEXTURE0_Q,
  PARAM_TEXTURE1_S,
  PARAM_TEXTURE1_T,
  PARAM_TEXTURE1_R,
  PARAM_TEXTURE1_Q,
  PARAM_TEXTURE2_S,
  PARAM_TEXTURE2_T,
  PARAM_TEXTURE2_R,
  PARAM_TEXTURE2_Q,
  PARAM_TEXTURE3_S,
  PARAM_TEXTURE3_T,
  PARAM_TEXTURE3_R,
  PARAM_TEXTURE3_Q,
  PARAM_SECONDARY_COLOR_B,
  PARAM_SECONDARY_COLOR_G,
  PARAM_SECONDARY_COLOR_R,
  PARAM_SECONDARY_COLOR_A,
  PARAM_Z,
  PARAM_1W,
  ALL
};

// For primitive types
enum NV2A_BEGIN_END {
  STOP = 0,
  POINTS = 1,
  LINES = 2,
  LINE_LOOP = 3,
  LINE_STRIP = 4,
  TRIANGLES = 5,
  TRIANGLE_STRIP = 6,
  TRIANGLE_FAN = 7,
  QUADS = 8,
  QUAD_STRIP = 9,
  POLYGON = 10
};

// For blending operations
enum NV2A_BLEND_EQUATION {
  FUNC_ADD = 0x8006,
  MIN = 0x8007,
  MAX = 0x8008,
  FUNC_SUBTRACT = 0x800a,
  FUNC_REVERSE_SUBTRACT = 0x80b
};

// For blending factors
enum NV2A_BLEND_FACTOR {
  ZERO = 0x0000,
  ONE = 0x0001,
  SRC_COLOR = 0x0300,
  ONE_MINUS_SRC_COLOR = 0x0301,
  SRC_ALPHA = 0x0302,
  ONE_MINUS_SRC_ALPHA = 0x0303,
  DST_ALPHA = 0x0304,
  ONE_MINUS_DST_ALPHA = 0x0305,
  DST_COLOR = 0x0306,
  ONE_MINUS_DST_COLOR = 0x0307,
  SRC_ALPHA_SATURATE = 0x0308,
  CONSTANT_COLOR = 0x8001,
  ONE_MINUS_CONSTANT_COLOR = 0x8002,
  CONSTANT_ALPHA = 0x8003,
  ONE_MINUS_CONSTANT_ALPHA = 0x8004
};

// For depth/stencil test functions
enum NV2A_COMPARISON_OP {
  NEVER = 0x0200,
  LESS = 0x0201,
  EQUAL = 0x0202,
  LEQUAL = 0x0203,
  GREATER = 0x0204,
  NOTEQUAL = 0x0205,
  GEQUAL = 0x0206,
  ALWAYS = 0x0207
};

// For stencil operations
enum NV2A_STENCIL_OP {
  ZEROOP = 0x0000,
  INVERTOP = 0x150a,
  KEEP = 0x1e00,
  REPLACE = 0x1e01,
  INCR = 0x1e02,
  DECR = 0x1e03,
  INCR_WRAP = 0x8507,
  DECR_WRAP = 0x8508
};

// For front face determination
enum NV2A_GL_FRONT_FACE {
  CW = 0x0900,
  CCW = 0x0901
};

// For face culling
enum NV2A_GL_CULL_FACE {
  FRONT = 0x0404,
  BACK = 0x0405,
  FRONT_AND_BACK = 0x0408
};

// For texture formats
enum NV2A_TEX_FORMAT {
  L8 = 0x0,
  I8 = 0x1,
  A1R5G5B5 = 0x2,
  A4R4G4B4 = 0x4,
  R5G6B5 = 0x5,
  A8R8G8B8 = 0x6,
  X8R8G8B8 = 0x7,
  INDEX8 = 0xb,
  DXT1 = 0xc,
  DXT3 = 0xe,
  DXT5 = 0xf,
  A1R5G5B5_RECT = 0x10,
  R5G6B5_RECT = 0x11,
  A8R8G8B8_RECT = 0x12,
  L8_RECT = 0x13,
  DSDT8_RECT = 0x17,
  A8 = 0x19,
  A8L8 = 0x1a,
  I8_RECT = 0x1b,
  A4R4G4B4_RECT = 0x1d,
  R8G8B8_RECT = 0x1e,
  A8L8_RECT = 0x20,
  Z24 = 0x2a,
  Z24_RECT = 0x2b,
  Z16 = 0x2c,
  Z16_RECT = 0x2d,
  DSDT8 = 0x28,
  HILO16 = 0x33,
  HILO16_RECT = 0x36,
  HILO8 = 0x44,
  SIGNED_HILO8 = 0x45,
  HILO8_RECT = 0x46,
  SIGNED_HILO8_RECT = 0x47
};

// For render target types
enum NV2A_RT_TYPE {
  LINEAR = 1,
  SWIZZLED = 2
};

// For depth formats
enum NV2A_RT_DEPTH_FORMAT {
  Z16 = 0x0001,
  Z24S8 = 0x0002
};

// For color formats
enum NV2A_COLOR_FORMAT {
  X1R5G5B5_Z1R5G5B5 = 1,
  X1R5G5B5_X1R5G5B5 = 2,
  R5G6B5 = 3,
  X8R8G8B8_Z8R8G8B8 = 4,
  X8R8G8B8_X8R8G8B8 = 5,
  X1A7R8G8B8_Z1A7R8G8B8 = 6,
  X1A7R8G8B8_X1A7R8G8B8 = 7,
  A8R8G8B8 = 8,
  B8 = 9,
  G8B8 = 10
};

// For logical operations
enum NV2A_LOGIC_OP {
  CLEAR = 0x1500,
  AND = 0x1501,
  AND_REVERSE = 0x1502,
  COPY = 0x1503,
  AND_INVERTED = 0x1504,
  NOOP = 0x1505,
  XOR = 0x1506,
  OR = 0x1507,
  NOR = 0x1508,
  EQUIV = 0x1509,
  INVERT = 0x150a,
  OR_REVERSE = 0x150b,
  COPY_INVERTED = 0x150c,
  OR_INVERTED = 0x150d,
  NAND = 0x150e,
  SET = 0x150f
};

// Define the rasterizer base class
class nv2a_rasterizer {
public:
  struct extent_t {
    Bit32s startx, stopx;
    struct {
      float dx, w;
    } param[26];
  };
  
  typedef void (*render_delegate)(int32_t scanline, const extent_t &extent, const nvidia_object_data &objectdata, int threadid);
  
  nv2a_rasterizer() {}
  virtual ~nv2a_rasterizer() {}
  
  void render_triangle(const rectangle &cliprect, render_delegate callback, 
                      const nv2avertex_t &v1, const nv2avertex_t &v2, const nv2avertex_t &v3);
  
  nvidia_object_data &object_data() { return m_object_data; }
  
private:
  nvidia_object_data m_object_data;
};

// Define simple NV2A register combiner interface
struct Combiner {
  enum class InputRegister {
    Zero = 0,
    Color0,
    Color1,
    FogColor,
    PrimaryColor,
    SecondaryColor,
    Texture0Color = 8,
    Texture1Color,
    Texture2Color,
    Texture3Color,
    Spare0,
    Spare1,
    SumClamp,
    EF
  };
  
  enum class MapFunction {
    UnsignedIdentity = 0,
    UnsignedInvert,
    ExpandNormal,
    ExpandNegate,
    HalfBiasNormal,
    HalfBiasNegate,
    SignedIdentyty,
    SignedNegate
  };
  
  // Register combiner working state (per thread)
  struct {
    struct {
      float A[4]; // 0=R 1=G 2=B 3=A
      float B[4];
      float C[4];
      float D[4];
      float E[4];
      float F[4];
      float G;
      float EF[4];
      float sumclamp[4];
    } variables;
    struct {
      float RGBop1[4]; // 0=R 1=G 2=B
      float RGBop2[4];
      float RGBop3[4];
      float Aop1;
      float Aop2;
      float Aop3;
    } functions;
    struct {
      float primarycolor[4]; // rw
      float secondarycolor[4];
      float texture0color[4];
      float texture1color[4];
      float texture2color[4];
      float texture3color[4];
      float color0[4];
      float color1[4];
      float spare0[4];
      float spare1[4];
      float fogcolor[4]; // ro
      float zero[4];
    } registers;
    float output[4];
  } work[WORK_MAX_THREADS];
  
  // Register combiner configuration
  struct {
    struct {
      float constantcolor0[4];
      float constantcolor1[4];
      struct {
        InputRegister A_input;
        int A_component;
        MapFunction A_mapping;
        InputRegister B_input;
        int B_component;
        MapFunction B_mapping;
        InputRegister C_input;
        int C_component;
        MapFunction C_mapping;
        InputRegister D_input;
        int D_component;
        MapFunction D_mapping;
      } mapin_alpha;
      struct {
        InputRegister A_input;
        int A_component;
        MapFunction A_mapping;
        InputRegister B_input;
        int B_component;
        MapFunction B_mapping;
        InputRegister C_input;
        int C_component;
        MapFunction C_mapping;
        InputRegister D_input;
        int D_component;
        MapFunction D_mapping;
      } mapin_rgb;
      struct {
        InputRegister CD_output;
        InputRegister AB_output;
        InputRegister SUM_output;
        int CD_dotproduct;
        int AB_dotproduct;
        int muxsum;
        int bias;
        int scale;
      } mapout_alpha;
      struct {
        InputRegister CD_output;
        InputRegister AB_output;
        InputRegister SUM_output;
        int CD_dotproduct;
        int AB_dotproduct;
        int muxsum;
        int bias;
        int scale;
      } mapout_rgb;
    } stage[8];
    struct {
      float constantcolor0[4];
      float constantcolor1[4];
      int color_sum_clamp;
      struct {
        InputRegister G_input;
        int G_component;
        MapFunction G_mapping;
      } mapin_alpha;
      struct {
        InputRegister A_input;
        int A_component;
        MapFunction A_mapping;
        InputRegister B_input;
        int B_component;
        MapFunction B_mapping;
        InputRegister C_input;
        int C_component;
        MapFunction C_mapping;
        InputRegister D_input;
        int D_component;
        MapFunction D_mapping;
        InputRegister E_input;
        int E_component;
        MapFunction E_mapping;
        InputRegister F_input;
        int F_component;
        MapFunction F_mapping;
      } mapin_rgb;
    } final;
    int stages;
  } setup;
  
  int used;
};

// Extent data structure for triangle rasterization
struct nvidia_object_data
{
  void* data;
};

class bx_geforce_c : public bx_vgacore_c
{
public:
  bx_geforce_c();
  virtual ~bx_geforce_c();

  virtual bool init_vga_extension(void);
  virtual void get_crtc_params(bx_crtc_params_t* crtcp, Bit32u* vclock);
  virtual void reset(unsigned type);
  virtual void redraw_area(unsigned x0, unsigned y0,
                           unsigned width, unsigned height);
  void redraw_area(Bit32s x0, Bit32s y0,
                   Bit32u width, Bit32u height);
  virtual Bit8u mem_read(bx_phy_address addr);
  virtual void mem_write(bx_phy_address addr, Bit8u value);
  virtual void get_text_snapshot(Bit8u **text_snapshot,
                                 unsigned *txHeight, unsigned *txWidth);
  virtual void register_state(void);
  virtual void after_restore_state(void);

#if BX_SUPPORT_PCI
  virtual void pci_write_handler(Bit8u address, Bit32u value, unsigned io_len);
#endif
#if BX_DEBUGGER
  virtual void debug_dump(int argc, char **argv);
#endif

protected:
  virtual void update(void);

private:
  static Bit32u svga_read_handler(void *this_ptr, Bit32u address, unsigned io_len);
  static void   svga_write_handler(void *this_ptr, Bit32u address, Bit32u value, unsigned io_len);
#if !BX_USE_CIRRUS_SMF
  Bit32u svga_read(Bit32u address, unsigned io_len);
  void   svga_write(Bit32u address, Bit32u value, unsigned io_len);
#endif
  BX_GEFORCE_SMF void   svga_init_members();
  BX_GEFORCE_SMF void   bitblt_init();

  BX_GEFORCE_SMF void draw_hardware_cursor(unsigned, unsigned, bx_svga_tileinfo_t *);

  // 0x3b4-0x3b5,0x3d4-0x3d5
  BX_GEFORCE_SMF Bit8u svga_read_crtc(Bit32u address, unsigned index);
  BX_GEFORCE_SMF void  svga_write_crtc(Bit32u address, unsigned index, Bit8u value);

  BX_GEFORCE_SMF void set_irq_level(bool level);
  BX_GEFORCE_SMF Bit32u get_mc_intr();
  BX_GEFORCE_SMF void update_irq_level();

  BX_GEFORCE_SMF Bit8u register_read8(Bit32u address);
  BX_GEFORCE_SMF void  register_write8(Bit32u address, Bit8u value);
  BX_GEFORCE_SMF Bit32u register_read32(Bit32u address);
  BX_GEFORCE_SMF void  register_write32(Bit32u address, Bit32u value);

  BX_GEFORCE_SMF Bit8u vram_read8(Bit32u address);
  BX_GEFORCE_SMF Bit16u vram_read16(Bit32u address);
  BX_GEFORCE_SMF Bit32u vram_read32(Bit32u address);
  BX_GEFORCE_SMF void vram_write8(Bit32u address, Bit8u value);
  BX_GEFORCE_SMF void vram_write16(Bit32u address, Bit16u value);
  BX_GEFORCE_SMF void vram_write32(Bit32u address, Bit32u value);
  BX_GEFORCE_SMF void vram_write64(Bit32u address, Bit64u value);
  BX_GEFORCE_SMF Bit8u ramin_read8(Bit32u address);
  BX_GEFORCE_SMF Bit32u ramin_read32(Bit32u address);
  BX_GEFORCE_SMF void ramin_write8(Bit32u address, Bit8u value);
  BX_GEFORCE_SMF void ramin_write32(Bit32u address, Bit32u value);
  BX_GEFORCE_SMF Bit8u physical_read8(Bit32u address);
  BX_GEFORCE_SMF Bit16u physical_read16(Bit32u address);
  BX_GEFORCE_SMF Bit32u physical_read32(Bit32u address);
  BX_GEFORCE_SMF void physical_write8(Bit32u address, Bit8u value);
  BX_GEFORCE_SMF void physical_write32(Bit32u address, Bit32u value);
  BX_GEFORCE_SMF void physical_write64(Bit32u address, Bit64u value);
  BX_GEFORCE_SMF Bit8u dma_read8(Bit32u object, Bit32u address);
  BX_GEFORCE_SMF Bit16u dma_read16(Bit32u object, Bit32u address);
  BX_GEFORCE_SMF Bit32u dma_read32(Bit32u object, Bit32u address);
  BX_GEFORCE_SMF void dma_write8(Bit32u object, Bit32u address, Bit8u value);
  BX_GEFORCE_SMF void dma_write16(Bit32u object, Bit32u address, Bit16u value);
  BX_GEFORCE_SMF void dma_write32(Bit32u object, Bit32u address, Bit32u value);
  BX_GEFORCE_SMF void dma_write64(Bit32u object, Bit32u address, Bit64u value);
  BX_GEFORCE_SMF Bit32u dma_pt_lookup(Bit32u object, Bit32u address);
  BX_GEFORCE_SMF Bit32u dma_lin_lookup(Bit32u object, Bit32u address);

  BX_GEFORCE_SMF Bit64u get_current_time();

  BX_GEFORCE_SMF Bit32u ramht_lookup(Bit32u handle, Bit32u chid);

  BX_GEFORCE_SMF void execute_command(Bit32u chid, Bit32u subc, Bit32u method, Bit32u param);

  BX_GEFORCE_SMF void execute_clip(Bit32u chid, Bit32u method, Bit32u param);
  BX_GEFORCE_SMF void execute_m2mf(Bit32u chid, Bit32u subc, Bit32u method, Bit32u param);
  BX_GEFORCE_SMF void execute_rop(Bit32u chid, Bit32u method, Bit32u param);
  BX_GEFORCE_SMF void execute_patt(Bit32u chid, Bit32u method, Bit32u param);
  BX_GEFORCE_SMF void execute_gdi(Bit32u chid, Bit32u method, Bit32u param);
  BX_GEFORCE_SMF void execute_imageblit(Bit32u chid, Bit8u cls, Bit32u method, Bit32u param);
  BX_GEFORCE_SMF void execute_ifc(Bit32u chid, Bit8u cls, Bit32u method, Bit32u param);
  BX_GEFORCE_SMF void execute_surf2d(Bit32u chid, Bit32u method, Bit32u param);
  BX_GEFORCE_SMF void execute_iifc(Bit32u chid, Bit32u method, Bit32u param);
  
  // NV20-specific method execution functions
  BX_GEFORCE_SMF void execute_nv20_3d(Bit32u chid, Bit32u method, Bit32u param);
  BX_GEFORCE_SMF void nv20_execute_vertex_program(vertex_nv *vertex_in, vertex_nv *vertex_out);
  BX_GEFORCE_SMF Bit32u texture_get_cubemap_texel(int unit, float x, float y, float z);
  BX_GEFORCE_SMF void convert_vertices(vertex_nv *source, nv2avertex_t *destination);

  BX_GEFORCE_SMF Bit32u color_565_to_888(Bit16u value);
  BX_GEFORCE_SMF void gdi_fillrect(Bit32u chid, bool clipped);
  BX_GEFORCE_SMF void gdi_blit(Bit32u chid, Bit32u type);
  BX_GEFORCE_SMF void ifc(Bit32u chid);
  BX_GEFORCE_SMF void iifc(Bit32u chid);
  BX_GEFORCE_SMF void copyarea(Bit32u chid);
  BX_GEFORCE_SMF void move(Bit32u chid);
  BX_GEFORCE_SMF void assemble_primitive(int source, int count);
  
  // NV20 shader helpers
  BX_GEFORCE_SMF void nv20_extract_combiner_input(Bit32u mapping, float registers[13][4], float output[4], int is_alpha);
  BX_GEFORCE_SMF void nv20_argb8_to_float(Bit32u argb, float rgba[4]);
  BX_GEFORCE_SMF Bit32u nv20_float_to_argb8(float rgba[4]);
  
  // Rasterizer render functions
  BX_GEFORCE_SMF void render_register_combiners_nv20(int32_t scanline, const nv2a_rasterizer::extent_t &extent, 
                                                   const nvidia_object_data &objectdata, int threadid);
  BX_GEFORCE_SMF void render_texture_simple(int32_t scanline, const nv2a_rasterizer::extent_t &extent, 
                                          const nvidia_object_data &objectdata, int threadid);
  BX_GEFORCE_SMF void render_color(int32_t scanline, const nv2a_rasterizer::extent_t &extent, 
                                 const nvidia_object_data &objectdata, int threadid);

  // Register structures for CRTC and other controllers
  struct {
    Bit8u index;
    Bit8u reg[GEFORCE_CRTC_MAX+1];
  } crtc; // 0x3b4-5/0x3d4-5

  // Core NVIDIA controller registers
  Bit32u mc_intr_en;
  Bit32u mc_enable;
  Bit32u fifo_intr;
  Bit32u fifo_intr_en;
  Bit32u fifo_ramht;
  Bit32u fifo_ramfc;
  Bit32u fifo_ramro;
  Bit32u fifo_mode;
  Bit32u fifo_cache1_push1;
  Bit32u fifo_cache1_put;
  Bit32u fifo_cache1_dma_instance;
  Bit32u fifo_cache1_dma_put;
  Bit32u fifo_cache1_dma_get;
  Bit32u fifo_cache1_ref_cnt;
  Bit32u fifo_cache1_pull0;
  Bit32u fifo_cache1_get;
  Bit32u fifo_cache1_method[GEFORCE_CACHE1_SIZE];
  Bit32u fifo_cache1_data[GEFORCE_CACHE1_SIZE];
  Bit32u rma_addr;
  Bit32u timer_intr;
  Bit32u timer_intr_en;
  Bit32u timer_num;
  Bit32u timer_den;
  Bit64u timer_inittime1;
  Bit64u timer_inittime2;
  Bit32u timer_alarm;
  Bit32u straps0_primary;
  Bit32u straps0_primary_original;
  Bit32u graph_intr;
  Bit32u graph_intr_en;
  Bit32u graph_status;
  Bit32u graph_fifo;
  Bit32u graph_channel_ctx_table;
  Bit32u crtc_intr;
  Bit32u crtc_intr_en;
  Bit32u crtc_start;
  Bit32u crtc_config;
  Bit32u crtc_cursor_offset;
  Bit32u crtc_cursor_config;
  Bit32u ramdac_cu_start_pos;
  Bit32u ramdac_vpll;
  Bit32u ramdac_vpll_b;
  Bit32u ramdac_pll_select;

  // BitBLT ROP handling
  bx_bitblt_rop_t rop_handler[0x100];
  Bit8u  rop_flags[0x100];

  // Channel state structure
  struct {
    Bit32u subr_return;
    bool subr_active;
    struct {
      Bit32u mthd;
      Bit32u subc;
      Bit32u mcnt;
      bool ni;
    } dma_state;
    struct {
      Bit32u object;
      Bit8u engine;
      Bit32u notifier;
    } schs[GEFORCE_SUBCHANNEL_COUNT];

    bool notify_pending;
    Bit32u notify_type;

    // Surface 2D state
    Bit32u s2d_img_src;
    Bit32u s2d_img_dst;
    Bit32u s2d_color_fmt;
    Bit32u s2d_color_bytes;
    Bit32u s2d_pitch;
    Bit32u s2d_ofs_src;
    Bit32u s2d_ofs_dst;

    // Image From CPU state
    Bit32u ifc_operation;
    Bit32u ifc_color_fmt;
    Bit32u ifc_color_bytes;
    Bit32u ifc_yx;
    Bit32u ifc_dhw;
    Bit32u ifc_shw;
    Bit32u ifc_words_ptr;
    Bit32u ifc_words_left;
    Bit32u* ifc_words;

    // Image Index From CPU state
    Bit32u iifc_palette;
    Bit32u iifc_color_fmt;
    Bit32u iifc_color_bytes;
    Bit32u iifc_bpp4;
    Bit32u iifc_yx;
    Bit32u iifc_dhw;
    Bit32u iifc_shw;
    Bit32u iifc_words_ptr;
    Bit32u iifc_words_left;
    Bit32u* iifc_words;

    // ImageBlit state
    Bit32u blit_operation;
    Bit32u blit_syx;
    Bit32u blit_dyx;
    Bit32u blit_hw;

    // Memory-to-memory state
    Bit32u m2mf_src;
    Bit32u m2mf_dst;
    Bit32u m2mf_src_offset;
    Bit32u m2mf_dst_offset;
    Bit32u m2mf_src_pitch;
    Bit32u m2mf_dst_pitch;
    Bit32u m2mf_line_length;
    Bit32u m2mf_line_count;
    Bit32u m2mf_format;
    Bit32u m2mf_buffer_notify;

    // Raster operation state
    Bit8u  rop;

    // Pattern state
    Bit32u patt_bg_color;
    Bit32u patt_fg_color;

    // GDI state
    Bit32u gdi_operation;
    Bit32u gdi_color_fmt;
    Bit32u gdi_rect_color;
    Bit32u gdi_rect_clip_yx0;
    Bit32u gdi_rect_clip_yx1;
    Bit32u gdi_rect_xy;
    Bit32u gdi_rect_yx0;
    Bit32u gdi_rect_yx1;
    Bit32u gdi_rect_wh;
    Bit32u gdi_bg_color;
    Bit32u gdi_fg_color;
    Bit32u gdi_image_swh;
    Bit32u gdi_image_dwh;
    Bit32u gdi_image_xy;
    Bit32u gdi_words_ptr;
    Bit32u gdi_words_left;
    Bit32u* gdi_words;
    
    // NV20-specific state (GeForce3 Ti 500 only)
    struct {
      // Vertex program (shader) state
      struct {
        Bit32u instructions[NV20_VERTEX_PROGRAM_MAX_INSTRUCTIONS][4]; // Instruction storage
        Bit32u constants[NV20_VERTEX_PROGRAM_MAX_CONSTANTS][4]; // Constant storage
        Bit32u program_start; // Entry point for program execution
        bool enabled;         // Whether vertex program is enabled
        
        // Upload state tracking
        Bit32u upload_instruction_slot;
        Bit32u upload_instruction_component;
        Bit32u upload_constant_slot;
        Bit32u upload_constant_component;
      } vertex_program;
      
      // Register combiners (pixel shader) state
      struct {
        bool enabled;
        Bit32u stages;
        struct {
          Bit32u input_mapping[4]; // Input register mappings A,B,C,D
          Bit32u output_mapping;   // Output mapping configuration
          Bit32u constant_color0;  // Stage constant color 0
          Bit32u constant_color1;  // Stage constant color 1
        } combiner_stage[NV20_REGISTER_COMBINERS_STAGES];
        
        struct {
          Bit32u input_mapping;  // Final combiner input mapping
          Bit32u output_mapping; // Final combiner output mapping
          Bit32u constant;       // Final combiner constant
        } final_combiner;
      } register_combiners;
      
      // Texture units specific to NV20
      struct {
        bool cube_mapping;     // Whether this unit supports cube mapping
        bool shadow_mapping;   // Whether this unit supports shadow mapping
        bool enabled;          // Whether this texture unit is enabled
      } texture_unit[NV20_TEXTURE_UNITS];
      
    } nv20;
  } chs[GEFORCE_CHANNEL_COUNT];

  // Miscellaneous register storage
  Bit32u unk_regs[4*1024*1024]; // temporary

  // SVGA state
  bool svga_unlock_special;
  bool svga_needs_update_tile;
  bool svga_needs_update_dispentire;
  bool svga_needs_update_mode;
  bool svga_double_width;

  // Device and rendering state
  Bit8u devfunc;
  unsigned svga_xres;
  unsigned svga_yres;
  unsigned svga_pitch;
  unsigned svga_bpp;
  unsigned svga_dispbpp;

  Bit32u card_type;
  Bit32u memsize_mask;
  Bit32u bar2_size;
  Bit32u ramin_flip;

  Bit8u *disp_ptr;
  Bit32u bank_base[2];

  // Hardware cursor state
  struct {
    Bit32u offset;
    Bit16s x, y;
    Bit8u size;
    bool bpp32;
    bool enabled;
  } hw_cursor;

  // Redraw area tracking
  struct {
    Bit16u x, y, w, h;
  } redraw;

  // Display Data Channel
  bx_ddc_c ddc;
  
  // Rasterization state
  nv2a_rasterizer rasterizer;
  nvidia_object_data *objectdata;
  Combiner combiner;
  
  // Renderer properties
  Bit32u color_mask;
  bool backface_culling_enabled;
  NV2A_GL_FRONT_FACE backface_culling_winding;
  NV2A_GL_CULL_FACE backface_culling_culled;
  bool depth_test_enabled;
  bool stencil_test_enabled;
  bool alpha_test_enabled;
  bool depth_write_enabled;
  bool blending_enabled;
  bool logical_operation_enabled;
  NV2A_COMPARISON_OP depth_function;
  NV2A_COMPARISON_OP stencil_func;
  NV2A_COMPARISON_OP alpha_func;
  int alpha_reference;
  int stencil_ref;
  int stencil_mask;
  NV2A_STENCIL_OP stencil_op_fail;
  NV2A_STENCIL_OP stencil_op_zfail;
  NV2A_STENCIL_OP stencil_op_zpass;
  NV2A_BLEND_EQUATION blend_equation;
  NV2A_BLEND_FACTOR blend_function_source;
  NV2A_BLEND_FACTOR blend_function_destination;
  Bit32u blend_color;
  NV2A_LOGIC_OP logical_operation;
  Bit32u fog_color;
  bool bilinear_filter;
  
  // Vertex transform state
  Bit32u vertex_count;
  Bit32u vertex_first;
  Bit32u primitives_count;
  Bit32u primitives_total_count;
  Bit32u primitives_batches_count;
  Bit32u triangles_bfculled;
  int indexesleft_count;
  int indexesleft_first;
  Bit32u vertex_indexes[1024*3]; // Triangle vertex indices
  vertex_nv vertex_software[1024+2]; // Vertex attributes before processing
  nv2avertex_t vertex_xy[1024+2]; // Vertex attributes after processing
  vertex_nv persistvertexattr; // Persistent vertex attributes
  NV2A_BEGIN_END primitive_type; // Current primitive type
  
  // Texture state
  struct {
    int enabled;
    int sizes;
    int sizet;
    int sizer;
    int dilate;
    NV2A_TEX_FORMAT format;
    bool rectangle;
    int rectangle_pitch;
    void *buffer;
    int dma0;
    int dma1;
    int mode;
    int cubic;
    int noborder;
    int dims;
    int mipmap;
    int colorkey;
    int imagefield;
    int aniso;
    int mipmapmaxlod;
    int mipmapminlod;
    int rectheight;
    int rectwidth;
    int addrmodes;
    int addrmodet;
    int addrmoder;
  } texture[NV20_TEXTURE_UNITS];
  
  // Viewport and render target state
  rectangle clippingwindows[8];
  rectangle limits_rendertarget;
  rectangle clear_rendertarget;
  Bit32u pitch_rendertarget;
  Bit32u pitch_depthbuffer;
  Bit32u size_rendertarget;
  Bit32u size_depthbuffer;
  int log2height_rendertarget;
  int log2width_rendertarget;
  int dilate_rendertarget;
  int antialiasing_rendertarget;
  NV2A_RT_TYPE type_rendertarget;
  NV2A_RT_DEPTH_FORMAT depthformat_rendertarget;
  NV2A_COLOR_FORMAT colorformat_rendertarget;
  int bytespixel_rendertarget;
  Bit32u antialias_control;
  float supersample_factor_x;
  float supersample_factor_y;
  
  // Framebuffer pointers
  Bit32u *rendertarget;
  Bit32u *depthbuffer;
  Bit32u *displayedtarget;
  Bit32u *old_rendertarget;
  
  // Dilation lookup tables
  Bit32u dilated0[16][2048];
  Bit32u dilated1[16][2048];
  int dilatechose[256];
  
  // BitBlit state
  struct {
    int format;
    Bit32u pitch_source;
    Bit32u pitch_destination;
    offs_t source_address;
    offs_t destination_address;
    int op;
    int width;
    int height;
    Bit32u sourcex;
    Bit32u sourcey;
    Bit32u destinationx;
    Bit32u destinationy;
  } bitblit;
  
  // Debug and miscellaneous
  int vertex_pipeline;
  int puller_waiting;
  int debug_grab_texttype;
  char *debug_grab_textfile;
  bool enable_waitvblank;
  bool enable_clipping_w;

  // Internal methods
  bool is_unlocked() { return svga_unlock_special; }
  BX_GEFORCE_SMF void svga_init_pcihandlers(void);
  void vertical_timer();
  BX_GEFORCE_SMF bool geforce_mem_read_handler(bx_phy_address addr, unsigned len, void *data, void *param);
  BX_GEFORCE_SMF bool geforce_mem_write_handler(bx_phy_address addr, unsigned len, void *data, void *param);
};

#endif // BX_SUPPORT_GEFORCE