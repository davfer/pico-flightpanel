#ifndef _COMMON_DVI_PIN_CONFIGS_H
#define _COMMON_DVI_PIN_CONFIGS_H

// This file defines the TMDS pair layouts on a handful of boards I have been
// developing on. It's not a particularly important file -- just saves some
// copy + paste.

#include "dvi_serialiser.h"

#define DEFAULT_DVI_SERIAL_CONFIG pico_sock_cfg
#ifndef DEFAULT_DVI_SERIAL_CONFIG
#define DEFAULT_DVI_SERIAL_CONFIG picodvi_dvi_cfg
#endif

// ----------------------------------------------------------------------------
// PicoDVI boards

// Legacy pin mapping for Rev A PicoDVI boards -- I think just Graham and I
// have these :)
static const struct dvi_serialiser_cfg picodvi_reva_dvi_cfg = {
	.pio = pio0,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {24, 26, 28},
	.pins_clk = 22,
	.invert_diffpairs = true
};

// The not-HDMI socket on Rev C PicoDVI boards
// (we don't talk about Rev B)
static const struct dvi_serialiser_cfg picodvi_dvi_cfg = {
	.pio = pio0,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {10, 12, 14},
	.pins_clk = 8,
	.invert_diffpairs = true
};

// You can jam an adapter board into either of the PMOD sockets on a PicoDVI!
static const struct dvi_serialiser_cfg picodvi_pmod0_cfg = {
	.pio = pio0,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {2, 4, 0},
	.pins_clk = 6,
	.invert_diffpairs = false
};

// ----------------------------------------------------------------------------
// Other boards

// The not-HDMI socket on SparkX HDMI carrier board with RP2040 MicroMod
// inserted.
static const struct dvi_serialiser_cfg micromod_cfg = {
	.pio = pio0,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {18, 20, 22},
	.pins_clk = 16,
	.invert_diffpairs = true
};

// Pico DVI Sock (small hat on the bottom) which solders to the end of a Pico
static const struct dvi_serialiser_cfg pico_sock_cfg = {
	.pio = pio0,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {12, 18, 16},
	.pins_clk = 14,
	.invert_diffpairs = false
};

#endif
