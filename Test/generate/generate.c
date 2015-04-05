/**
 * $Id: generate.c 1246 2014-06-02 09:07am pdorazio $
 *
 * @brief Red Pitaya simple signal/function generator with pre-defined
 *        signal types.
 *
 * @Author Ales Bardorfer <ales.bardorfer@redpitaya.com>
 *
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in C programming language.
 * Please visit http://en.wikipedia.org/wiki/C_(programming_language)
 * for more details on the language used herein.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "fpga_awg.h"
#include "version.h"

/**
 * GENERAL DESCRIPTION:
 *
 * The code below performs a function of a signal generator, which produces
 * a a signal of user-selectable pred-defined Signal shape
 * [Sine, Square, Triangle], Amplitude and Frequency on a selected Channel:
 *
 *
 *                   /-----\
 *   Signal shape -->|     | -->[data]--+-->[FPGA buf 1]--><DAC 1>
 *   Amplitude ----->| AWG |            |
 *   Frequency ----->|     |             -->[FPGA buf 2]--><DAC 2>
 *                   \-----/            ^
 *                                      |
 *   Channel ---------------------------+ 
 *
 *
 * This is achieved by first parsing the four parameters defining the 
 * signal properties from the command line, followed by synthesizing the 
 * signal in data[] buffer @ 125 MHz sample rate within the
 * generate_signal() function, depending on the Signal shape, Amplitude
 * and Frequency parameters. The data[] buffer is then transferred
 * to the specific FPGA buffer, defined by the Channel parameter -
 * within the write_signal_fpga() function.
 * The FPGA logic repeatably sends the data from both FPGA buffers to the
 * corresponding DACs @ 125 MHz, which in turn produces the synthesized
 * signal on Red Pitaya SMA output connectors labeled DAC1 & DAC2.
 *
 */

/** Maximal signal frequency [Hz] */
const double c_max_frequency = 62.5e6;

/** Minimal signal frequency [Hz] */
const double c_min_frequency = 0;

/** Maximal signal amplitude [Vpp] */
const double c_max_amplitude = 2.0;

/** AWG buffer length [samples]*/
#define n (16*1024)

/** Program name */
const char *g_argv0 = NULL;

/** Maximal Signal Voltage on DAC outputs on channel A. It is expressed in [V],
 * and calculated from apparent Back End Full Scale calibration parameter.
 */
float ch1_max_dac_v;

/** Maximal Signal Voltage on DAC outputs on channel B. It is expressed in [V],
 * and calculated from apparent Back End Full Scale calibration parameter.
 */
float ch2_max_dac_v;
/** @defgroup calib_h Calibration
 * @{
 */

/** Calibration parameters, stored in eeprom device
 */
typedef struct rp_osc_calib_params_s {
	uint32_t fe_ch1_fs_g_hi; /**< High gain front end full scale voltage, channel 1 */
	uint32_t fe_ch2_fs_g_hi; /**< High gain front end full scale voltage, channel 2 */
	uint32_t fe_ch1_fs_g_lo; /**< Low gain front end full scale voltage, channel 1  */
	uint32_t fe_ch2_fs_g_lo; /**< Low gain front end full scale voltage, channel 2  */
	int32_t fe_ch1_dc_offs; /**< Front end DC offset, channel 1  */
	int32_t fe_ch2_dc_offs; /**< Front end DC offset, channel 2  */
	uint32_t be_ch1_fs; /**< Back end full scale voltage, channel 1  */
	uint32_t be_ch2_fs; /**< Back end full scale voltage, channel 2  */
	int32_t be_ch1_dc_offs; /**< Back end DC offset, channel 1 */
	int32_t be_ch2_dc_offs; /**< Back end DC offset, on channel 2 */
} rp_calib_params_t;

/** @} */

int rp_read_calib_params(rp_calib_params_t *calib_params);

int rp_default_calib_params(rp_calib_params_t *calib_params);

/** Signal types */
typedef enum {
	eSignalSine,         ///< Sinusoidal waveform.
	eSignalSquare,       ///< Square waveform.
	eSignalTriangle,     ///< Triangular waveform.
	eSignalSweep         ///< Sinusoidal frequency sweep.
} signal_e;

/** AWG FPGA parameters */
typedef struct {
	int32_t offsgain;   ///< AWG offset & gain.
	uint32_t wrap;       ///< AWG buffer wrap value.
	uint32_t step;       ///< AWG step interval.
} awg_param_t;

/* Forward declarations */
void synthesize_signal(double ampl, double freq, signal_e type, int32_t *data,
		awg_param_t *params);
void write_data_fpga(uint32_t ch, const int32_t *data, const awg_param_t *awg);
void make_AM_signal();
void rp_app_init(void);
int generate_init(rp_calib_params_t *calib_params);
int test();
void generate(int ch, int start);
rp_calib_params_t rp_main_calib_params;
rp_calib_params_t *gen_calib_params = NULL;
void doShape(int ch, int start, awg_param_t awg);
/** Print usage information */
void usage() {

	const char *format =
			"%s version %s-%s\n"
					"\n"
					"Usage: %s   channel amplitude frequency <type> <end frequency>\n"
					"\n"
					"\tchannel     Channel to generate signal on [1, 2].\n"
					"\tamplitude   Peak-to-peak signal amplitude in Vpp [0.0 - %1.1f].\n"
					"\tfrequency   Signal frequency in Hz [%2.1f - %2.1e].\n"
					"\ttype        Signal type [sine, sqr, tri, sweep].\n"
					"\tend frequency   Sweep-to frequency in Hz [%2.1f - %2.1e].\n"
					"\n";

	fprintf(stderr, format, g_argv0, VERSION_STR, REVISION_STR, g_argv0,
			c_max_amplitude, c_min_frequency, c_max_frequency);
}

/** Signal generator main */
int main(int argc, char *argv[]) {
	generate(0, 0);
	generate(1, 0);
	int offset = 0;
	while (1 > 0) {
		int i = getchar();
		if (i == 0 || i == '0') {
			return 0;
		}
		if (i == 1 || i == '1') {
			offset += 1000;
			generate(0, offset);
		}
	}
	//generate(1, 0);
//	make_AM_signal();
	/* Signal amplitude argument parsing */
//	double ampl = 2.0;
	/* Signal frequency argument parsing */
//	double freq = 10000000;
//	signal_e type = eSignalSine;
//	awg_param_t params;
	/** AWG data buffer */
//	int32_t data[n];
	/* Prepare data buffer (calculate from input arguments) */
//	synthesize_signal(ampl, freq, type, data, &params);
	/* Write the data to the FPGA and set FPGA AWG state machine */
//	write_data_fpga(1, data, &params);
}

/**
 * Synthesize a desired signal.
 *
 * Generates/synthesized  a signal, based on three pre-defined signal
 * types/shapes, signal amplitude & frequency. The data[] vector of 
 * samples at 125 MHz is generated to be re-played by the FPGA AWG module.
 *
 * @param ampl  Signal amplitude [Vpp].
 * @param freq  Signal frequency [Hz].
 * @param type  Signal type/shape [Sine, Square, Triangle].
 * @param data  Returned synthesized AWG data vector.
 * @param awg   Returned AWG parameters.
 *
 */
void synthesize_signal(double ampl, double freq, signal_e type, int32_t *data,
		awg_param_t *awg) {

	uint32_t i;

	/* Various locally used constants - HW specific parameters */
	const int dcoffs = -155;

	/* This is where frequency is used... */
	awg->offsgain = (dcoffs << 16) + 0x1fff;
	awg->step = round(65536 * freq / c_awg_smpl_freq * n);
	awg->wrap = round(65536 * (n - 1));
	//fprintf(stderr, "check: %d, %d, %d", awg->offsgain, awg->step, awg->wrap);

	uint32_t amp = ampl * 4000.0; /* 1 Vpp ==> 4000 DAC counts */
	if (amp > 8191) {
		/* Truncate to max value if needed */
		amp = 8191;
	}
	/* Fill data[] with appropriate buffer samples */
	for (i = 0; i < n; i++) {
		data[i] = round(amp * cos(2 * M_PI * (double) i / (double) n));
	}
}

/**
 * Write synthesized data[] to FPGA buffer.
 *
 * @param ch    Channel number [0, 1].
 * @param data  AWG data to write to FPGA.
 * @param awg   AWG paramters to write to FPGA.
 */
void write_data_fpga(uint32_t ch, const int32_t *data, const awg_param_t *awg) {

	uint32_t i;

	fpga_awg_init();

	if (ch == 0) {
		/* Channel A */
		g_awg_reg->state_machine_conf = 0x000041;
		g_awg_reg->cha_scale_off = awg->offsgain;
		g_awg_reg->cha_count_wrap = awg->wrap;
		g_awg_reg->cha_count_step = awg->step;
		g_awg_reg->cha_start_off = 0;

		for (i = 0; i < n; i++) {
			g_awg_cha_mem[i] = data[i];
		}
	} else {
		/* Channel B */
		g_awg_reg->state_machine_conf = 0x410000;
		g_awg_reg->chb_scale_off = awg->offsgain;
		g_awg_reg->chb_count_wrap = awg->wrap;
		g_awg_reg->chb_count_step = awg->step;
		g_awg_reg->chb_start_off = 0;

		for (i = 0; i < n; i++) {
			g_awg_chb_mem[i] = data[i];
		}
	}

	/* Enable both channels */
	/* TODO: Should this only happen for the specified channel?
	 *       Otherwise, the not-to-be-affected channel is restarted as well
	 *       causing unwanted disturbances on that channel.
	 */
	g_awg_reg->state_machine_conf = 0x110011;

	fpga_awg_exit();
}

void make_AM_signal() {
	double ampl = 2.0;
	double freq = 10000000;
	int32_t data[n];
	awg_param_t params;

	uint32_t i;

	/* Various locally used constants - HW specific parameters */
	const int dcoffs = -155;

	/* This is where frequency is used... */
	params.offsgain = (dcoffs << 16) + 0x1fff;
	params.step = round(65536 * freq / c_awg_smpl_freq * n);
	params.wrap = round(65536 * (n - 1));
	//fprintf(stderr, "check: %d, %d, %d", awg->offsgain, awg->step, awg->wrap);

	uint32_t amp = ampl * 4000.0; /* 1 Vpp ==> 4000 DAC counts */
	if (amp > 8191) {
		/* Truncate to max value if needed */
		amp = 8191;
	}
	/* Fill data[] with appropriate buffer samples */
	//for (i = 0; i < n; i++) {
	//	data[i] = round(amp * cos(2 * M_PI * (double) i / (double) n));
	//}
	const int trans = 30;
	const float tt2 = 0.249;

	for (i = 0; i < n; i++) {
		data[i] = round(amp * cos(2 * M_PI * (float) i / (float) AWG_SIG_LEN));
		data[i] = (data[i] > 0) ? amp : -amp;

		/* Soft linear transitions */
		float mm, qq, xx, xm;
		float x1, x2, y1, y2;

		xx = i;
		xm = AWG_SIG_LEN;
		mm = -2.0 * (float) amp / (float) trans;
		qq = (float) amp * (2 + xm / (2.0 * (float) trans));

		x1 = xm * tt2;
		x2 = xm * tt2 + (float) trans;

		if ((xx > x1) && (xx <= x2)) {

			y1 = (float) amp;
			y2 = -(float) amp;

			mm = (y2 - y1) / (x2 - x1);
			qq = y1 - mm * x1;

			data[i] = round(mm * xx + qq);
		}

		x1 = xm * 0.75;
		x2 = xm * 0.75 + trans;

		if ((xx > x1) && (xx <= x2)) {

			y1 = -(float) amp;
			data[i] = round(
					amp * cos(2 * M_PI * (float) i / (float) AWG_SIG_LEN));
			data[i] = (data[i] > 0) ? amp : -amp;

			/* Soft linear transitions */
			float mm, qq, xx, xm;
			float x1, x2, y1, y2;

			xx = i;
			xm = AWG_SIG_LEN;
			mm = -2.0 * (float) amp / (float) trans;
			qq = (float) amp * (2 + xm / (2.0 * (float) trans));

			x1 = xm * tt2;
			x2 = xm * tt2 + (float) trans;

			if ((xx > x1) && (xx <= x2)) {

				y1 = (float) amp;
				y2 = -(float) amp;

				mm = (y2 - y1) / (x2 - x1);
				qq = y1 - mm * x1;

				data[i] = round(mm * xx + qq);
			}

			x1 = xm * 0.75;
			x2 = xm * 0.75 + trans;

			if ((xx > x1) && (xx <= x2)) {

				y1 = -(float) amp;
				y2 = (float) amp;

				mm = (y2 - y1) / (x2 - x1);
				qq = y1 - mm * x1;

				data[i] = round(mm * xx + qq);
			}
			y2 = (float) amp;

			mm = (y2 - y1) / (x2 - x1);
			qq = y1 - mm * x1;

			data[i] = round(mm * xx + qq);
		}
	}
	write_data_fpga(0, data, &params);
}

void rp_app_init(void) {
	fprintf(stderr, "Loading scope (with gen+pid extensions) version %s-%s.\n",
	VERSION_STR, REVISION_STR);

	rp_default_calib_params(&rp_main_calib_params);
	if (rp_read_calib_params(&rp_main_calib_params) < 0) {
		fprintf(stderr, "rp_read_calib_params() failed, using default"
				" parameters\n");
	}
	generate_init(&rp_main_calib_params);
}

int generate_init(rp_calib_params_t *calib_params) {
	gen_calib_params = calib_params;

	if (fpga_awg_init() < 0) {
		return -1;
	}

	ch1_max_dac_v = fpga_awg_calc_dac_max_v(gen_calib_params->be_ch1_fs);
	ch2_max_dac_v = fpga_awg_calc_dac_max_v(gen_calib_params->be_ch2_fs);
	return 0;
}

const char eeprom_device[] = "/sys/bus/i2c/devices/0-0050/eeprom";
const int eeprom_calib_off = 0x0008;

/*----------------------------------------------------------------------------*/
/**
 * @brief Read calibration parameters from EEPROM device.
 *
 * Function reads calibration parameters from EEPROM device and stores them to the
 * specified buffer. Communication to the EEPROM device is taken place through
 * appropriate system driver accessed through the file system device
 * /sys/bus/i2c/devices/0-0050/eeprom.
 *
 * @param[out]   calib_params  Pointer to destination buffer.
 * @retval       0 Success
 * @retval      -1 Failure, error message is put on stderr device
 *
 */
int rp_read_calib_params(rp_calib_params_t *calib_params) {
	FILE *fp;
	size_t size;

	/* sanity check */
	if (calib_params == NULL) {
		fprintf(stderr, "rp_read_calib_params(): input structure "
				"not initialized\n");
		return -1;
	}

	/* open eeprom device */
	fp = fopen(eeprom_device, "r");
	if (fp == NULL) {
		fprintf(stderr, "rp_read_calib_params(): Can not open EEPROM device: "
				" %s\n", "strerror(errno)");
		return -1;
	}

	/* ...and seek to the appropriate storage offset */
	if (fseek(fp, eeprom_calib_off, SEEK_SET) < 0) {
		fclose(fp);
		fprintf(stderr, "rp_read_calib_params(): fseek() failed: %s\n",
				"strerror(errno)");
		return -1;
	}

	/* read data from eeprom component and store it to the specified buffer */
	size = fread(calib_params, sizeof(char), sizeof(rp_calib_params_t), fp);
	if (size != sizeof(rp_calib_params_t)) {
		fclose(fp);
		fprintf(stderr, "rp_read_calib_params(): fread() failed, "
				"returned bytes: %d (should be :%d)\n", size,
				sizeof(rp_calib_params_t));
		return -1;
	}
	fclose(fp);

	return 0;
}

/*----------------------------------------------------------------------------*/
/**
 * Initialize calibration parameters to default values.
 *
 * @param[out] calib_params  Pointer to target buffer to be initialized.
 * @retval 0 Success, could never fail.
 */

int rp_default_calib_params(rp_calib_params_t *calib_params) {
	calib_params->fe_ch1_fs_g_hi = 28101971; /* 0.6543 [V] */
	calib_params->fe_ch2_fs_g_hi = 28101971; /* 0.6543 [V] */
	calib_params->fe_ch1_fs_g_lo = 625682246; /* 14.56 [V] */
	calib_params->fe_ch2_fs_g_lo = 625682246; /* 14.56 [V] */
	calib_params->fe_ch1_dc_offs = 585;
	calib_params->fe_ch2_dc_offs = 585;
	calib_params->be_ch1_fs = 42949673; /* 1 [V] */
	calib_params->be_ch2_fs = 42949673; /* 1 [V] */
	calib_params->be_ch1_dc_offs = 0x3eac;
	calib_params->be_ch2_dc_offs = 0x3eac;

	return 0;
}

int test() {

	//float ampl, , int calib_dc_offs, int calib_fs, awg_signal_t type,  int32_t *data,

	rp_app_init();

	const int c_awg_fpga_dac_bits = 14;
	float freq = 10000000;
	float user_dc_offs = 0;
	int calib_dc_offs = gen_calib_params->be_ch1_dc_offs;
	awg_param_t awg;
	float ampl = 2;
	int calib_fs = gen_calib_params->be_ch1_fs;
	signal_e type = eSignalSquare;
	int32_t data[AWG_SIG_LEN];

	float max_dac_v = ch1_max_dac_v;

	uint32_t i;

	/* Various locally used constants - HW specific parameters */
	const int trans0 = 30;
	const int trans1 = 300;
	const float tt2 = 0.249;
	const int c_dac_max = (1 << (c_awg_fpga_dac_bits - 1)) - 1;
	const int c_dac_min = -(1 << (c_awg_fpga_dac_bits - 1));

	int trans = round(freq / 1e6 * ((float) trans1)); /* 300 samples at 1 MHz */
	int user_dc_off_cnt = round(
			(1 << (c_awg_fpga_dac_bits - 1)) * user_dc_offs / max_dac_v);
	uint32_t amp;

	/* Saturate offset - depending on calibration offset, it could overflow */
	int offsgain = calib_dc_offs + user_dc_off_cnt;
	offsgain = (offsgain > c_dac_max) ? c_dac_max : offsgain;
	offsgain = (offsgain < c_dac_min) ? c_dac_min : offsgain;

	awg.offsgain = (offsgain << 16) | 0x2000;
	awg.step = round(65536.0 * freq / c_awg_smpl_freq * ((float) AWG_SIG_LEN));
	awg.wrap = round(65536 * (AWG_SIG_LEN - 1));

	//= (ampl) * (1<<(c_awg_fpga_dac_bits-2));
	//fpga_awg_calc_dac_max_v(calib_fs)

	amp = round(ampl / 2 / fpga_awg_calc_dac_max_v(calib_fs) * c_dac_max);

	/* Truncate to max value */
	amp = (amp > c_dac_max) ? c_dac_max : amp;

	if (trans <= 10) {
		trans = trans0;
	}

	/* Fill data[] with appropriate buffer samples */
	for (i = 0; i < AWG_SIG_LEN; i++) {
		/* Sine */
		if (type == eSignalSine) {
			data[i] = round(
					amp * cos(2 * M_PI * (float) i / (float) AWG_SIG_LEN));
		}

		/* Square */
		if (type == eSignalSquare) {
			data[i] = round(
					amp * cos(2 * M_PI * (float) i / (float) AWG_SIG_LEN));
			data[i] = (data[i] > 0) ? amp : -amp;

			/* Soft linear transitions */
			float mm, qq, xx, xm;
			float x1, x2, y1, y2;

			xx = i;
			xm = AWG_SIG_LEN;
			mm = -2.0 * (float) amp / (float) trans;
			qq = (float) amp * (2 + xm / (2.0 * (float) trans));

			x1 = xm * tt2;
			x2 = xm * tt2 + (float) trans;

			if ((xx > x1) && (xx <= x2)) {

				y1 = (float) amp;
				y2 = -(float) amp;

				mm = (y2 - y1) / (x2 - x1);
				qq = y1 - mm * x1;

				data[i] = round(mm * xx + qq);
			}

			x1 = xm * 0.75;
			x2 = xm * 0.75 + trans;

			if ((xx > x1) && (xx <= x2)) {

				y1 = -(float) amp;
				y2 = (float) amp;

				mm = (y2 - y1) / (x2 - x1);
				qq = y1 - mm * x1;

				data[i] = round(mm * xx + qq);
			}
		}

		/* Triangle */
		if (type == eSignalTriangle) {
			data[i] =
					round(
							-1.0 * (float) amp
									* (acos(
											cos(
													2 * M_PI
															* (float) i/(float)AWG_SIG_LEN))
											/ M_PI * 2 - 1));
		}
	}
	write_data_fpga(0, data, &awg);
	return 0;
}

awg_param_t awg;
int time = 0;
void generate(int ch, int start) {
	if (time == 0) {
		time++;
		rp_app_init();
		const int c_awg_fpga_dac_bits = 14;
		float freq = 100000;
		float user_dc_offs = 0;
		int calib_dc_offs;
		int calib_fs;
		float max_dac_v;
		if (ch == 0) {
			calib_dc_offs = gen_calib_params->be_ch1_dc_offs;
			calib_fs = gen_calib_params->be_ch1_fs;
			max_dac_v = ch1_max_dac_v;
		} else {
			calib_dc_offs = gen_calib_params->be_ch2_dc_offs;
			calib_fs = gen_calib_params->be_ch2_fs;
			max_dac_v = ch2_max_dac_v;
		}
		float ampl = 2;

		/* Various locally used constants - HW specific parameters */
		const int trans0 = 30;
		const int trans1 = 300;
		const int c_dac_max = (1 << (c_awg_fpga_dac_bits - 1)) - 1;
		const int c_dac_min = -(1 << (c_awg_fpga_dac_bits - 1));

		int trans = round(freq / 1e6 * ((float) trans1)); /* 300 samples at 1 MHz */
		int user_dc_off_cnt = round(
				(1 << (c_awg_fpga_dac_bits - 1)) * user_dc_offs / max_dac_v);
		uint32_t amp;

		/* Saturate offset - depending on calibration offset, it could overflow */
		int offsgain = calib_dc_offs + user_dc_off_cnt;
		offsgain = (offsgain > c_dac_max) ? c_dac_max : offsgain;
		offsgain = (offsgain < c_dac_min) ? c_dac_min : offsgain;

		awg.offsgain = (offsgain << 16) | 0x2000;
		awg.step = round(
				65536.0 * freq / c_awg_smpl_freq * ((float) AWG_SIG_LEN));
		awg.wrap = round(65536 * (AWG_SIG_LEN - 1));

		amp = round(ampl / 2 / fpga_awg_calc_dac_max_v(calib_fs) * c_dac_max);

		/* Truncate to max value */
		amp = (amp > c_dac_max) ? c_dac_max : amp;

		if (trans <= 10) {
			trans = trans0;
		}
	}
	doShape(ch, start, awg);
}

void doShape(int ch, int start, awg_param_t awg) {
	/* Fill data[] with appropriate buffer samples */
	uint32_t i;
	int32_t data[AWG_SIG_LEN];
	for (i = 0; i < AWG_SIG_LEN; i++) {
		data[i] = (i > 4000 + start) ? -8000 : 8000;
	}
	write_data_fpga(ch, data, &awg);
}
