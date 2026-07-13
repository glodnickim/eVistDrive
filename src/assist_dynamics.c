#include "assist_dynamics.h"

#include "config.h"

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

#if IQ_RAMP_TIME_MODE
static int32_t iq_reference_q;
#endif

int32_t assist_dynamics_apply_legacy(
	int32_t iq_target,
	int32_t iq_reference,
	const assist_dynamics_input_t *input)
{
#if IQ_RAMP_TIME_MODE
	if (input->safety_cut) {
		iq_reference_q = iq_target << IQ_RAMP_Q_SHIFT;
		return iq_target;
	}

#if IQ_RAMP_ADAPTIVE
	int32_t up_s = map((int32_t)input->speed_x100,
		IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI,
		IQ_RAMP_UP_SLOW_TICKS, IQ_RAMP_UP_FAST_TICKS);
	int32_t up_c = map((int32_t)input->cadence_rpm,
		IQ_RAMP_CAD_LO, IQ_RAMP_CAD_HI,
		IQ_RAMP_UP_SLOW_TICKS, IQ_RAMP_UP_FAST_TICKS);
	int32_t dn_s = map((int32_t)input->speed_x100,
		IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI,
		IQ_RAMP_DOWN_SLOW_TICKS, IQ_RAMP_DOWN_FAST_TICKS);
	int32_t dn_c = map((int32_t)input->cadence_rpm,
		IQ_RAMP_CAD_LO, IQ_RAMP_CAD_HI,
		IQ_RAMP_DOWN_SLOW_TICKS, IQ_RAMP_DOWN_FAST_TICKS);
	int32_t up_ticks = (up_c < up_s) ? up_c : up_s;
	int32_t dn_ticks = (dn_c < dn_s) ? dn_c : dn_s;
#else
	int32_t up_ticks = IQ_RAMP_UP_SLOW_TICKS;
	int32_t dn_ticks = IQ_RAMP_DOWN_SLOW_TICKS;
#endif

	if (input->walk_active) {
		up_ticks = IQ_RAMP_UP_FAST_TICKS;
		dn_ticks = IQ_RAMP_DOWN_FAST_TICKS;
	}

	int32_t iq_scale = input->iq_scale;
	if (iq_scale < 1) iq_scale = input->phase_current_max;
	if (iq_scale < 1) iq_scale = PH_CURRENT_MAX;
	if (iq_scale < 1) iq_scale = 1;

	int32_t target_q = iq_target << IQ_RAMP_Q_SHIFT;
	int32_t ticks = (target_q > iq_reference_q) ? up_ticks : dn_ticks;
	if (ticks < 1) ticks = 1;
	int32_t step_q = ((iq_scale << IQ_RAMP_Q_SHIFT) + ticks - 1) / ticks;
	if (step_q < 1) step_q = 1;

	if (target_q > iq_reference_q) {
		int32_t d = target_q - iq_reference_q;
		iq_reference_q += (d > step_q) ? step_q : d;
	} else if (target_q < iq_reference_q) {
		int32_t d = iq_reference_q - target_q;
		iq_reference_q -= (d > step_q) ? step_q : d;
	}

	iq_reference = (iq_reference_q + (1 << (IQ_RAMP_Q_SHIFT - 1))) >> IQ_RAMP_Q_SHIFT;
	if (iq_target == 0 && iq_reference == 0) iq_reference_q = 0;
	return iq_reference;
#else
#if IQ_RAMP_ADAPTIVE
	int32_t up_s = map((int32_t)input->speed_x100,
		IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI, IQ_SLEW_UP_SLOW, IQ_SLEW_UP_FAST);
	int32_t up_c = map((int32_t)input->cadence_rpm,
		IQ_RAMP_CAD_LO, IQ_RAMP_CAD_HI, IQ_SLEW_UP_SLOW, IQ_SLEW_UP_FAST);
	int32_t up_step = (up_c > up_s) ? up_c : up_s;
	if (up_step < IQ_SLEW_UP_SLOW) up_step = IQ_SLEW_UP_SLOW;
	int32_t dn_step = map((int32_t)input->speed_x100,
		IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI, IQ_SLEW_DOWN_SLOW, IQ_SLEW_DOWN_FAST);
	if (dn_step < IQ_SLEW_DOWN_SLOW) dn_step = IQ_SLEW_DOWN_SLOW;
#else
	int32_t up_step = IQ_SLEW_UP;
	int32_t dn_step = IQ_SLEW_DOWN;
#endif

	if (input->safety_cut) return iq_target;
	if (iq_target > iq_reference) {
		int32_t d = iq_target - iq_reference;
		return iq_reference + ((d > up_step) ? up_step : d);
	}

	int32_t d = iq_reference - iq_target;
	return iq_reference - ((d > dn_step) ? dn_step : d);
#endif
}
