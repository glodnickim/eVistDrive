/* Assist Behavior V3 rider/bike stimulus generator - see rider_script.h. TEST CODE ONLY. */
#include "rider_script.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RS_STEP_DEG (360.0 / (double)RIDER_SCRIPT_TRANSITIONS_PER_REV)

static const char *const LABEL_NAMES[] = {
	"NORMAL", "PHASE_DIP", "TRUE_RELEASE", "ATTACK", "PEDAL_STOP", "REVERSE", "BRAKE", "COAST"
};
static const char *const KIND_NAMES[] = {
	"steady", "ramp", "release", "attack", "stop", "coast", "reverse", "brake"
};
static const char *const EVENT_NAMES[] = { "bounce", "drop", "jitter", "illegal", "stall" };

const char *rider_script_label_name(rs_label_t l)
{
	return ((unsigned)l < sizeof(LABEL_NAMES) / sizeof(LABEL_NAMES[0])) ? LABEL_NAMES[l] : "?";
}

const char *rider_script_kind_name(rs_seg_kind_t k)
{
	return ((unsigned)k < sizeof(KIND_NAMES) / sizeof(KIND_NAMES[0])) ? KIND_NAMES[k] : "?";
}

/* ---------------------------------------------------------------- shape (crank_model.c law) */

/* Identical math to crank_torque_raw_mv() in crank_model.c, expressed as a dimensionless
 * multiplier of the mean instead of native mV: rectified half-sine per leg, optional raised-
 * cosine dead-spot notch, L/R asymmetry, ripple centred on the mean, floor at zero. */
static double shape_raw(const rs_shape_t *sh, double angle_deg)
{
	double angle = fmod(angle_deg - sh->phase_shift_deg, 360.0);
	if (angle < 0.0) angle += 360.0;
	double leg_progress = fmod(angle, 180.0) / 180.0;
	double envelope = sin(M_PI * leg_progress);
	double dist = (leg_progress <= 0.5) ? leg_progress * 180.0 : 180.0 - leg_progress * 180.0;
	if (sh->dead_spot_width_deg > 0.0 && dist < sh->dead_spot_width_deg) {
		double notch = 0.5 * (1.0 + cos(M_PI * dist / sh->dead_spot_width_deg));
		envelope *= (1.0 - (sh->dead_spot_depth_pct / 100.0) * notch);
	}
	int leg_b = (angle >= 180.0);
	double asym = leg_b ? (1.0 - sh->asymmetry_pct / 200.0) : (1.0 + sh->asymmetry_pct / 200.0);
	double v = 1.0 + (sh->ripple_pct / 100.0) * asym * (envelope - 0.5) * 2.0;
	return v < 0.0 ? 0.0 : v;
}

static void compute_shape_norm(rider_script_t *sc)
{
	/* 0.1 degree midpoint rule; the shape is smooth except at the floor, so this is exact to
	 * well below the noise of anything measured downstream. */
	double sum = 0.0;
	const int n = 3600;
	for (int i = 0; i < n; i++) sum += shape_raw(&sc->shape, (i + 0.5) * 360.0 / n);
	double mean = sum / n;
	sc->shape_norm = mean > 1e-9 ? 1.0 / mean : 1.0;
}

double rider_script_shape(const rider_script_t *sc, double angle_deg)
{
	return shape_raw(&sc->shape, angle_deg) * sc->shape_norm;
}

/* ---------------------------------------------------------------- parsing */

static void set_err(char *err, size_t n, int line, const char *msg, const char *tok)
{
	if (err && n) snprintf(err, n, "line %d: %s%s%s", line, msg, tok ? ": " : "", tok ? tok : "");
}

static int parse_range(const char *v, double *a, double *b)
{
	char *end;
	*a = strtod(v, &end);
	if (end == v) return -1;
	if (*end == ':') {
		const char *s = end + 1;
		*b = strtod(s, &end);
		if (end == s) return -1;
	} else {
		*b = *a;
	}
	return *end == '\0' ? 0 : -1;
}

static int parse_num(const char *v, double *out)
{
	char *end;
	*out = strtod(v, &end);
	return (end != v && *end == '\0') ? 0 : -1;
}

#define RS_MAX_TOK 16

static int tokenize(char *line, char **tok)
{
	int n = 0;
	char *p = line;
	while (*p && n < RS_MAX_TOK) {
		while (*p && isspace((unsigned char)*p)) p++;
		if (!*p) break;
		tok[n++] = p;
		while (*p && !isspace((unsigned char)*p)) p++;
		if (*p) *p++ = '\0';
	}
	return n;
}

static int kind_from_name(const char *s)
{
	for (unsigned i = 0; i < sizeof(KIND_NAMES) / sizeof(KIND_NAMES[0]); i++)
		if (strcmp(s, KIND_NAMES[i]) == 0) return (int)i;
	return -1;
}

static int event_from_name(const char *s)
{
	for (unsigned i = 0; i < sizeof(EVENT_NAMES) / sizeof(EVENT_NAMES[0]); i++)
		if (strcmp(s, EVENT_NAMES[i]) == 0) return (int)i;
	return -1;
}

static int parse_line(rider_script_t *sc, char *line, int ln, char *err, size_t errlen,
                      double *prev_rpm, double *prev_mean, double *prev_speed)
{
	char *hash = strchr(line, '#');
	if (hash) *hash = '\0';
	char *tok[RS_MAX_TOK];
	int n = tokenize(line, tok);
	if (n == 0) return 0;
	const char *cmd = tok[0];
	double d;

	if (strcmp(cmd, "name") == 0 && n == 2) {
		snprintf(sc->name, sizeof(sc->name), "%s", tok[1]);
	} else if (strcmp(cmd, "seed") == 0 && n == 2) {
		sc->seed = (uint32_t)strtoul(tok[1], NULL, 0);
	} else if (strcmp(cmd, "noise") == 0 && n == 2 && parse_num(tok[1], &d) == 0 && d >= 0.0) {
		sc->noise_ckg = d;
	} else if (strcmp(cmd, "cadence_ripple") == 0 && n == 2 && parse_num(tok[1], &d) == 0 &&
	           d >= 0.0 && d < 0.9) {
		sc->cadence_ripple = d;
	} else if (strcmp(cmd, "start_angle") == 0 && n == 2 && parse_num(tok[1], &d) == 0) {
		sc->start_angle_deg = d;
	} else if (strcmp(cmd, "level") == 0 && n == 2 && parse_num(tok[1], &d) == 0 &&
	           d >= 0.0 && d <= 5.0) {
		sc->level = (uint8_t)d;
	} else if (strcmp(cmd, "phase_current_max") == 0 && n == 2 && parse_num(tok[1], &d) == 0 &&
	           d >= 0.0 && d <= 4000.0) {
		sc->phase_current_max = (uint16_t)d;
	} else if (strcmp(cmd, "legal") == 0 && n == 2 && parse_num(tok[1], &d) == 0) {
		sc->legal = d != 0.0;
	} else if (strcmp(cmd, "decimate") == 0 && n == 2 && parse_num(tok[1], &d) == 0 &&
	           d >= 1.0 && d <= 4000.0) {
		sc->decimate = (uint32_t)d;
	} else if (strcmp(cmd, "dip_threshold") == 0 && n == 2 && parse_num(tok[1], &d) == 0 &&
	           d > 0.0 && d < 1.0) {
		sc->dip_threshold = d;
	} else if (strcmp(cmd, "shape") == 0) {
		for (int i = 1; i < n; i++) {
			char *eq = strchr(tok[i], '=');
			if (!eq) { set_err(err, errlen, ln, "bad shape token", tok[i]); return -1; }
			*eq = '\0';
			if (parse_num(eq + 1, &d) != 0) { set_err(err, errlen, ln, "bad number", eq + 1); return -1; }
			if (strcmp(tok[i], "ripple") == 0) sc->shape.ripple_pct = d;
			else if (strcmp(tok[i], "asym") == 0) sc->shape.asymmetry_pct = d;
			else if (strcmp(tok[i], "dsdepth") == 0) sc->shape.dead_spot_depth_pct = d;
			else if (strcmp(tok[i], "dswidth") == 0) sc->shape.dead_spot_width_deg = d;
			else if (strcmp(tok[i], "phase") == 0) sc->shape.phase_shift_deg = d;
			else { set_err(err, errlen, ln, "unknown shape key", tok[i]); return -1; }
		}
	} else if (strcmp(cmd, "seg") == 0 && n >= 2) {
		if (sc->nseg >= RIDER_SCRIPT_MAX_SEG) { set_err(err, errlen, ln, "too many segments", NULL); return -1; }
		int k = kind_from_name(tok[1]);
		if (k < 0) { set_err(err, errlen, ln, "unknown segment kind", tok[1]); return -1; }
		rs_segment_t *s = &sc->seg[sc->nseg];
		memset(s, 0, sizeof(*s));
		s->kind = (rs_seg_kind_t)k;
		bool zero_default = (s->kind == RS_SEG_STOP || s->kind == RS_SEG_COAST);
		s->rpm0 = s->rpm1 = zero_default ? 0.0 : *prev_rpm;
		s->mean0 = s->mean1 = zero_default ? 0.0 : *prev_mean;
		s->speed0 = s->speed1 = *prev_speed;
		s->dur_s = -1.0;
		for (int i = 2; i < n; i++) {
			char *eq = strchr(tok[i], '=');
			if (!eq) { set_err(err, errlen, ln, "bad segment token", tok[i]); return -1; }
			*eq = '\0';
			const char *v = eq + 1;
			int rc;
			if (strcmp(tok[i], "dur") == 0) rc = parse_num(v, &s->dur_s);
			else if (strcmp(tok[i], "rpm") == 0) rc = parse_range(v, &s->rpm0, &s->rpm1);
			else if (strcmp(tok[i], "mean") == 0) rc = parse_range(v, &s->mean0, &s->mean1);
			else if (strcmp(tok[i], "speed") == 0) rc = parse_range(v, &s->speed0, &s->speed1);
			else { set_err(err, errlen, ln, "unknown segment key", tok[i]); return -1; }
			if (rc != 0) { set_err(err, errlen, ln, "bad value", v); return -1; }
		}
		if (s->dur_s <= 0.0) { set_err(err, errlen, ln, "segment needs dur>0", NULL); return -1; }
		if (s->mean0 < 0.0 || s->mean1 < 0.0 || s->speed0 < 0.0 || s->speed1 < 0.0 ||
		    fabs(s->rpm0) > 250.0 || fabs(s->rpm1) > 250.0) {
			set_err(err, errlen, ln, "segment value out of range", NULL);
			return -1;
		}
		*prev_rpm = s->rpm1;
		*prev_mean = s->mean1;
		*prev_speed = s->speed1;
		sc->nseg++;
	} else if (strcmp(cmd, "event") == 0 && n >= 2) {
		if (sc->nev >= RIDER_SCRIPT_MAX_EV) { set_err(err, errlen, ln, "too many events", NULL); return -1; }
		int t = event_from_name(tok[1]);
		if (t < 0) { set_err(err, errlen, ln, "unknown event type", tok[1]); return -1; }
		rs_event_t *e = &sc->ev[sc->nev];
		memset(e, 0, sizeof(*e));
		e->type = (rs_event_type_t)t;
		e->count = 1U;
		e->ticks = 1U;
		e->t_s = -1.0;
		for (int i = 2; i < n; i++) {
			char *eq = strchr(tok[i], '=');
			if (!eq) { set_err(err, errlen, ln, "bad event token", tok[i]); return -1; }
			*eq = '\0';
			if (parse_num(eq + 1, &d) != 0 || d < 0.0) {
				set_err(err, errlen, ln, "bad value", eq + 1);
				return -1;
			}
			if (strcmp(tok[i], "t") == 0) e->t_s = d;
			else if (strcmp(tok[i], "count") == 0) e->count = (uint32_t)d;
			else if (strcmp(tok[i], "period") == 0) e->period_s = d;
			else if (strcmp(tok[i], "ticks") == 0) e->ticks = (uint32_t)d;
			else if (strcmp(tok[i], "dur") == 0) e->dur_s = d;
			else { set_err(err, errlen, ln, "unknown event key", tok[i]); return -1; }
		}
		if (e->t_s < 0.0) { set_err(err, errlen, ln, "event needs t", NULL); return -1; }
		if (e->type == RS_EV_STALL && (e->ticks < 1U || e->ticks > 64U)) {
			set_err(err, errlen, ln, "stall ticks must be 1..64", NULL);
			return -1;
		}
		sc->nev++;
	} else {
		set_err(err, errlen, ln, "unknown or malformed command", cmd);
		return -1;
	}
	return 0;
}

int rider_script_parse_text(const char *text, rider_script_t *out, char *err, size_t errlen)
{
	memset(out, 0, sizeof(*out));
	snprintf(out->name, sizeof(out->name), "unnamed");
	out->seed = 1U;
	out->level = 3U;
	out->legal = true;
	out->decimate = 4U;
	out->dip_threshold = 0.5;
	out->start_angle_deg = 60.0;
	double prev_rpm = 0.0, prev_mean = 0.0, prev_speed = 0.0;
	int ln = 0;
	const char *p = text;
	char buf[512];
	while (*p) {
		const char *nl = strchr(p, '\n');
		size_t len = nl ? (size_t)(nl - p) : strlen(p);
		ln++;
		if (len >= sizeof(buf)) { set_err(err, errlen, ln, "line too long", NULL); return -1; }
		memcpy(buf, p, len);
		buf[len] = '\0';
		if (len && buf[len - 1] == '\r') buf[len - 1] = '\0';
		if (parse_line(out, buf, ln, err, errlen, &prev_rpm, &prev_mean, &prev_speed) != 0) return -1;
		p += len;
		if (*p == '\n') p++;
	}
	if (out->nseg == 0U) { set_err(err, errlen, ln, "script has no segments", NULL); return -1; }
	if (out->seed == 0U) out->seed = 1U;
	compute_shape_norm(out);
	return 0;
}

int rider_script_parse_file(const char *path, rider_script_t *out, char *err, size_t errlen)
{
	FILE *f = fopen(path, "rb");
	if (!f) { if (err && errlen) snprintf(err, errlen, "cannot open %s", path); return -1; }
	if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return -1; }
	long n = ftell(f);
	if (n < 0 || n > 1L << 20) { fclose(f); if (err && errlen) snprintf(err, errlen, "bad size"); return -1; }
	rewind(f);
	char *buf = (char *)malloc((size_t)n + 1U);
	if (!buf) { fclose(f); return -1; }
	size_t got = fread(buf, 1, (size_t)n, f);
	fclose(f);
	buf[got] = '\0';
	int rc = rider_script_parse_text(buf, out, err, errlen);
	free(buf);
	return rc;
}

/* ---------------------------------------------------------------- runtime */

static uint32_t sec_to_tick(double s)
{
	return (uint32_t)llround(s * RIDER_SCRIPT_TICK_HZ);
}

static uint32_t seg_end_tick(const rider_script_t *sc, uint32_t idx)
{
	double t = 0.0;
	for (uint32_t i = 0; i <= idx && i < sc->nseg; i++) t += sc->seg[i].dur_s;
	return sec_to_tick(t);
}

double rider_script_duration_s(const rider_script_t *sc)
{
	double t = 0.0;
	for (uint32_t i = 0; i < sc->nseg; i++) t += sc->seg[i].dur_s;
	return t;
}

uint32_t rider_script_total_ticks(const rider_script_t *sc)
{
	return sec_to_tick(rider_script_duration_s(sc));
}

static uint32_t xorshift(uint32_t *s)
{
	uint32_t x = *s;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*s = x;
	return x;
}

static double gauss(rider_script_state_t *st)
{
	if (st->have_spare) { st->have_spare = false; return st->spare; }
	double u1, u2;
	do { u1 = (double)(xorshift(&st->noise_state) >> 8) / 16777216.0; } while (u1 <= 1e-12);
	u2 = (double)(xorshift(&st->noise_state) >> 8) / 16777216.0;
	double r = sqrt(-2.0 * log(u1));
	st->spare = r * sin(2.0 * M_PI * u2);
	st->have_spare = true;
	return r * cos(2.0 * M_PI * u2);
}

void rider_script_start(rider_script_state_t *st, const rider_script_t *sc)
{
	memset(st, 0, sizeof(*st));
	st->sc = sc;
	st->total_ticks = rider_script_total_ticks(sc);
	st->cum_deg = sc->start_angle_deg;
	st->seg_start_cum_deg = st->cum_deg;
	st->noise_state = sc->seed;
	st->jitter_state = sc->seed ^ 0x9E3779B9U;
	if (st->jitter_state == 0U) st->jitter_state = 0x1234567U;
	st->last_index = 0U;
}

static int32_t floor_div_pos(double rel_deg)
{
	return (int32_t)floor(rel_deg / RS_STEP_DEG + 1e-9);
}

static bool jitter_active(const rider_script_state_t *st, uint32_t k, uint32_t *ticks)
{
	const rider_script_t *sc = st->sc;
	for (uint32_t i = 0; i < sc->nev; i++) {
		const rs_event_t *e = &sc->ev[i];
		if (e->type != RS_EV_JITTER) continue;
		uint32_t a = sec_to_tick(e->t_s), b = sec_to_tick(e->t_s + e->dur_s);
		if (k >= a && k < b) { *ticks = e->ticks; return true; }
	}
	return false;
}

static void push_edge(rider_script_state_t *st, int8_t dir, uint32_t k, uint8_t *glitch)
{
	uint32_t release = k;
	uint32_t jt;
	if (jitter_active(st, k, &jt) && jt > 0U) {
		release = k + (xorshift(&st->jitter_state) % (jt + 1U));
		*glitch |= RS_GLITCH_JITTER;
	}
	/* Delayed edges never overtake each other: the lines cannot show edge n+1 before edge n. */
	if (st->qn > 0U && release <= st->last_release_tick) release = st->last_release_tick + 1U;
	if (st->qn >= RIDER_SCRIPT_EDGE_QUEUE) {
		/* Cannot happen at physical cadences; fall back to an immediate release. */
		st->obs_pos += dir;
		return;
	}
	rs_edge_t *e = &st->q[st->qn];
	e->dir = dir;
	e->held = false;
	e->release_tick = release;
	/* A withheld edge is released on the same tick as this one: an observed two-bit jump. */
	for (uint32_t i = 0; i < st->qn; i++) {
		if (st->q[i].held) { st->q[i].held = false; st->q[i].release_tick = release; }
	}
	if (st->drop_armed > 0U) {
		e->held = true;
		st->drop_armed--;
		*glitch |= RS_GLITCH_DROP;
	}
	st->qn++;
	st->last_release_tick = release;
}

static void fire_events(rider_script_state_t *st, uint32_t k)
{
	const rider_script_t *sc = st->sc;
	for (uint32_t i = 0; i < sc->nev; i++) {
		const rs_event_t *e = &sc->ev[i];
		uint32_t occurrences = (e->type == RS_EV_BOUNCE || e->type == RS_EV_JITTER) ? 1U : e->count;
		while (st->ev_fired[i] < occurrences) {
			uint32_t at = sec_to_tick(e->t_s + (double)st->ev_fired[i] * e->period_s);
			if (k < at) break;
			st->ev_fired[i]++;
			switch (e->type) {
			case RS_EV_BOUNCE: st->bounce_budget += e->count; break;
			case RS_EV_DROP: st->drop_armed++; break;
			case RS_EV_ILLEGAL: st->illegal_left = e->ticks; break;
			case RS_EV_STALL: if (e->ticks > 1U) st->stall_left = e->ticks - 1U; break;
			case RS_EV_JITTER: break;
			}
		}
	}
}

bool rider_script_step(rider_script_state_t *st, rider_script_sample_t *out)
{
	const rider_script_t *sc = st->sc;
	if (st->tick >= st->total_ticks) return false;
	const uint32_t k = st->tick++;
	uint8_t glitch = 0U;

	while (st->seg_index + 1U < sc->nseg && k >= seg_end_tick(sc, st->seg_index)) {
		st->seg_index++;
		st->seg_start_tick = k;
		st->seg_start_cum_deg = st->cum_deg;
	}
	const rs_segment_t *seg = &sc->seg[st->seg_index];
	const uint32_t seg_begin = st->seg_index ? seg_end_tick(sc, st->seg_index - 1U) : 0U;
	const uint32_t seg_ticks = seg_end_tick(sc, st->seg_index) - seg_begin;
	const double frac = seg_ticks ? (double)(k - seg_begin) / (double)seg_ticks : 0.0;
	const double rpm = seg->rpm0 + (seg->rpm1 - seg->rpm0) * frac;
	const double mean = seg->mean0 + (seg->mean1 - seg->mean0) * frac;
	const double speed = seg->speed0 + (seg->speed1 - seg->speed0) * frac;

	fire_events(st, k);

	/* Crank kinematics: physical angle, slowest through the dead spots (0/180 deg of the shape). */
	double inst = rpm;
	if (rpm > 0.0 && sc->cadence_ripple > 0.0) {
		double a = (st->cum_deg - sc->shape.phase_shift_deg) * M_PI / 180.0;
		inst = rpm * (1.0 - sc->cadence_ripple * cos(2.0 * a));
	}
	st->cum_deg += inst * 6.0 / RIDER_SCRIPT_TICK_HZ;

	/* Quadrature: one transition every 3.75 deg, counted from the start angle (no TDC). */
	int32_t new_pos = floor_div_pos(st->cum_deg - sc->start_angle_deg);
	bool edge = false;
	while (st->true_pos != new_pos) {
		int8_t dir = (new_pos > st->true_pos) ? 1 : -1;
		st->true_pos += dir;
		push_edge(st, dir, k, &glitch);
		edge = true;
	}

	/* Output index. A bounce armed on the previous tick shows the previous state for one tick. */
	bool bounce_now = false;
	if (st->bounce_pending) {
		st->bounce_pending = 0U;
		bounce_now = true;
	}
	uint8_t before = (uint8_t)(((st->obs_pos % 4) + 4) % 4);
	bool fwd_released = false;
	while (st->qn > 0U && !st->q[0].held && st->q[0].release_tick <= k) {
		st->obs_pos += st->q[0].dir;
		if (st->q[0].dir > 0) fwd_released = true;
		memmove(&st->q[0], &st->q[1], (st->qn - 1U) * sizeof(st->q[0]));
		st->qn--;
	}
	uint8_t idx = (uint8_t)(((st->obs_pos % 4) + 4) % 4);
	if (fwd_released && st->bounce_budget > 0U) {
		st->bounce_budget--;
		st->bounce_pending = 1U;
		st->bounce_prev_index = before;
	}
	if (st->illegal_left > 0U) {
		st->illegal_left--;
		idx = (uint8_t)((idx + 2U) & 3U);
		glitch |= RS_GLITCH_ILLEGAL;
	} else if (bounce_now) {
		idx = st->bounce_prev_index;
		glitch |= RS_GLITCH_BOUNCE;
	}
	st->last_index = idx;

	/* Foreground scheduling. */
	uint32_t elapsed;
	if (st->stall_left > 0U) {
		st->stall_left--;
		st->stall_elapsed++;
		elapsed = 0U;
		glitch |= RS_GLITCH_STALL;
	} else {
		elapsed = 1U + st->stall_elapsed;
		st->stall_elapsed = 0U;
	}

	/* Torque. */
	double angle = fmod(st->cum_deg, 360.0);
	if (angle < 0.0) angle += 360.0;
	double clean = mean > 0.0 ? mean * rider_script_shape(sc, st->cum_deg) : 0.0;
	double torque = clean;
	if (sc->noise_ckg > 0.0) torque += sc->noise_ckg * gauss(st);
	if (torque < 0.0) torque = 0.0;

	/* Ground truth. */
	rs_label_t label;
	switch (seg->kind) {
	case RS_SEG_BRAKE: label = RS_LABEL_BRAKE; break;
	case RS_SEG_REVERSE: label = RS_LABEL_REVERSE; break;
	case RS_SEG_STOP: label = RS_LABEL_PEDAL_STOP; break;
	case RS_SEG_COAST: label = RS_LABEL_COAST; break;
	case RS_SEG_RELEASE: label = RS_LABEL_TRUE_RELEASE; break;
	default:
		if (seg->kind == RS_SEG_ATTACK && fabs(st->cum_deg - st->seg_start_cum_deg) < 360.0)
			label = RS_LABEL_ATTACK;
		else if (mean > 0.0 && clean < sc->dip_threshold * mean)
			label = RS_LABEL_PHASE_DIP;
		else
			label = RS_LABEL_NORMAL;
		break;
	}
	double intent = (label == RS_LABEL_BRAKE || label == RS_LABEL_REVERSE ||
	                 label == RS_LABEL_PEDAL_STOP || label == RS_LABEL_COAST) ? 0.0 : mean;

	memset(out, 0, sizeof(*out));
	out->tick = st->tick;
	out->t_s = (double)k / RIDER_SCRIPT_TICK_HZ;
	out->cadence_rpm = inst;
	out->crank_deg = angle;
	out->crank_cum_deg = st->cum_deg;
	out->torque_ckg = torque;
	out->torque_clean_ckg = clean;
	out->intent_ckg = intent;
	out->mean_ckg = mean;
	double sx = speed * 100.0;
	out->speed_x100 = (uint16_t)(sx > 65535.0 ? 65535.0 : llround(sx));
	out->brake = seg->kind == RS_SEG_BRAKE;
	out->pas_index = idx;
	out->true_pos = st->true_pos;
	out->edge = edge;
	out->elapsed = elapsed;
	out->glitch = glitch;
	out->label = label;
	out->seg = st->seg_index;
	out->seg_kind = seg->kind;
	return true;
}
