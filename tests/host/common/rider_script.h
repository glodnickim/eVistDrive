/*
 * Assist Behavior V3 rider/bike stimulus generator - TEST STIMULUS, NOT FIRMWARE.
 *
 * A deterministic, time-stepped (4 kHz SIL control tick) script of what the rider and the bike
 * do, plus the GROUND TRUTH the V3 classifier is scored against. It deliberately includes no
 * production header (same rule as crank_model.h): a generator that tracked its DUT's constants
 * could hide a divergence instead of exposing it.
 *
 * What it produces every tick (rider_script_sample_t):
 *   - crank angle (signed cumulative and wrapped), instantaneous signed cadence,
 *   - instantaneous pedal torque in centi-kg (the SIL maps it through the production sensor
 *     curve), shaped per crank angle with the crank_model.c half-sine-per-leg law
 *     (ripple, L/R asymmetry, dead-spot depth/width, phase shift), normalised so its mean over
 *     one revolution equals the scripted mean torque exactly, plus seeded Gaussian noise,
 *   - the raw quadrature ring index 0..3 (96 transitions per revolution, derived from crank
 *     angle) AFTER glitch injection (bounce, dropped edge, timing jitter, illegal jump). The
 *     SIL feeds it through the production pas_sampler ISR path, so production PAS code decodes
 *     it exactly as it decodes a real sensor,
 *   - wheel speed (open loop, scripted), brake flag,
 *   - foreground scheduling: elapsed = 0 means the foreground is stalled this tick (only the
 *     4 kHz ISR samples PAS); elapsed = N >= 1 means the foreground runs with elapsed_ticks = N,
 *   - ground truth: label, true rider intent (scripted stroke-envelope mean effort, 0 when the
 *     rider is not asking for forward assist), segment index/kind.
 *
 * SCRIPT FORMAT (text, one command per line, '#' starts a comment, key=value tokens):
 *
 *   name <identifier>
 *   seed <u32>                     PRNG seed (noise and jitter use two derived streams)
 *   noise <ckg>                    torque sensor noise, Gaussian sigma in centi-kg (0 = off)
 *   cadence_ripple <fraction>      2/rev crank speed modulation, slowest at the dead spots
 *   shape ripple=<pct> asym=<pct> dsdepth=<pct> dswidth=<deg> phase=<deg>
 *   start_angle <deg>              crank angle at t = 0 (relative; edges are counted from it)
 *   level <0..5>                   assist_level_index fed to ride_control
 *   phase_current_max <counts>     ride_control phase_current_max / iq_scale
 *   legal <0|1>                    legal speed limiting
 *   decimate <n>                   write one CSV row every n control ticks (default 4 = 1 kHz)
 *   dip_threshold <fraction>       PHASE_DIP when the noise-free torque < fraction * mean (0.5)
 *   seg <kind> dur=<s> [rpm=<a>[:<b>]] [mean=<a>[:<b>]] [speed=<a>[:<b>]]
 *         kind: steady ramp release attack stop coast reverse brake
 *         rpm in crank rpm (negative = backwards), mean in centi-kg, speed in km/h.
 *         a:b ramps linearly over the segment. Omitted values continue from the previous
 *         segment's end value; stop/coast default to rpm=0 mean=0.
 *   event <type> t=<s> [count=<n>] [period=<s>] [ticks=<n>] [dur=<s>]
 *         bounce  : the next <count> forward edges each bounce back for one tick
 *         drop    : <count> times every <period> s, one edge is withheld and released together
 *                   with the following edge (an observed two-bit jump)
 *         jitter  : for <dur> s every edge is delayed by a seeded 0..<ticks> control ticks
 *         illegal : <count> times every <period> s, the lines jump to the diagonal state for
 *                   <ticks> ticks and back
 *         stall   : <count> times every <period> s, the foreground misses <ticks>-1 ticks and
 *                   then runs once with elapsed_ticks = <ticks>
 *
 * GROUND-TRUTH LABEL RULES (per tick, first match wins):
 *   segment brake -> BRAKE, reverse -> REVERSE, stop -> PEDAL_STOP, coast -> COAST,
 *   release -> TRUE_RELEASE, attack -> ATTACK for the first crank revolution of the segment,
 *   otherwise PHASE_DIP when the noise-free instantaneous torque < dip_threshold * mean
 *   (the dip the crank angle explains), else NORMAL.
 *   intent = scripted mean for NORMAL/PHASE_DIP/ATTACK/TRUE_RELEASE; 0 for
 *   BRAKE/REVERSE/PEDAL_STOP/COAST.
 */
#ifndef RIDER_SCRIPT_H
#define RIDER_SCRIPT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RIDER_SCRIPT_TICK_HZ 4000.0
#define RIDER_SCRIPT_TRANSITIONS_PER_REV 96
#define RIDER_SCRIPT_MAX_SEG 32
#define RIDER_SCRIPT_MAX_EV 32
#define RIDER_SCRIPT_EDGE_QUEUE 16

typedef enum {
	RS_LABEL_NORMAL = 0,
	RS_LABEL_PHASE_DIP,
	RS_LABEL_TRUE_RELEASE,
	RS_LABEL_ATTACK,
	RS_LABEL_PEDAL_STOP,
	RS_LABEL_REVERSE,
	RS_LABEL_BRAKE,
	RS_LABEL_COAST
} rs_label_t;

typedef enum {
	RS_SEG_STEADY = 0,
	RS_SEG_RAMP,
	RS_SEG_RELEASE,
	RS_SEG_ATTACK,
	RS_SEG_STOP,
	RS_SEG_COAST,
	RS_SEG_REVERSE,
	RS_SEG_BRAKE
} rs_seg_kind_t;

typedef enum {
	RS_EV_BOUNCE = 0,
	RS_EV_DROP,
	RS_EV_JITTER,
	RS_EV_ILLEGAL,
	RS_EV_STALL
} rs_event_type_t;

/* Bits in rider_script_sample_t.glitch: what was injected on this tick. */
#define RS_GLITCH_BOUNCE  0x01U
#define RS_GLITCH_DROP    0x02U
#define RS_GLITCH_JITTER  0x04U
#define RS_GLITCH_ILLEGAL 0x08U
#define RS_GLITCH_STALL   0x10U

typedef struct {
	rs_seg_kind_t kind;
	double dur_s;
	double rpm0, rpm1;
	double mean0, mean1;     /* centi-kg */
	double speed0, speed1;   /* km/h */
} rs_segment_t;

typedef struct {
	rs_event_type_t type;
	double t_s;
	uint32_t count;
	double period_s;
	uint32_t ticks;
	double dur_s;
} rs_event_t;

typedef struct {
	double ripple_pct;
	double asymmetry_pct;
	double dead_spot_depth_pct;
	double dead_spot_width_deg;
	double phase_shift_deg;
} rs_shape_t;

typedef struct {
	char name[64];
	uint32_t seed;
	double noise_ckg;
	double cadence_ripple;
	rs_shape_t shape;
	double start_angle_deg;
	uint8_t level;
	uint16_t phase_current_max;   /* 0 = harness default */
	bool legal;
	uint32_t decimate;
	double dip_threshold;
	rs_segment_t seg[RIDER_SCRIPT_MAX_SEG];
	uint32_t nseg;
	rs_event_t ev[RIDER_SCRIPT_MAX_EV];
	uint32_t nev;
	double shape_norm;            /* computed: 1 / revolution mean of the raw shape */
} rider_script_t;

typedef struct {
	int8_t dir;
	bool held;                    /* drop: waits for the following edge */
	uint32_t release_tick;
} rs_edge_t;

typedef struct {
	const rider_script_t *sc;
	uint32_t tick;                /* run ticks taken; sample t_s = (tick - 1) / 4000 */
	uint32_t total_ticks;
	double cum_deg;               /* signed cumulative crank angle (deg), starts at start_angle */
	double seg_start_cum_deg;
	uint32_t seg_index;
	uint32_t seg_start_tick;
	int32_t true_pos;             /* floor((cum - start) / 3.75) */
	int32_t obs_pos;              /* what the lines show, after delays/drops */
	uint8_t last_index;
	rs_edge_t q[RIDER_SCRIPT_EDGE_QUEUE];
	uint32_t qn;
	uint32_t last_release_tick;
	uint32_t bounce_budget;
	uint8_t bounce_pending;       /* 1: output the previous index on the next tick */
	uint8_t bounce_prev_index;
	uint32_t drop_armed;          /* edges to withhold */
	uint32_t illegal_left;        /* ticks remaining on the diagonal */
	uint32_t stall_left;          /* foreground ticks still to skip */
	uint32_t stall_elapsed;
	uint32_t ev_fired[RIDER_SCRIPT_MAX_EV];
	uint32_t noise_state;
	uint32_t jitter_state;
	bool have_spare;
	double spare;
} rider_script_state_t;

typedef struct {
	uint32_t tick;
	double t_s;
	double cadence_rpm;
	double crank_deg;
	double crank_cum_deg;
	double torque_ckg;            /* sensor input incl. noise, >= 0 */
	double torque_clean_ckg;      /* noise-free instantaneous torque */
	double intent_ckg;            /* ground truth */
	double mean_ckg;              /* scripted envelope mean */
	uint16_t speed_x100;
	bool brake;
	uint8_t pas_index;            /* 0..3 index into the harness's forward quadrature ring */
	int32_t true_pos;
	bool edge;                    /* a physical crank transition happened this tick */
	uint32_t elapsed;             /* 0 = foreground stalled, else elapsed_ticks for this call */
	uint8_t glitch;
	rs_label_t label;
	uint32_t seg;
	rs_seg_kind_t seg_kind;
} rider_script_sample_t;

/* Parse a script file. Returns 0 on success, otherwise writes a message to err. */
int rider_script_parse_file(const char *path, rider_script_t *out, char *err, size_t errlen);
/* Parse script text (NUL-terminated). Same contract. */
int rider_script_parse_text(const char *text, rider_script_t *out, char *err, size_t errlen);

double rider_script_duration_s(const rider_script_t *sc);
uint32_t rider_script_total_ticks(const rider_script_t *sc);

/* Shape factor at a crank angle, normalised to a revolution mean of exactly 1. */
double rider_script_shape(const rider_script_t *sc, double angle_deg);

void rider_script_start(rider_script_state_t *st, const rider_script_t *sc);
/* Advance one control tick. Returns false once the script is exhausted (no sample written). */
bool rider_script_step(rider_script_state_t *st, rider_script_sample_t *out);

const char *rider_script_label_name(rs_label_t l);
const char *rider_script_kind_name(rs_seg_kind_t k);

#endif /* RIDER_SCRIPT_H */
