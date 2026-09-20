#!/usr/bin/env python3
"""
FW-112 eMTB P/U CEILING FORENSIC SWEEP
Exact replication of assist_modes.c math for the P/U ceiling hypothesis.

All constants from production code. No simplification.
"""

import csv
import sys
import io

# === PRODUCTION CONSTANTS (assist_modes.c) ===
EMTB_TORQUE_RANGE = 160
EMTB_PARAMETER_MAX = 250
EMTB_DENOMINATOR_BASE = 510
EMTB_DENOMINATOR_MIN = 10
EMTB_REFERENCE_VOLTAGE_MIN_MV = 24000
EMTB_REFERENCE_VOLTAGE_MAX_MV = 60000
EMTB_UNIT_CURRENT_MA = 160
EMTB_FIXED_Q_SHIFT = 8
EMTB_FIXED_Q_ONE = 1 << EMTB_FIXED_Q_SHIFT  # 256
MOTOR_VOLTAGE_UTILIZATION_SCALE = 2048

# === TORQUE INPUT CONSTANTS (torque_input.h) ===
# FW-150: three-point default curve (origin implicit), measured 2026-09-17.
TORQUE_CURVE = ((20, 300), (185, 950), (780, 2000))
TORQUE_SPAN_MAX_NATIVE = 4200
TORQUE_ASSIST_DEADBAND_NATIVE = 10
TORQUE_PUBLIC_FULL_SCALE_CENTIKG = 6000
TORQUE_INPUT_MAX_CENTIKG = 12000

# === POWER CONSTANTS (config.h) ===
CAL_I = 95
_U_MAX = 1920  # DISABLE_DYNAMIC_ADC commented out

# === HUMAN POWER (assist_modes.c) ===
HUMAN_POWER_NUM = 1694
HUMAN_POWER_DEN = 1000

# === DEFAULT LEVEL CONFIGS (assist_modes.c) ===
# Format: (mode_type, support_ratio_pct, emtb_parameter, max_iq_pct, max_motor_power_w)
LEVELS = {
    # eMTB bank
    "eMTB_1": {"mode": "EMTB", "ratio": 100, "emtb_param": 60,  "max_iq_pct": 100, "max_power_w": 250, "ref_voltage_mv": 36000, "based_on_power": True},
    "eMTB_2": {"mode": "EMTB", "ratio": 200, "emtb_param": 100, "max_iq_pct": 100, "max_power_w": 250, "ref_voltage_mv": 36000, "based_on_power": True},
    "eMTB_3": {"mode": "EMTB", "ratio": 320, "emtb_param": 140, "max_iq_pct": 100, "max_power_w": 250, "ref_voltage_mv": 36000, "based_on_power": True},
    "eMTB_4": {"mode": "EMTB", "ratio": 420, "emtb_param": 160, "max_iq_pct": 100, "max_power_w": 250, "ref_voltage_mv": 36000, "based_on_power": True},
    "eMTB_5": {"mode": "EMTB", "ratio": 520, "emtb_param": 180, "max_iq_pct": 100, "max_power_w": 250, "ref_voltage_mv": 36000, "based_on_power": True},
    # POWER_LINEAR bank
    "LIN_1": {"mode": "LINEAR", "ratio": 100, "emtb_param": 0, "max_iq_pct": 100, "max_power_w": 250},
    "LIN_2": {"mode": "LINEAR", "ratio": 200, "emtb_param": 0, "max_iq_pct": 100, "max_power_w": 250},
    "LIN_3": {"mode": "LINEAR", "ratio": 320, "emtb_param": 0, "max_iq_pct": 100, "max_power_w": 250},
    "LIN_4": {"mode": "LINEAR", "ratio": 420, "emtb_param": 0, "max_iq_pct": 100, "max_power_w": 250},
    "LIN_5": {"mode": "LINEAR", "ratio": 520, "emtb_param": 0, "max_iq_pct": 100, "max_power_w": 250},
}


def _lerp(x, x0, x1, y0, y1):
    return y0 + ((x - x0) * (y1 - y0) + (x1 - x0) // 2) // (x1 - x0)


def default_native_delta_to_centikg(delta_native):
    """Replicate default_native_delta_to_centikg() in torque_input.c"""
    if delta_native <= TORQUE_CURVE[0][0]:
        load = _lerp(delta_native, 0, TORQUE_CURVE[0][0], 0, TORQUE_CURVE[0][1])
    else:
        i = len(TORQUE_CURVE) - 1
        while i > 1 and delta_native <= TORQUE_CURVE[i - 1][0]:
            i -= 1
        load = _lerp(delta_native, TORQUE_CURVE[i - 1][0], TORQUE_CURVE[i][0],
                     TORQUE_CURVE[i - 1][1], TORQUE_CURVE[i][1])
    return min(load, TORQUE_INPUT_MAX_CENTIKG)


def native_delta_to_centikg(delta_native):
    """Replicate torque_input.c:128-137 (default calibration)"""
    return default_native_delta_to_centikg(delta_native)


def centikg_to_native_delta(centikg):
    """Inverse: centikg -> native (for input mapping)"""
    if centikg <= TORQUE_CURVE[0][1]:
        delta = _lerp(centikg, 0, TORQUE_CURVE[0][1], 0, TORQUE_CURVE[0][0])
    else:
        i = len(TORQUE_CURVE) - 1
        while i > 1 and centikg <= TORQUE_CURVE[i - 1][1]:
            i -= 1
        delta = _lerp(centikg, TORQUE_CURVE[i - 1][1], TORQUE_CURVE[i][1],
                      TORQUE_CURVE[i - 1][0], TORQUE_CURVE[i][0])
    return min(delta, TORQUE_SPAN_MAX_NATIVE)


def calculate_human_power_mw(load_centikg, cadence_rpm):
    """Replicate assist_modes.c:547-560"""
    product = load_centikg * cadence_rpm
    return product + (product * (HUMAN_POWER_NUM - HUMAN_POWER_DEN) +
                      HUMAN_POWER_DEN // 2) // HUMAN_POWER_DEN


def emtb_calculate(load_centikg, cadence_rpm, level_cfg, battery_mv, iq_limit, u_abs):
    """Full eMTB pipeline replication from assist_modes.c:945-1033"""
    emtb_parameter = level_cfg["emtb_param"]
    max_iq_pct = level_cfg["max_iq_pct"]
    max_power_w = level_cfg["max_power_w"]
    ref_voltage_mv = level_cfg["ref_voltage_mv"]
    based_on_power = level_cfg["based_on_power"]

    # Early returns
    if battery_mv == 0 or iq_limit <= 0 or emtb_parameter == 0 or max_iq_pct == 0:
        return {"error": "early_return_config"}

    # Torque for assist = ARUN (steady state ≈ assist_delta after deadband)
    # Assume steady state: ARUN ≈ delta_native - DEADBAND
    delta_native = centikg_to_native_delta(load_centikg)
    assist_delta = max(0, delta_native - TORQUE_ASSIST_DEADBAND_NATIVE)
    torque_for_assist = assist_delta  # ARUN steady state ≈ AFILT ≈ assist_delta

    # Clamp to span
    torque_range = TORQUE_SPAN_MAX_NATIVE
    if torque_for_assist > torque_range:
        torque_for_assist = torque_range

    # eMTB: delta_x160_q
    delta_x160_q = (torque_for_assist * EMTB_TORQUE_RANGE * EMTB_FIXED_Q_ONE +
                    torque_range // 2) // torque_range

    # eMTB denominator
    parameter = min(emtb_parameter, EMTB_PARAMETER_MAX)
    denominator = EMTB_DENOMINATOR_BASE - 2 * parameter
    if based_on_power:
        if denominator > cadence_rpm:
            denominator -= cadence_rpm
        else:
            denominator = 0
    denominator += EMTB_DENOMINATOR_MIN

    # eMTB target
    if denominator == 0:
        target_x160_q = 0
    else:
        target_x160_q = (delta_x160_q * delta_x160_q) // (denominator * EMTB_FIXED_Q_ONE)

    # Phase Iq request (before P/U)
    full_scale_q = EMTB_TORQUE_RANGE * EMTB_FIXED_Q_ONE
    if iq_limit <= 0 or target_x160_q == 0:
        phase_iq_before = 0
    else:
        tq_clamped = min(target_x160_q, full_scale_q)
        phase_iq_before = (tq_clamped * iq_limit + full_scale_q - 1) // full_scale_q

    # Human power
    human_power_mw = calculate_human_power_mw(load_centikg, cadence_rpm)
    assist_basis_power_mw = calculate_human_power_mw(
        native_delta_to_centikg(torque_for_assist), cadence_rpm)

    # Motor power
    target_for_power_q = min(target_x160_q, full_scale_q)
    target_current_ma = (target_for_power_q * EMTB_UNIT_CURRENT_MA +
                         EMTB_FIXED_Q_ONE // 2) // EMTB_FIXED_Q_ONE
    motor_power_mw = (target_current_ma * ref_voltage_mv) // 1000

    # Power limit
    power_limit_mw = max_power_w * 1000
    if motor_power_mw > power_limit_mw:
        motor_power_mw = power_limit_mw

    # requested_current_ma (battery current -> phase current)
    if battery_mv == 0:
        requested_current_ma = 0
    else:
        requested_current_ma = (motor_power_mw * 1000) // battery_mv

    # Profile Iq limit
    profile_iq_limit = (iq_limit * max_iq_pct) // 100
    phase_iq_after = min(max(phase_iq_before, 0), profile_iq_limit)

    # P/U ceiling (assist_modes.c:821-828)
    pu_zeroed = False
    pu_limit = None
    start_phase = False  # Assume not in start phase for steady-state riding
    if (not start_phase and requested_current_ma > 0 and u_abs > 0):
        power_iq_limit = (requested_current_ma * MOTOR_VOLTAGE_UTILIZATION_SCALE) // (u_abs * CAL_I)
        pu_limit = power_iq_limit
        if phase_iq_after > power_iq_limit:
            phase_iq_after = power_iq_limit
            pu_zeroed = (power_iq_limit == 0)

    return {
        "delta_native": delta_native,
        "assist_delta": assist_delta,
        "torque_for_assist": torque_for_assist,
        "delta_x160_q": delta_x160_q,
        "denominator": denominator,
        "target_x160_q": target_x160_q,
        "phase_iq_before_pu": phase_iq_before,
        "profile_iq_limit": profile_iq_limit,
        "phase_iq_after_profile": min(max(phase_iq_before, 0), profile_iq_limit),
        "human_power_mw": human_power_mw,
        "assist_basis_power_mw": assist_basis_power_mw,
        "target_current_ma": target_current_ma,
        "motor_power_mw": motor_power_mw,
        "requested_current_ma": requested_current_ma,
        "pu_limit": pu_limit,
        "phase_iq_after_pu": phase_iq_after,
        "pu_zeroed": pu_zeroed,
    }


def linear_calculate(load_centikg, cadence_rpm, level_cfg, battery_mv, iq_limit, u_abs):
    """LINEAR mode pipeline (assist_modes.c calculate_power path)"""
    support_ratio = level_cfg["ratio"]
    max_iq_pct = level_cfg["max_iq_pct"]
    max_power_w = level_cfg["max_power_w"]

    if battery_mv == 0 or iq_limit <= 0 or support_ratio == 0 or max_iq_pct == 0:
        return {"error": "early_return"}

    delta_native = centikg_to_native_delta(load_centikg)
    assist_delta = max(0, delta_native - TORQUE_ASSIST_DEADBAND_NATIVE)
    torque_for_assist = assist_delta

    # LINEAR: iq = load * ratio / 3000 (calculate_load_iq_request)
    load_clamped = min(load_centikg, TORQUE_PUBLIC_FULL_SCALE_CENTIKG)
    ratio_clamped = min(support_ratio, 500)
    demand_permille = (load_clamped * ratio_clamped + 1500) // 3000
    phase_iq_before = min(demand_permille * iq_limit // 1000, iq_limit)

    # Profile limit
    profile_iq_limit = (iq_limit * max_iq_pct) // 100
    phase_iq_after = min(max(phase_iq_before, 0), profile_iq_limit)

    # Human power
    human_power_mw = calculate_human_power_mw(load_centikg, cadence_rpm)
    assist_basis_power_mw = human_power_mw

    # Motor power for LINEAR: motor_power = (phase_iq * battery_mv) / 1000
    motor_power_mw = (phase_iq_after * battery_mv) // 1000 if battery_mv > 0 else 0
    power_limit_mw = max_power_w * 1000
    if motor_power_mw > power_limit_mw:
        motor_power_mw = power_limit_mw

    # requested_current_ma
    requested_current_ma = (motor_power_mw * 1000) // battery_mv if battery_mv > 0 else 0

    # P/U ceiling
    pu_zeroed = False
    pu_limit = None
    start_phase = False
    if (not start_phase and requested_current_ma > 0 and u_abs > 0):
        power_iq_limit = (requested_current_ma * MOTOR_VOLTAGE_UTILIZATION_SCALE) // (u_abs * CAL_I)
        pu_limit = power_iq_limit
        if phase_iq_after > power_iq_limit:
            phase_iq_after = power_iq_limit
            pu_zeroed = (power_iq_limit == 0)

    return {
        "delta_native": delta_native,
        "assist_delta": assist_delta,
        "torque_for_assist": torque_for_assist,
        "phase_iq_before_pu": phase_iq_before,
        "profile_iq_limit": profile_iq_limit,
        "phase_iq_after_profile": min(max(phase_iq_before, 0), profile_iq_limit),
        "human_power_mw": human_power_mw,
        "motor_power_mw": motor_power_mw,
        "requested_current_ma": requested_current_ma,
        "pu_limit": pu_limit,
        "phase_iq_after_pu": phase_iq_after,
        "pu_zeroed": pu_zeroed,
    }


def pu_threshold_table():
    """Section 3: u_abs threshold table - minimum requested_current_ma for power_iq_limit >= 1"""
    print("\n=== P/U THRESHOLD TABLE ===")
    print("power_iq_limit = (requested_current_ma * 2048) / (u_abs * 95)")
    print("For power_iq_limit >= 1: requested_current_ma >= ceil(u_abs * 95 / 2048)")
    print()
    print(f"{'u_abs':>8}  {'threshold_ma':>14}  {'note'}")
    print("-" * 60)
    for u_abs in [100, 200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800, 1920, 2000]:
        threshold = (u_abs * CAL_I + MOTOR_VOLTAGE_UTILIZATION_SCALE - 1) // MOTOR_VOLTAGE_UTILIZATION_SCALE
        note = "at _U_MAX" if u_abs == 1920 else ""
        if u_abs == 2000:
            note = "beyond _U_MAX (clamped in hw)"
        print(f"{u_abs:>8}  {threshold:>14} mA  {note}")


def emtb_sweep(load_ckg, cadence, level_name, level_cfg, battery_mv, iq_limit, u_abs):
    """Section 4: full eMTB sweep for one combination"""
    r = emtb_calculate(load_ckg, cadence, level_cfg, battery_mv, iq_limit, u_abs)
    return r


def hardware_reconstruction():
    """Section 6: Real hardware case reconstruction"""
    print("\n=== HARDWARE-LIKE 800/40 RECONSTRUCTION ===")
    print("HMI torque = 800 ckg, cadence = 40 rpm")
    print("Assume: iq_limit = 1000 (typical full battery)")
    print()

    load_ckg = 800
    cadence = 40

    # Realistic battery voltages
    batteries = [
        ("36V discharged", 34000),
        ("36V nominal", 36000),
        ("36V full", 42000),
        ("48V nominal", 48000),
        ("48V full", 54600),
    ]

    # Realistic u_abs ranges
    # At low speed, u_abs is low (motor not loaded yet). At riding speed ~20 km/h, u_abs ≈ 400-1000
    u_abs_values = [200, 400, 600, 800, 1000, 1200, 1600, 1920]

    iq_limit = 1000

    for level_name in ["eMTB_1", "eMTB_3", "eMTB_5"]:
        level_cfg = LEVELS[level_name]
        print(f"\n--- {level_name} (param={level_cfg['emtb_param']}) ---")
        print(f"{'u_abs':>6}  {'bat_mV':>7}  {'iq_before':>9}  {'req_mA':>7}  {'pu_lim':>6}  {'iq_after':>8}  {'PU_ZERO'}")
        print("-" * 70)
        for bat_name, bat_mv in batteries:
            for u_abs in u_abs_values:
                r = emtb_calculate(load_ckg, cadence, level_cfg, bat_mv, iq_limit, u_abs)
                pu_str = ">>>ZERO<<<" if r["pu_zeroed"] else ""
                print(f"{u_abs:>6}  {bat_mv:>7}  {r['phase_iq_before_pu']:>9}  "
                      f"{r['requested_current_ma']:>7}  {str(r['pu_limit']):>6}  "
                      f"{r['phase_iq_after_pu']:>8}  {pu_str}")
            print()  # blank line between batteries


def full_map():
    """Section 5: Classification map"""
    print("\n=== FULL CLASSIFICATION MAP ===")
    print("A=NO_INPUT B=eMTB_RAW_ZERO C=RAW_POS+PU_POS D=RAW_POS+PU_ZERO E=LIMITER_ZERO")
    print()

    iq_limit = 1000
    battery_mv = 36000

    loads = [400, 500, 600, 700, 750, 800, 850, 900, 1000, 1200, 1500]
    cadences = [20, 30, 40, 50, 60, 80, 100]
    u_abs_values = [400, 800, 1200, 1920]

    for u_abs in u_abs_values:
        print(f"\n--- u_abs = {u_abs} ---")
        header = f"{'load':>6}"
        for cad in cadences:
            header += f"  {cad:>4}rpm"
        print(header)
        print("-" * (8 + len(cadences) * 7))

        for level_name in ["eMTB_3", "eMTB_5"]:
            level_cfg = LEVELS[level_name]
            print(f"  {level_name}:")
            for load in loads:
                row = f"{load:>6}"
                for cad in cadences:
                    r = emtb_calculate(load, cad, level_cfg, battery_mv, iq_limit, u_abs)
                    if r.get("error"):
                        row += "     ERR"
                    elif r["phase_iq_before_pu"] == 0:
                        row += "       B"
                    elif r["pu_zeroed"]:
                        row += "       D"
                    elif r["phase_iq_after_pu"] > 0:
                        row += "       C"
                    else:
                        row += "       E"
                print(row)


def linear_comparison():
    """Section 7: LINEAR vs eMTB comparison"""
    print("\n=== LINEAR vs eMTB COMPARISON ===")
    print("Same torque/cadence/voltage/level, different mode")
    print()

    iq_limit = 1000
    battery_mv = 36000
    u_abs = 800

    loads = [400, 600, 800, 1000, 1200]
    cadences = [30, 40, 60]

    for cad in cadences:
        print(f"\n--- cadence = {cad} rpm, u_abs = {u_abs}, battery = 36V ---")
        print(f"{'load':>6}  {'LIN_before':>10}  {'LIN_PU':>6}  {'LIN_after':>9}  "
              f"{'eMTB3_before':>12}  {'eMTB3_PU':>8}  {'eMTB3_after':>10}  "
              f"{'eMTB5_before':>12}  {'eMTB5_PU':>8}  {'eMTB5_after':>10}")
        print("-" * 110)

        for load in loads:
            lin = linear_calculate(load, cad, LEVELS["LIN_3"], battery_mv, iq_limit, u_abs)
            emtb3 = emtb_calculate(load, cad, LEVELS["eMTB_3"], battery_mv, iq_limit, u_abs)
            emtb5 = emtb_calculate(load, cad, LEVELS["eMTB_5"], battery_mv, iq_limit, u_abs)

            lin_pu = "ZERO" if lin["pu_zeroed"] else str(lin["pu_limit"])
            e3_pu = "ZERO" if emtb3["pu_zeroed"] else str(emtb3["pu_limit"])
            e5_pu = "ZERO" if emtb5["pu_zeroed"] else str(emtb5["pu_limit"])

            print(f"{load:>6}  {lin['phase_iq_before_pu']:>10}  {lin_pu:>6}  {lin['phase_iq_after_pu']:>9}  "
                  f"{emtb3['phase_iq_before_pu']:>12}  {e3_pu:>8}  {emtb3['phase_iq_after_pu']:>10}  "
                  f"{emtb5['phase_iq_before_pu']:>12}  {e5_pu:>8}  {emtb5['phase_iq_after_pu']:>10}")


def boundary_analysis():
    """Find the exact torque/cadence boundary where P/U kills the request"""
    print("\n=== P/U ZERO BOUNDARY FOR eMTB_3 (param=140) ===")
    print("Load where phase_iq_before > 0 but power_iq_limit = 0")
    print()

    iq_limit = 1000
    battery_mv = 36000

    for u_abs in [400, 800, 1200, 1920]:
        print(f"\n--- u_abs = {u_abs} ---")
        threshold_ma = (u_abs * CAL_I + MOTOR_VOLTAGE_UTILIZATION_SCALE - 1) // MOTOR_VOLTAGE_UTILIZATION_SCALE
        print(f"  P/U threshold: requested_current_ma >= {threshold_ma} mA for iq_limit >= 1")

        # Binary search: find minimum load where phase_iq_after_pu > 0
        for cad in [30, 40, 60]:
            min_load = 0
            max_load = 3000
            # First check: is there ANY load that survives?
            r_max = emtb_calculate(max_load, cad, LEVELS["eMTB_3"], battery_mv, iq_limit, u_abs)
            if r_max["phase_iq_after_pu"] == 0:
                print(f"  cad={cad:>3}: NO load survives P/U (even 3000 ckg => iq=0)")
                continue

            # Binary search
            while max_load - min_load > 1:
                mid = (min_load + max_load) // 2
                r = emtb_calculate(mid, cad, LEVELS["eMTB_3"], battery_mv, iq_limit, u_abs)
                if r["phase_iq_after_pu"] > 0:
                    max_load = mid
                else:
                    min_load = mid

            # Show boundary
            r_below = emtb_calculate(min_load, cad, LEVELS["eMTB_3"], battery_mv, iq_limit, u_abs)
            r_above = emtb_calculate(max_load, cad, LEVELS["eMTB_3"], battery_mv, iq_limit, u_abs)
            r_800 = emtb_calculate(800, cad, LEVELS["eMTB_3"], battery_mv, iq_limit, u_abs)

            print(f"  cad={cad:>3}: boundary = {max_load} ckg "
                  f"(at {min_load}: iq={r_below['phase_iq_after_pu']}, "
                  f"at {max_load}: iq={r_above['phase_iq_after_pu']})")
            print(f"         at load=800: iq_before={r_800['phase_iq_before_pu']}, "
                  f"req_mA={r_800['requested_current_ma']}, "
                  f"pu_limit={r_800['pu_limit']}, iq_after={r_800['phase_iq_after_pu']}")


def exact_800_40_reconstruction():
    """Section 6 focused: exact reconstruction of user case"""
    print("\n=== EXACT 800/40 RECONSTRUCTION ===")

    load_ckg = 800
    cadence = 40
    iq_limit = 1000

    # Step-by-step for eMTB_3
    r = emtb_calculate(load_ckg, cadence, LEVELS["eMTB_3"], 36000, iq_limit, 800)

    print(f"\nInput: load_centikg={load_ckg}, cadence={cadence} rpm")
    print(f"Level: eMTB_3 (param=140, based_on_power=true)")
    print(f"Battery: 36V, iq_limit={iq_limit}")
    print(f"u_abs: 800 (typical mid-speed)")
    print()
    print(f"Step 1: delta_native = {r['delta_native']}")
    print(f"Step 2: assist_delta (after deadband) = {r['assist_delta']}")
    print(f"Step 3: ARUN steady-state ~ {r['torque_for_assist']}")
    print(f"Step 4: delta_x160_q = {r['delta_x160_q']} (Q8 format, = {r['delta_x160_q']/256:.1f})")
    print(f"Step 5: denominator = {r['denominator']}")
    print(f"Step 6: target_x160_q = {r['target_x160_q']} (Q8, = {r['target_x160_q']/256:.1f})")
    print(f"Step 7: phase_iq BEFORE P/U = {r['phase_iq_before_pu']}")
    print(f"Step 8: motor_power_mw = {r['motor_power_mw']}")
    print(f"Step 9: requested_current_ma = {r['requested_current_ma']}")
    print(f"Step 10: power_iq_limit (P/U) = {r['pu_limit']}")
    print(f"Step 11: phase_iq AFTER P/U = {r['phase_iq_after_pu']}")
    print(f"PU_ZEROED = {r['pu_zeroed']}")

    if r["pu_zeroed"]:
        print("\n*** P/U CEILING CONFIRMED: raw request > 0 but P/U kills it to 0 ***")
    elif r["phase_iq_after_pu"] > 0:
        print(f"\n*** P/U does NOT zero this case (iq_after={r['phase_iq_after_pu']}) ***")
        # Check lower u_abs
        for u in [200, 300, 400, 500, 600]:
            r2 = emtb_calculate(load_ckg, cadence, LEVELS["eMTB_3"], 36000, iq_limit, u)
            if r2["pu_zeroed"]:
                print(f"    BUT at u_abs={u}: P/U zeros (req={r2['requested_current_ma']}, "
                      f"pu_lim={r2['pu_limit']}, iq_before={r2['phase_iq_before_pu']})")
            else:
                print(f"    at u_abs={u}: iq_after={r2['phase_iq_after_pu']} (pu_lim={r2['pu_limit']})")


if __name__ == "__main__":
    print("=" * 80)
    print("FW-112 eMTB P/U CEILING FORENSIC SWEEP")
    print("All constants from production assist_modes.c / torque_input.c / config.h")
    print("=" * 80)

    # Section 3: P/U threshold
    pu_threshold_table()

    # Section 4+5: Full map
    full_map()

    # Section 6: Hardware reconstruction
    exact_800_40_reconstruction()
    hardware_reconstruction()

    # Section 7: LINEAR comparison
    linear_comparison()

    # Boundary analysis
    boundary_analysis()
