#!/usr/bin/env python3
"""
generate_board_config.py
========================
Reads  config/board_config.yaml  and emits
       ${CMAKE_BINARY_DIR}/generated/board_config_generated.h

Called automatically by CMake at configure-time.

Usage (stand-alone):
    python3 scripts/generate_board_config.py  \
        config/board_config.yaml              \
        build/generated/board_config_generated.h
"""

import sys, os, yaml, textwrap

def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def emit_header(cfg, outpath):
    board   = cfg.get("board", {})
    can     = cfg.get("can", {})
    brushed = cfg.get("brushed_motors", []) or []
    bldc    = cfg.get("brushless_motors", []) or []
    servos  = cfg.get("servos", []) or []

    lines = []
    a = lines.append

    a("/* AUTO-GENERATED – DO NOT EDIT                        */")
    a("/* Re-generate with: python3 scripts/generate_board_config.py */")
    a("#ifndef BOARD_CONFIG_GENERATED_H")
    a("#define BOARD_CONFIG_GENERATED_H")
    a("")

    # ── board identity ──
    a(f'#define BOARD_NAME           "{board.get("name", "unnamed")}"')
    a(f'#define BOARD_NODE_ID        0x{board.get("node_id", 1):02X}')
    uid_src = board.get("unique_id_source", "hwinfo")
    a(f'#define BOARD_UID_SOURCE     "{uid_src}"')
    a(f'#define BOARD_UID_USE_HWINFO {1 if uid_src == "hwinfo" else 0}')
    a(f'#define BOARD_UID_MANUAL     0x{board.get("unique_id", 0):08X}')
    a("")

    # ── CAN ──
    a(f'#define CAN_NOMINAL_BITRATE  {can.get("nominal_bitrate", 1000000)}')
    a(f'#define CAN_DATA_BITRATE     {can.get("data_bitrate", 2000000)}')
    a(f'#define HEARTBEAT_INTERVAL_MS {can.get("heartbeat_ms", 1000)}')
    a(f'#define STATUS_INTERVAL_MS   {can.get("status_ms", 100)}')
    a(f'#define CONTROL_LOOP_HZ      {can.get("control_loop_hz", 100)}')
    a(f'#define CONTROL_DT           (1.0f / CONTROL_LOOP_HZ)')
    a("")

    # ── Counts ──
    a(f"#define NUM_BRUSHED_MOTORS   {len(brushed)}")
    a(f"#define NUM_BRUSHLESS_MOTORS {len(bldc)}")
    a(f"#define NUM_SERVOS           {len(servos)}")
    a(f"#define NUM_MOTORS           (NUM_BRUSHED_MOTORS + NUM_BRUSHLESS_MOTORS)")
    a("")

    # ── Per-motor / servo config structs ──
    # We encode each entry as a set of #defines so the builder can
    # reference them without a YAML parser on-target.

    for idx, m in enumerate(brushed):
        pfx = f"BRUSHED_MOTOR_{idx}"
        a(f"/* --- Brushed motor {idx}: {m.get('joint_name','')} --- */")
        a(f'#define {pfx}_ID              {m["id"]}')
        a(f'#define {pfx}_JOINT_NAME      "{m["joint_name"]}"')
        a(f'#define {pfx}_PWM_LABEL       DT_NODELABEL({m["pwm_label"]})')
        a(f'#define {pfx}_PWM_CHANNEL     {m["pwm_channel"]}')
        freq = m.get("pwm_frequency_hz", 20000)
        period_ns = int(1e9 / freq)
        a(f'#define {pfx}_PWM_PERIOD_NS   {period_ns}')
        a(f'#define {pfx}_GPIO_LABEL      DT_NODELABEL({m["gpio_label"]})')
        a(f'#define {pfx}_DIR_PIN         {m["dir_pin"]}')
        a(f'#define {pfx}_SLP_PIN         {m["slp_pin"]}')
        a(f'#define {pfx}_FLT_PIN         {m["flt_pin"]}')

        enc = m.get("encoder")
        if enc:
            a(f'#define {pfx}_HAS_ENCODER     1')
            a(f'#define {pfx}_ENC_GPIO_LABEL  DT_NODELABEL({enc["gpio_label"]})')
            a(f'#define {pfx}_ENC_A_PIN       {enc["a_pin"]}')
            a(f'#define {pfx}_ENC_B_PIN       {enc["b_pin"]}')
            a(f'#define {pfx}_ENC_TPR         {enc["ticks_per_rev"]}')
        else:
            a(f'#define {pfx}_HAS_ENCODER     0')

        cs = m.get("current_sense")
        if cs:
            a(f'#define {pfx}_HAS_CS          1')
            a(f'#define {pfx}_CS_ADC_LABEL    DT_NODELABEL({cs["adc_label"]})')
            a(f'#define {pfx}_CS_ADC_CH       {cs["channel"]}')
            a(f'#define {pfx}_CS_OC_MA        {cs["overcurrent_ma"]}')
        else:
            a(f'#define {pfx}_HAS_CS          0')

        pid = m.get("pid", {})
        a(f'#define {pfx}_KP              {pid.get("kp", 1.0)}f')
        a(f'#define {pfx}_KI              {pid.get("ki", 0.01)}f')
        a(f'#define {pfx}_KD              {pid.get("kd", 0.1)}f')

        lim = m.get("limits", {})
        a(f'#define {pfx}_MAX_VEL         {lim.get("max_velocity", 5000.0)}f')
        a(f'#define {pfx}_MAX_ACCEL       {lim.get("max_acceleration", 2000.0)}f')
        a(f'#define {pfx}_MIN_POS         {lim.get("min_position", -1000000)}')
        a(f'#define {pfx}_MAX_POS         {lim.get("max_position",  1000000)}')
        a(f'#define {pfx}_MAX_DUTY        {lim.get("max_duty", 100.0)}f')
        a("")

    for idx, m in enumerate(bldc):
        pfx = f"BRUSHLESS_MOTOR_{idx}"
        a(f"/* --- Brushless motor {idx}: {m.get('joint_name','')} --- */")
        a(f'#define {pfx}_ID              {m["id"]}')
        a(f'#define {pfx}_JOINT_NAME      "{m["joint_name"]}"')
        a(f'#define {pfx}_PWM_LABEL       DT_NODELABEL({m["pwm_label"]})')
        a(f'#define {pfx}_PWM_CHANNEL     {m["pwm_channel"]}')
        a(f'#define {pfx}_PWM_PERIOD_NS   {m.get("pwm_period_ns", 50000)}')
        a(f'#define {pfx}_GPIO_LABEL      DT_NODELABEL({m["gpio_label"]})')
        a(f'#define {pfx}_EN_PIN          {m["enable_pin"]}')
        a(f'#define {pfx}_DIR_PIN         {m["direction_pin"]}')
        a(f'#define {pfx}_BRK_PIN         {m["brake_pin"]}')
        a(f'#define {pfx}_FLT_PIN         {m["fault_pin"]}')
        pid = m.get("pid", {})
        a(f'#define {pfx}_KP              {pid.get("kp", 1.0)}f')
        a(f'#define {pfx}_KI              {pid.get("ki", 0.01)}f')
        a(f'#define {pfx}_KD              {pid.get("kd", 0.1)}f')
        lim = m.get("limits", {})
        a(f'#define {pfx}_MAX_VEL         {lim.get("max_velocity", 10000.0)}f')
        a(f'#define {pfx}_MAX_ACCEL       {lim.get("max_acceleration", 5000.0)}f')
        a(f'#define {pfx}_MIN_POS         {lim.get("min_position", -100000)}')
        a(f'#define {pfx}_MAX_POS         {lim.get("max_position",  100000)}')
        a("")

    for idx, s in enumerate(servos):
        pfx = f"SERVO_{idx}"
        a(f"/* --- Servo {idx}: {s.get('joint_name','')} --- */")
        a(f'#define {pfx}_ID              {s["id"]}')
        a(f'#define {pfx}_JOINT_NAME      "{s["joint_name"]}"')
        a(f'#define {pfx}_PWM_LABEL       DT_NODELABEL({s["pwm_label"]})')
        a(f'#define {pfx}_PWM_CHANNEL     {s["pwm_channel"]}')
        a(f'#define {pfx}_ANGLE_MIN       {s.get("angle_min", -900)}')
        a(f'#define {pfx}_ANGLE_MAX       {s.get("angle_max",  900)}')
        a(f'#define {pfx}_ANGLE_OFFSET    {s.get("angle_offset", 0)}')
        a(f'#define {pfx}_SPEED_DPS       {s.get("speed_dps", 180)}')
        a(f'#define {pfx}_PWM_MIN_US      {s.get("pwm_min_us", 500)}')
        a(f'#define {pfx}_PWM_MAX_US      {s.get("pwm_max_us", 2500)}')
        a(f'#define {pfx}_PWM_PERIOD_US   {s.get("pwm_period_us", 20000)}')
        a("")

    a("#endif /* BOARD_CONFIG_GENERATED_H */")

    os.makedirs(os.path.dirname(outpath), exist_ok=True)
    with open(outpath, "w") as f:
        f.write("\n".join(lines) + "\n")

    print(f"[generate_board_config] wrote {outpath}  "
          f"({len(brushed)} brushed, {len(bldc)} brushless, {len(servos)} servos)")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <yaml_input> <header_output>")
        sys.exit(1)

    cfg = load_yaml(sys.argv[1])
    emit_header(cfg, sys.argv[2])
