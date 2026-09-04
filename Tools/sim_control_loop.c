/* Host-side simulation of the PID + step generator, to confirm the control
 * loop actually converges and the machine sequence produces the right moves.
 * Uses the real PID.c so the maths under test is the shipped code. */
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "PID.h"

#define CONTROL_LOOP_DT   0.001f
#define STEP_FREQ_MAX     7500U
#define STEP_FREQ_MIN     10U
#define DEG_PER_MM        3.6f
#define POS_TOLERANCE_DEG 0.75f
#define PID_DEADBAND_DEG  0.70f
#define STEP_TIMER_HZ     1000000U

/* Plant: a stepper that moves exactly as many degrees as it is pulsed.
 * 1/16 microstepping on a 1.8 deg motor = 3200 microsteps per revolution. */
#define MICROSTEPS_PER_REV 3200.0f
#define DEG_PER_MICROSTEP  (360.0f / MICROSTEPS_PER_REV)

static float plant_pos = 0.0f;   /* true shaft angle, degrees */

/* Advance the plant by one 1 ms tick given a signed step frequency. */
static void plant_tick(float step_velocity)
{
    unsigned freq = (unsigned)fabsf(step_velocity);
    if (freq < STEP_FREQ_MIN) return;
    if (freq > STEP_FREQ_MAX) freq = STEP_FREQ_MAX;

    /* Steps issued during a 1 ms tick. */
    float steps = (float)freq * CONTROL_LOOP_DT;
    float delta = steps * DEG_PER_MICROSTEP;
    plant_pos += (step_velocity > 0.0f) ? delta : -delta;
}

/* Wrap the true angle the way the MT6816 reports it (0..360). */
static float encoder_read(void)
{
    float a = fmodf(plant_pos, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

/* Run a closed-loop move, returning ms elapsed or -1 on timeout. */
static int run_move(PID_Controller_t *pid, float target, int timeout_ms, float *overshoot_out)
{
    float peak = 0.0f;
    float start = plant_pos;
    /* Overshoot means travelling PAST the target, measured in the direction of
     * travel - so it depends on which way we are moving, not on the sign of
     * the target itself. */
    float dir = (target >= start) ? 1.0f : -1.0f;
    PID_SetTarget(pid, target);

    for (int t = 0; t < timeout_ms; t++) {
        float fb = PID_UnwrapAngle(pid, encoder_read());
        float u  = PID_Update(pid, target, fb, CONTROL_LOOP_DT);
        plant_tick(u);

        /* Track how far past the target we ever got. */
        float past = (fb - target) * dir;
        if (past > peak) peak = past;

        if (fabsf(target - fb) <= POS_TOLERANCE_DEG) {
            *overshoot_out = peak;
            return t;
        }
    }
    *overshoot_out = peak;
    return -1;
}

int main(void)
{
    PID_Controller_t pid_roller, pid_blade;
    PID_Init(&pid_roller, 15.0f, 0.5f, 0.08f, -20000.0f, 20000.0f);
    PID_Init(&pid_blade,  15.0f, 0.5f, 0.08f, -20000.0f, 20000.0f);
    pid_roller.deadband = PID_DEADBAND_DEG;
    pid_blade.deadband  = PID_DEADBAND_DEG;

    /* Seed the multi-turn tracker at the homed origin. */
    PID_UnwrapAngle(&pid_roller, encoder_read());

    printf("=== Test 1: feed moves (roller) ===\n");
    float targets_mm[] = { 8.0f, 50.0f, 100.0f, 250.0f };
    float target_deg = 0.0f;
    for (unsigned i = 0; i < sizeof(targets_mm)/sizeof(targets_mm[0]); i++) {
        target_deg += targets_mm[i] * DEG_PER_MM;
        float ov = 0.0f;
        int ms = run_move(&pid_roller, target_deg, 3000 + (int)(fabsf(target_deg-plant_pos)*(3000.0f/844.0f)), &ov);
        float err_mm = (plant_pos - target_deg) / DEG_PER_MM;
        printf("  feed %6.1f mm -> %s in %4d ms | final err %+.3f mm | overshoot %.2f deg\n",
               targets_mm[i], ms < 0 ? "TIMEOUT" : "ok", ms, err_mm, ov);
        if (ms < 0) return 1;
    }

    printf("\n=== Test 2: multi-turn tracking ===\n");
    printf("  commanded total : %8.1f deg (%.1f turns)\n", target_deg, target_deg/360.0f);
    printf("  plant true pos  : %8.1f deg\n", plant_pos);
    printf("  tracker pos     : %8.1f deg\n", pid_roller.accumulated_angle_deg);
    if (fabsf(pid_roller.accumulated_angle_deg - plant_pos) > 1.0f) {
        printf("  FAIL: unwrap lost position across revolutions\n");
        return 1;
    }
    printf("  ok: unwrap tracked %.1f revolutions without losing position\n", plant_pos/360.0f);

    printf("\n=== Test 3: blade strokes ===\n");
    plant_pos = 0.0f;                      /* blade homed at its top stop */
    PID_UnwrapAngle(&pid_blade, encoder_read());
    struct { const char *name; float depth; } strokes[] = {
        { "strip head", 120.0f }, { "retract", 0.0f },
        { "strip tail", 120.0f }, { "retract", 0.0f },
        { "cut",        180.0f }, { "retract", 0.0f },
    };
    for (unsigned i = 0; i < sizeof(strokes)/sizeof(strokes[0]); i++) {
        float ov = 0.0f;
        int ms = run_move(&pid_blade, strokes[i].depth, 3000 + (int)(fabsf(strokes[i].depth-plant_pos)*(3000.0f/844.0f)), &ov);
        printf("  %-11s -> %7.1f deg | %s in %4d ms | err %+.3f deg | overshoot %.2f\n",
               strokes[i].name, strokes[i].depth, ms < 0 ? "TIMEOUT" : "ok",
               ms, plant_pos - strokes[i].depth, ov);
        if (ms < 0) return 1;
        /* Overshooting a cut depth means the blade digs into the anvil. */
        if (ov > 5.0f) { printf("  FAIL: overshoot too large for a blade\n"); return 1; }
    }

    printf("\n=== Test 4: deadband holds still at target ===\n");
    float before = plant_pos;
    for (int t = 0; t < 500; t++) {
        float fb = PID_UnwrapAngle(&pid_blade, encoder_read());
        float u  = PID_Update(&pid_blade, 0.0f, fb, CONTROL_LOOP_DT);
        plant_tick(u);
    }
    printf("  drift over 500 ms at rest: %+.4f deg\n", plant_pos - before);
    if (fabsf(plant_pos - before) > 0.5f) { printf("  FAIL: hunting at standstill\n"); return 1; }
    printf("  ok: no hunting\n");

    printf("\n=== Test 5: anti-windup on a stalled axis ===\n");
    /* Freeze the plant: the axis is jammed and cannot move. */
    PID_Controller_t stuck;
    PID_Init(&stuck, 15.0f, 0.5f, 0.08f, -20000.0f, 20000.0f);
    stuck.deadband = PID_DEADBAND_DEG;
    PID_UnwrapAngle(&stuck, 0.0f);
    for (int t = 0; t < 3000; t++) PID_Update(&stuck, 500.0f, 0.0f, CONTROL_LOOP_DT);
    float wound = stuck.integral * stuck.Ki;
    printf("  integral contribution after 3 s stalled: %.1f (limit %.1f)\n",
           wound, stuck.integral_limit);
    if (fabsf(wound) > stuck.integral_limit * 1.01f) {
        printf("  FAIL: integrator wound past its clamp\n");
        return 1;
    }
    printf("  ok: integrator stayed clamped\n");

    printf("\nALL TESTS PASSED\n");
    return 0;
}
