/*
 * asahi-brightnessd — ambient-light auto-brightness for Apple Silicon Macs
 * running Asahi Linux.
 *
 * Reads lux from the aop-sensors-als IIO device and drives both
 * /sys/class/backlight/apple-panel-bl and /sys/class/leds/kbd_backlight.
 *
 * External writes (e.g. brightnessctl invoked by the compositor's
 * XF86MonBrightnessUp/Down bindings) are detected by comparing the observed
 * value against what we last wrote. When a divergence is seen, the daemon
 * yields control of that channel until ambient light shifts significantly.
 *
 * Curve constants are adapted from juicecultus/asahi-auto-brightness (MIT).
 */

#define _POSIX_C_SOURCE 200809L

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

/* ------------------------------------------------------------------ */
/* Paths                                                              */
/* ------------------------------------------------------------------ */
#define IIO_BASE    "/sys/bus/iio/devices"
#define ALS_NAME    "aop-sensors-als"
#define ALS_ATTR    "in_illuminance_input"
#define SCREEN_DIR  "/sys/class/backlight/apple-panel-bl"
#define KBD_DIR     "/sys/class/leds/kbd_backlight"
#define AC_ONLINE   "/sys/class/power_supply/macsmc-ac/online"

/* ------------------------------------------------------------------ */
/* Tuning                                                             */
/* ------------------------------------------------------------------ */
#define POLL_NSEC          33000000L   /* ~30 Hz update rate          */
#define SMOOTH_SHIFT       4           /* alpha = 1/(2^4) = 0.0625    */
                                       /* ≈510 ms time constant       */
                                       /* (~90% convergence in 1.2 s) */

#define LUX_RESUME_PCT     75       /* % delta to resume after override */
#define LUX_RESUME_ABS     5        /* absolute lux delta floor         */

#define SCREEN_FLOOR_PCT   2        /* never let screen go fully dark   */
#define KBD_FLOOR_PCT      0

/* When plugged into AC, scale the screen curve up by this many percentage
 * points (target_pct += AC_SCREEN_BOOST), clamped to 100. The keyboard
 * curve is unaffected — visibility in the dark is power-source neutral. */
#define AC_SCREEN_BOOST    30
#define AC_RECHECK_TICKS   30       /* poll AC status ~1 Hz */

/* lux → percentage curves (linearly interpolated) */
typedef struct { int lux; int pct; } curve_point;

static const curve_point SCREEN_CURVE[] = {
    {   0,   2},
    {   1,   3},
    {   5,   6},
    {  20,  12},
    {  50,  24},
    { 100,  40},
    { 200,  60},
    { 400,  80},
    { 700,  92},
    {1000, 100},
};
#define SCREEN_CURVE_LEN (sizeof(SCREEN_CURVE)/sizeof(SCREEN_CURVE[0]))

/* Inverse: bright keyboard in the dark, off in daylight. */
static const curve_point KBD_CURVE[] = {
    {   0,  75},
    {   5,  50},
    {  20,  25},
    {  50,   0},
    {1000,   0},
};
#define KBD_CURVE_LEN (sizeof(KBD_CURVE)/sizeof(KBD_CURVE[0]))

/* ------------------------------------------------------------------ */
/* Globals                                                            */
/* ------------------------------------------------------------------ */
static volatile sig_atomic_t g_terminate = 0;

static char als_path[PATH_MAX];
static char screen_path[PATH_MAX];
static char kbd_path[PATH_MAX];
static int  screen_max;
static int  kbd_max;

typedef struct {
    int  last_written;       /* -1 = nothing written yet */
    bool override_active;
    int  override_baseline_lux;
} channel_state;

static channel_state screen_state = { -1, false, 0 };
static channel_state kbd_state    = { -1, false, 0 };

static bool g_on_ac = false;
static int  g_ac_recheck = 0;

/* ------------------------------------------------------------------ */
/* Signals                                                            */
/* ------------------------------------------------------------------ */
static void on_signal(int sig) {
    (void)sig;
    g_terminate = 1;
}

/* ------------------------------------------------------------------ */
/* sysfs helpers                                                      */
/* ------------------------------------------------------------------ */
static int read_int(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int val = -1;
    if (fscanf(f, "%d", &val) != 1) val = -1;
    fclose(f);
    return val;
}

static int write_int(const char *path, int val) {
    FILE *f = fopen(path, "w");
    if (!f) return -1;
    int rc = fprintf(f, "%d\n", val);
    fclose(f);
    return rc < 0 ? -1 : 0;
}

/* Find the iio:deviceN whose `name` attribute matches ALS_NAME. */
static int locate_als(char *out, size_t outsz) {
    DIR *d = opendir(IIO_BASE);
    if (!d) return -1;
    struct dirent *de;
    char namepath[PATH_MAX];
    while ((de = readdir(d)) != NULL) {
        if (strncmp(de->d_name, "iio:device", 10) != 0) continue;
        snprintf(namepath, sizeof(namepath), "%s/%s/name", IIO_BASE, de->d_name);
        FILE *f = fopen(namepath, "r");
        if (!f) continue;
        char name[64] = {0};
        if (fgets(name, sizeof(name), f)) {
            char *nl = strchr(name, '\n');
            if (nl) *nl = '\0';
            if (strcmp(name, ALS_NAME) == 0) {
                snprintf(out, outsz, "%s/%s/%s", IIO_BASE, de->d_name, ALS_ATTR);
                fclose(f);
                closedir(d);
                return 0;
            }
        }
        fclose(f);
    }
    closedir(d);
    return -1;
}

/* ------------------------------------------------------------------ */
/* Curve evaluation                                                   */
/* ------------------------------------------------------------------ */
static int interpolate(const curve_point *curve, size_t n, int lux) {
    if (lux <= curve[0].lux)     return curve[0].pct;
    if (lux >= curve[n-1].lux)   return curve[n-1].pct;
    for (size_t i = 1; i < n; i++) {
        if (lux <= curve[i].lux) {
            int x0 = curve[i-1].lux, x1 = curve[i].lux;
            int y0 = curve[i-1].pct, y1 = curve[i].pct;
            return y0 + (y1 - y0) * (lux - x0) / (x1 - x0);
        }
    }
    return curve[n-1].pct;
}

static int pct_to_units(int pct, int max_units, int floor_pct) {
    if (pct < floor_pct) pct = floor_pct;
    if (pct > 100)       pct = 100;
    return pct * max_units / 100;
}

/* Re-read AC status every AC_RECHECK_TICKS ticks. */
static void refresh_ac_status(void) {
    if (g_ac_recheck > 0) { g_ac_recheck--; return; }
    g_ac_recheck = AC_RECHECK_TICKS;
    int online = read_int(AC_ONLINE);
    g_on_ac = (online == 1);
}

/* Exponential smoothing: move alpha (= 1/2^SMOOTH_SHIFT) of the remaining
 * gap toward target each tick. Large deltas move quickly; small deltas
 * fall below the perceptual threshold automatically. The fallback ±1 step
 * guarantees we don't stall on integer truncation in the tail. */
static int step_toward(int current, int target, int max_units) {
    (void)max_units;
    int delta = target - current;
    if (delta == 0) return current;
    int step = delta >> SMOOTH_SHIFT;
    if (delta > 0 && step < 1)  step = 1;
    if (delta < 0 && step > -1) step = -1;
    return current + step;
}

/* ------------------------------------------------------------------ */
/* Override logic                                                     */
/* ------------------------------------------------------------------ */
static bool lux_shifted(int lux, int baseline) {
    int delta = lux > baseline ? lux - baseline : baseline - lux;
    if (delta < LUX_RESUME_ABS) return false;
    if (baseline == 0)          return true;
    return (delta * 100 / baseline) >= LUX_RESUME_PCT;
}

/* Update one channel: detect override, compute target, step. */
static void tick_channel(channel_state *st, const char *path,
                         const curve_point *curve, size_t curve_len,
                         int max_units, int floor_pct, int boost_pct, int lux)
{
    int observed = read_int(path);
    if (observed < 0) return;

    if (st->last_written >= 0 && observed != st->last_written) {
        st->override_active = true;
        st->override_baseline_lux = lux;
    }
    if (st->override_active && lux_shifted(lux, st->override_baseline_lux)) {
        st->override_active = false;
    }

    if (st->override_active) {
        st->last_written = observed;
        return;
    }

    int pct    = interpolate(curve, curve_len, lux) + boost_pct;
    int target = pct_to_units(pct, max_units, floor_pct);
    int next   = step_toward(observed, target, max_units);

    if (next != observed) {
        if (write_int(path, next) == 0) {
            int readback = read_int(path);
            st->last_written = readback >= 0 ? readback : next;
        }
    } else {
        st->last_written = observed;
    }
}

/* ------------------------------------------------------------------ */
/* Main loop                                                          */
/* ------------------------------------------------------------------ */
static int run(void) {
    char screen_max_path[PATH_MAX], kbd_max_path[PATH_MAX];
    snprintf(screen_path,     sizeof(screen_path),     "%s/brightness",     SCREEN_DIR);
    snprintf(screen_max_path, sizeof(screen_max_path), "%s/max_brightness", SCREEN_DIR);
    snprintf(kbd_path,        sizeof(kbd_path),        "%s/brightness",     KBD_DIR);
    snprintf(kbd_max_path,    sizeof(kbd_max_path),    "%s/max_brightness", KBD_DIR);

    screen_max = read_int(screen_max_path);
    kbd_max    = read_int(kbd_max_path);
    if (screen_max <= 0 || kbd_max <= 0) {
        fprintf(stderr, "asahi-brightnessd: could not read max_brightness "
                "(screen=%d kbd=%d)\n", screen_max, kbd_max);
        return 1;
    }
    if (locate_als(als_path, sizeof(als_path)) != 0) {
        fprintf(stderr, "asahi-brightnessd: could not locate ALS "
                "(no iio:device* with name=%s)\n", ALS_NAME);
        return 1;
    }
    fprintf(stderr,
            "asahi-brightnessd: als=%s screen_max=%d kbd_max=%d\n",
            als_path, screen_max, kbd_max);

    struct timespec poll_interval = { .tv_sec = 0, .tv_nsec = POLL_NSEC };
    while (!g_terminate) {
        refresh_ac_status();
        int lux = read_int(als_path);
        if (lux >= 0) {
            int screen_boost = g_on_ac ? AC_SCREEN_BOOST : 0;
            tick_channel(&screen_state, screen_path,
                         SCREEN_CURVE, SCREEN_CURVE_LEN,
                         screen_max, SCREEN_FLOOR_PCT, screen_boost, lux);
            tick_channel(&kbd_state, kbd_path,
                         KBD_CURVE, KBD_CURVE_LEN,
                         kbd_max, KBD_FLOOR_PCT, 0, lux);
        }
        nanosleep(&poll_interval, NULL);
    }
    return 0;
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    struct sigaction sa = { 0 };
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    return run();
}
