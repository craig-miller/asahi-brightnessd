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
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
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
#define DRM_BASE    "/sys/class/drm"
#define DPMS_ATTR   "dpms"
#define EDP_PREFIX  "eDP-"

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

/* When a Noctalia shell is running, fire a brightness-suppress IPC just
 * before each screen-channel sysfs write so its OSD stays quiet for
 * ALS-driven adjustments. 100 ms covers a few poll ticks of headroom; if
 * Noctalia isn't reachable the call fails silently and the daemon keeps
 * driving the backlight. */
#define NOCTALIA_SUPPRESS_MS   100
#define NOCTALIA_SOCK_PREFIX   "noctalia-"
#define NOCTALIA_USER_ROOT     "/run/user"

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
static char dpms_path[PATH_MAX] = "";   /* "" if no eDP connector found */
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

/* Cached Noctalia IPC socket path. Empty string means "unknown / not running".
 * Re-discovered on the next call after a connect failure. */
static char g_noctalia_sock[PATH_MAX] = "";

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

/* Find the first /sys/class/drm/cardN-eDP-* connector's dpms attribute.
 * Used to zero the kbd backlight when the laptop panel blanks. External
 * monitors (HDMI, DP) are deliberately ignored — they don't affect kbd
 * visibility on the integrated keyboard.
 *
 * Returns 0 on success, -1 if no eDP connector exists (uncommon — e.g.
 * desktop-style replicas without an internal panel). Non-fatal; the
 * caller treats absence as "DPMS always on". */
static int locate_edp_dpms(char *out, size_t outsz) {
    DIR *d = opendir(DRM_BASE);
    if (!d) return -1;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (strncmp(de->d_name, "card", 4) != 0) continue;
        /* Skip the bare card[0-9]* nodes; we want card[0-9]+-eDP-N. */
        const char *dash = strchr(de->d_name + 4, '-');
        if (!dash) continue;
        if (strncmp(dash + 1, EDP_PREFIX, sizeof(EDP_PREFIX) - 1) != 0)
            continue;
        int n = snprintf(out, outsz, "%s/%s/%s",
                         DRM_BASE, de->d_name, DPMS_ATTR);
        if (n > 0 && (size_t)n < outsz) {
            closedir(d);
            return 0;
        }
    }
    closedir(d);
    return -1;
}

/* True if the located DPMS attribute reads anything other than "On".
 * On read failure or empty path, returns false ("assume on" — safer:
 * an unreachable DPMS file shouldn't force the kbd backlight off and
 * leave the user typing in the dark). */
static bool dpms_is_off(const char *path) {
    if (path[0] == '\0') return false;
    FILE *f = fopen(path, "r");
    if (!f) return false;
    char buf[16] = {0};
    if (!fgets(buf, sizeof(buf), f)) {
        fclose(f);
        return false;
    }
    fclose(f);
    char *nl = strchr(buf, '\n');
    if (nl) *nl = '\0';
    return strcmp(buf, "On") != 0;
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
/* Noctalia IPC                                                       */
/* ------------------------------------------------------------------ */

/* Find the first /run/user/<uid>/noctalia-*.sock and copy the absolute path
 * into out. Returns 0 on success. */
static int find_noctalia_socket(char *out, size_t outsz) {
    DIR *d = opendir(NOCTALIA_USER_ROOT);
    if (!d) return -1;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        char user_dir[PATH_MAX];
        int n = snprintf(user_dir, sizeof(user_dir), "%s/%s",
                         NOCTALIA_USER_ROOT, de->d_name);
        if (n < 0 || (size_t)n >= sizeof(user_dir)) continue;
        DIR *ud = opendir(user_dir);
        if (!ud) continue;
        struct dirent *ude;
        while ((ude = readdir(ud)) != NULL) {
            if (strncmp(ude->d_name, NOCTALIA_SOCK_PREFIX,
                        sizeof(NOCTALIA_SOCK_PREFIX) - 1) != 0) continue;
            size_t len = strlen(ude->d_name);
            if (len < 5 || strcmp(ude->d_name + len - 5, ".sock") != 0) continue;
            n = snprintf(out, outsz, "%s/%s", user_dir, ude->d_name);
            closedir(ud);
            closedir(d);
            return (n > 0 && (size_t)n < outsz) ? 0 : -1;
        }
        closedir(ud);
    }
    closedir(d);
    return -1;
}

/* Fire-and-forget: ask Noctalia to suppress its brightness OSD for `ms`.
 * Blocks briefly (≤50 ms) on the socket round-trip so the suppression
 * window is established before the sysfs write fires inotify. Failures
 * (no socket, connect refused, etc.) are silently ignored. */
static void noctalia_suppress_osd(int ms) {
    if (g_noctalia_sock[0] == '\0') {
        if (find_noctalia_socket(g_noctalia_sock,
                                 sizeof(g_noctalia_sock)) != 0) {
            return;
        }
    }
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return;
    int flags = fcntl(fd, F_GETFD);
    if (flags >= 0) (void)fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
    struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 };
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    /* Unix domain sun_path is 108 bytes on Linux but g_noctalia_sock is
     * PATH_MAX (4096). A path that doesn't fit can't reach the socket,
     * and a silent strncpy truncation would point connect() at an
     * unrelated path. Bail explicitly and invalidate the cache. */
    size_t sock_len = strlen(g_noctalia_sock);
    if (sock_len >= sizeof(addr.sun_path)) {
        g_noctalia_sock[0] = '\0';
        close(fd);
        return;
    }
    memcpy(addr.sun_path, g_noctalia_sock, sock_len);  /* sun_path zero-initialised */
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        /* Stale path (compositor restart/exit). Invalidate so the next
         * call rediscovers — or, if noctalia is gone, gives up cheaply. */
        g_noctalia_sock[0] = '\0';
        close(fd);
        return;
    }
    char cmd[64];
    int n = snprintf(cmd, sizeof(cmd), "brightness-suppress %d\n", ms);
    if (n > 0) {
        ssize_t w = write(fd, cmd, (size_t)n);
        (void)w;
        char buf[32];
        ssize_t r = read(fd, buf, sizeof(buf));  /* wait for response */
        (void)r;
    }
    close(fd);
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

/* Update one channel: detect override, compute target, step.
 * When `announce_to_noctalia` is true, fire a brightness-suppress IPC
 * immediately before each sysfs write so Noctalia's OSD stays quiet for
 * ALS-driven adjustments. */
static void tick_channel(channel_state *st, const char *path,
                         const curve_point *curve, size_t curve_len,
                         int max_units, int floor_pct, int boost_pct, int lux,
                         bool announce_to_noctalia)
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
        if (announce_to_noctalia) {
            noctalia_suppress_osd(NOCTALIA_SUPPRESS_MS);
        }
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
    /* DPMS lookup is best-effort. Without it the daemon still drives
     * both channels from lux; it just won't zero kbd when the panel
     * blanks. */
    (void)locate_edp_dpms(dpms_path, sizeof(dpms_path));
    fprintf(stderr,
            "asahi-brightnessd: als=%s screen_max=%d kbd_max=%d dpms=%s\n",
            als_path, screen_max, kbd_max,
            dpms_path[0] ? dpms_path : "(none)");

    struct timespec poll_interval = { .tv_sec = 0, .tv_nsec = POLL_NSEC };
    while (!g_terminate) {
        refresh_ac_status();
        int lux = read_int(als_path);
        if (lux >= 0) {
            if (dpms_is_off(dpms_path)) {
                /* Panel is blanked. Force kbd off and skip the screen
                 * channel — writes to apple-panel-bl while the panel is
                 * dark are wasted, and the kernel preserves the value
                 * for when DPMS resumes. Clearing override_active means
                 * a manual kbd setting from before idle doesn't fight
                 * the forced zero on the way down. */
                int observed = read_int(kbd_path);
                if (observed > 0 && write_int(kbd_path, 0) == 0) {
                    kbd_state.last_written = 0;
                    kbd_state.override_active = false;
                }
            } else {
                int screen_boost = g_on_ac ? AC_SCREEN_BOOST : 0;
                tick_channel(&screen_state, screen_path,
                             SCREEN_CURVE, SCREEN_CURVE_LEN,
                             screen_max, SCREEN_FLOOR_PCT, screen_boost, lux,
                             true);
                tick_channel(&kbd_state, kbd_path,
                             KBD_CURVE, KBD_CURVE_LEN,
                             kbd_max, KBD_FLOOR_PCT, 0, lux,
                             false);
            }
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
