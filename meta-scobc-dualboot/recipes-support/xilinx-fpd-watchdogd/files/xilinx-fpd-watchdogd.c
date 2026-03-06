#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>

#ifndef TIMEOUT
#define TIMEOUT 30
#endif

#ifndef CLOSED_WINDOW_PERCENT
#define CLOSED_WINDOW_PERCENT 50
#endif

// ref. linux-xlnx/drivers/watchdog/xilinx_wwdt.c
#define DEFAULT_CLOSED_WINDOW_PERCENT 50

/*
 * Fraction of the open window used to schedule the watchdog kick.
 * The kick is sent at:
 *
 *     closed_window + open_window * (NUM / DEN)
 *
 * Using 3/8 places the kick safely inside the open window (slightly
 * after it begins) while keeping enough margin before the timeout.
 */
#define OPEN_WINDOW_FRACTION_NUM 3
#define OPEN_WINDOW_FRACTION_DEN 8

static volatile sig_atomic_t running = 1;

static void handle_signal(int sig)
{
    (void)sig;
    running = 0;
}

static int sleep_ms_interruptible(long ms)
{
    struct timespec req, rem;

    req.tv_sec = ms / 1000;
    req.tv_nsec = (ms % 1000) * 1000000L;

    while (nanosleep(&req, &rem) < 0) {
        if (errno == EINTR) {
            if (!running)
                return 0;
            req = rem;
            continue;
        }
        perror("nanosleep");
        return -1;
    }

    return 0;
}

static void log_mono(const char *msg)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
        printf("[%lld.%03ld] %s\n",
               (long long)ts.tv_sec,
               ts.tv_nsec / 1000000L,
               msg);
    } else {
        printf("%s\n", msg);
    }
}

int main(void)
{
    const char *dev = "/dev/watchdog0";
    int fd;
    int ret;
    int timeout = 0;
    int closed_window_percent = CLOSED_WINDOW_PERCENT;

    long total_ms;
    long closed_ms;
    long open_ms;
    long kick_delay_ms;

    struct sigaction sa;

    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_signal;
    sigemptyset(&sa.sa_mask);

    if (sigaction(SIGINT, &sa, NULL) < 0) {
        perror("sigaction(SIGINT)");
        return 1;
    }

    if (sigaction(SIGTERM, &sa, NULL) < 0) {
        perror("sigaction(SIGTERM)");
        return 1;
    }

    fd = open(dev, O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", dev, strerror(errno));
        return 1;
    }

    ret = ioctl(fd, WDIOC_GETTIMEOUT, &timeout);
    if (ret < 0 || timeout <= 0) {
        fprintf(stderr, "WDIOC_GETTIMEOUT failed, fallback to default timeout %d sec: %s\n",
                TIMEOUT, strerror(errno));
        timeout = TIMEOUT;
    }

    if (closed_window_percent <= 0 || closed_window_percent >= 100)
        closed_window_percent = DEFAULT_CLOSED_WINDOW_PERCENT;

    total_ms  = (long)timeout * 1000L;
    closed_ms = total_ms * closed_window_percent / 100L;
    open_ms   = total_ms - closed_ms;

    if (open_ms <= 0) {
        fprintf(stderr, "Invalid open window: timeout=%d sec closed=%d%%\n",
                timeout, closed_window_percent);
        close(fd);
        return 1;
    }

    kick_delay_ms = closed_ms + (open_ms * OPEN_WINDOW_FRACTION_NUM) / OPEN_WINDOW_FRACTION_DEN;

    /*
     * Avoid kick timings too close to the window boundaries.
     * Scheduling jitter or timing inaccuracies could otherwise cause
     * the kick to fall into the closed window.
     */
    if (kick_delay_ms <= closed_ms)
        kick_delay_ms = closed_ms + 100;
    if (kick_delay_ms >= total_ms)
        kick_delay_ms = total_ms - 100;

    printf("Watchdog timeout           : %d sec\n", timeout);
    printf("Closed window percent      : %d %%\n", closed_window_percent);
    printf("Closed window              : %ld ms\n", closed_ms);
    printf("Open window                : %ld ms\n", open_ms);
    printf("Kick delay from (re)start  : %ld ms\n", kick_delay_ms);

    log_mono("Watchdog keepalive loop started.");

    while (running) {
        ret = sleep_ms_interruptible(kick_delay_ms);
        if (ret < 0) {
            close(fd);
            return 1;
        }

        if (!running)
            break;

        ret = ioctl(fd, WDIOC_KEEPALIVE, 0);
        if (ret < 0) {
            fprintf(stderr, "WDIOC_KEEPALIVE failed: %s\n", strerror(errno));
            close(fd);
            return 1;
        }

        log_mono("Sent keepalive to watchdog.");
    }

    log_mono("Stopping keepalive loop.");

    /*
     * xilinx_wwdt is nowayout and has no stop op.
     * close(fd) does not stop the watchdog hardware; it only ends this
     * userspace keepalive loop.
     */
    close(fd);
    return 0;
}
