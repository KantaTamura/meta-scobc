#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>

#ifndef TIMEOUT
#define TIMEOUT 10
#endif
#ifndef KICK_DELAY
#define KICK_DELAY 1
#endif

static volatile int running = 1;
static void handle_signal(int sig) { running = 0; }

int main(int argc, char **argv)
{
    const char *dev = "/dev/versal-fpd-watchdog";
    int fd, ret, timeout = TIMEOUT, readback = 0;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    fd = open(dev, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", dev, strerror(errno));
        return 1;
    }

    ret = ioctl(fd, WDIOC_SETTIMEOUT, &timeout);
    if (!ret)
        printf("Watchdog timeout set to %d sec\n", timeout);
    ioctl(fd, WDIOC_GETTIMEOUT, &readback);
    printf("WDIOC_GETTIMEOUT returned %d sec\n", readback);

    while (running) {
        sleep(KICK_DELAY);
        ret = ioctl(fd, WDIOC_KEEPALIVE, 0);
        if (ret)
            fprintf(stderr, "KEEPALIVE failed: %s\n", strerror(errno));
    }

    printf("Stopping watchdog daemon.\n");

    int opt = WDIOS_DISABLECARD;
    ioctl(fd, WDIOC_SETOPTIONS, &opt);

    close(fd);

    return 0;
}
