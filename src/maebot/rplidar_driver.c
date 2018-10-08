#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <signal.h>

#include "rplidar.h"
#include <maebot/maebot_channels.h>

#include "common/getopt.h"
#include "common/serial.h"

#include "lcmtypes/rplidar_laser_t.h"

#define VERBOSE 1

/* Set up serial communication with the RPLIDAR and broadcast LCM on
 * an appropriate channel */
typedef struct state
{
    getopt_t *gopt;

    // Device file descriptor
    int dev;

    // Scan thread stuff
    pthread_t scan_thread;
    pthread_t stop_thread;

    // LCM stuff
    const char *channel;
    lcm_t *lcm;
} state_t;
static state_t *state;

static void sig_handler(int signo)
{
    rp_lidar_stop(state->dev);
}

static void* scan_loop(void *args)
{
    state_t *state = (state_t*)args;

    printf("Beginning scans...\n");
    if (system("echo 1 > /sys/class/gpio/gpio122/value")) {
      printf("Error starting rplidar motor\n");
    }
    // This loops forever, barring an error
    rp_lidar_scan(state->dev, state->lcm, state->channel);
    printf("Terminating rplidar...\n");
    if (system("echo 0 > /sys/class/gpio/gpio122/value")) {
      printf("Error Stopping rplidar motor\n");
    }

    return NULL;
}

int main(int argc, char* argv[])
{
    state = (state_t*) calloc(1, sizeof(state_t));

    state->gopt = getopt_create();
    getopt_add_bool(state->gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_string(state->gopt, 'd', "device", "/dev/ttyO0", "Serial device");
    getopt_add_int(state->gopt, 'b', "baud", "115200", "Baud rate");
    getopt_add_string(state->gopt, 'c', "channel", RPLIDAR_LASER_CHANNEL, "LCM channel name");

    if (!getopt_parse(state->gopt, argc, argv, 1) || getopt_get_bool(state->gopt, "help")) {
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(state->gopt);
        return 0;
    }

    signal(SIGTERM, sig_handler);
    signal(SIGINT, sig_handler);

    // Open device
    const char *name = getopt_get_string(state->gopt, "device");
    int baud = getopt_get_int(state->gopt, "baud");
    state->dev = serial_open(name, baud, 1);

    if (state->dev == -1) {
        printf("ERR: Could not open device at %s\n", name);
        return -1;
    }

    state->lcm = lcm_create(NULL);
    state->channel = getopt_get_string(state->gopt, "channel");

    // Check device health
    if (rp_lidar_check_health(state->dev) != HEALTH_GOOD)
        return -2;

    // Check device info
    if (VERBOSE)
        rp_lidar_check_info(state->dev);

    // Begin scanning
    pthread_create(&state->scan_thread, NULL, scan_loop, state);
    pthread_join(state->scan_thread, NULL);

    lcm_destroy(state->lcm);
    getopt_destroy(state->gopt);
    serial_close(state->dev);
    free(state);

    return 0;
}
