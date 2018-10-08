#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <math.h>
#include <assert.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

#include <lcm/lcm.h>
#include <common/getopt.h>
#include <common/time_util.h>
#include <common/serial_util.h>
#include <common/io_util.h>
#include <common/encode_bytes.h>

#include <maebot/maebot_channels.h>

#include <lcmtypes/sama5_request_t.h>
#include <lcmtypes/sama5_response_t.h>
#include <lcmtypes/sama5_sync_t.h>
#include <lcmtypes/maebot_status_t.h>
#include <lcmtypes/maebot_motor_command_t.h>
#include <lcmtypes/maebot_imu_t.h>
#include <lcmtypes/maebot_ir_rangefinder_t.h>
#include <lcmtypes/maebot_line_sensors_t.h>
#include <lcmtypes/maebot_encoders_t.h>
#include <lcmtypes/maebot_leds_command_t.h>
#include <lcmtypes/maebot_targeting_laser_command_t.h>

#define OPCODE_SYNC        1
#define OPCODE_STDOUT      2
#define OPCODE_STDERR      3
#define OPCODE_DRIVE       6
#define OPCODE_STATUS      7
#define OPCODE_ENCODERS    8
#define OPCODE_IMU         9
#define OPCODE_RANGE      10
#define OPCODE_LINE_SENSE 11
#define OPCODE_LEDS       12

#define I2C_DEVICE_PATH "/dev/i2c-3"
#define LED_ADDRESS 0x4D

typedef struct state state_t;
struct state
{
    getopt_t *gopt;
    lcm_t *lcm;

    int fd;
    FILE *serial; // serial port

    uint16_t sync_last_xid;

    // Threads
    pthread_t sama5_reader_thread;
    pthread_t poll_sync_thread;
    pthread_t poll_status_thread;
    pthread_t poll_encoders_thread;
    pthread_t poll_imu_thread;
    pthread_t poll_range_thread;
    pthread_t poll_line_sensors_thread;
    pthread_t lcm_handle_thread;
    pthread_t leds_write_thread;

    // Clock offset
    // offset = host_utime - device_utime
    // host_utime = offset + device_utime
    int64_t utime_offset;
    int64_t utime_offset_uncertainty;

    // Tracking
    uint32_t rx_count, rx_bytes;
    uint32_t tx_count, tx_bytes;
    uint32_t sync_errors, checksum_errors;

    // User parameters
    uint8_t line_sensor_leds_on;

    // User Button
    int user_button_fd;

    // RGB Leds
    int32_t leds_left, leds_right;
    int i2c_fd;
    pthread_cond_t leds_cond;
    pthread_mutex_t leds_cond_mtx;
    
    // Command to keep sending until timeout
    maebot_motor_command_t cmd;

    // Mutexes
    // do not acquire BOTH!
    pthread_mutex_t mutex_state;
    pthread_mutex_t mutex_tx;
};

// Utilities
static inline uint32_t checksum_update(uint32_t chk, uint8_t c)
{
    chk = chk + (c & 0xff);
    chk = (chk << 1) ^ (chk >> 23);

    return chk;
}

static inline uint8_t checksum_finish(uint32_t chk)
{
    return (chk & 0xff) ^ ((chk >> 8) & 0xff) ^ ((chk >> 16) & 0xff);
}

static uint8_t read_u8(FILE *f, uint32_t *chk)
{
    uint8_t b[1];
    int res = fread(b, 1, 1, f);

    // If something was read, return the the parsed value.
    if(res > 0)
    {
        if (chk)
            *chk = checksum_update(*chk, b[0]);
        return b[0];
    }

    // Otherwise indicate garbage using a 0, which should cause checksums to fail.
    return 0;
}

static void write_u8(FILE *f, uint8_t v, uint32_t *chk)
{
    ssize_t res = write(fileno(f), &v, 1);
    if (res != 1)
        printf("error\n");
//    fputc(v, f); //fwrite(&v, 1, 1, f);
    if (chk)
        *chk = checksum_update(*chk, v);
}

static uint16_t read_u16(FILE *f, uint32_t *chk)
{
    uint8_t b[2];
    int res = fread(b, 1, 2, f);

    // If something was read, return the the parsed value.
    if(res > 0)
    {
        if (chk) {
            *chk = checksum_update(*chk, b[0]);
            *chk = checksum_update(*chk, b[1]);
        }

        return (b[0]<<8) + b[1];
    }

    // Otherwise indicate garbage using a 0, which should cause checksums to fail.
    return 0;
}

static void write_u16(FILE *f, uint16_t v, uint32_t *chk)
{
    write_u8(f, (v>>8) & 0xff, chk);
    write_u8(f, (v>>0) & 0xff, chk);
}

static uint64_t read_u64(FILE *f, uint32_t *chk)
{
    uint8_t b[8];
    int res = fread(b, 1, 8, f);

    // If something was read, return the the parsed value.
    if(res > 0)
    {
        if (chk)
            for (int i = 0; i < sizeof(b); i++)
                *chk = checksum_update(*chk, b[i]);

        uint64_t v = 0;
        for (int i = 0; i < 8; i++)
            v = (v<<8) + b[i];
        return v;
    }

    // Otherwise indicate garbage using a 0, which should cause checksums to fail.
    return 0;
}

// Reads the value of the gpio value file and returns a boolean 0 or 1
static unsigned char read_gpio(int fd)
{
	char c;
	lseek (fd, 0, SEEK_SET);
	if (read (fd, &c, 1) != 1) {
        printf ("error reading gpio\r\n");
        return 0;
	}
	return (c == '1');
}

static int leds_write(int fd, int32_t left, int32_t right)
{
    uint8_t cmd[6];
    cmd[0] = (1 << 5) | ((right >> (16 + 3)) & 0x1F);
    cmd[1] = (2 << 5) | ((right >> ( 8 + 3)) & 0x1F);
    cmd[2] = (3 << 5) | ((right >> ( 0 + 3)) & 0x1F);
    cmd[3] = (4 << 5) | ((left  >> (16 + 3)) & 0x1F);
    cmd[4] = (5 << 5) | ((left  >> ( 8 + 3)) & 0x1F);
    cmd[5] = (6 << 5) | ((left  >> ( 0 + 3)) & 0x1F);

    if (ioctl(fd, I2C_SLAVE, LED_ADDRESS) < 0)
    {
        printf("Failed to set slave address: %m\n");
        return 1;
    }

    int ret = 0;
    for (int i = 0; i < 6; i++) {
        if (i2c_smbus_write_byte(fd, cmd[i]) < 0)
        {
            printf("Failed to write to I2C device: %m\n");
            ret = 1;
        }
    }

    return ret;
}

void actual_send_command_to_sama5(state_t *state, int16_t xid, int16_t opcode, uint8_t *data, int datalen)
{
    pthread_mutex_lock(&state->mutex_tx);

    uint32_t chk = 0;

    write_u8(state->serial, 0xa5, NULL);
    write_u16(state->serial, xid, &chk);
    write_u16(state->serial, opcode, &chk);
    write_u8(state->serial, datalen, &chk);
    ssize_t res = write_fully(state->fd, data, datalen);
    if (res != datalen) {
        printf("short write\n");
    }

    for (int i = 0; i < datalen; i++)
        chk = checksum_update(chk, data[i]);

    uint8_t computed_checksum = checksum_finish(chk);
    write_u8(state->serial, computed_checksum, NULL);

    state->tx_count++;
    state->tx_bytes += datalen + 7;

    pthread_mutex_unlock(&state->mutex_tx);
}

// Response Handlers
static void response_sync(state_t *state, sama5_response_t *response, int64_t recv_utime)
{
    if ((response->xid &0xffff) != state->sync_last_xid || response->len != 8) {
        printf("Rejecting sync %d %d\n", response->xid, state->sync_last_xid);
        return;
    }

    int64_t send_utime = 0;
    for (int i = 0; i < 8; i++)
        send_utime = (send_utime<<8) + response->data[i];

    int64_t utime_uncertainty = recv_utime - send_utime;
    assert(utime_uncertainty > 0);

    pthread_mutex_lock(&state->mutex_state);

    if (utime_uncertainty < state->utime_offset_uncertainty) {
        state->utime_offset_uncertainty = utime_uncertainty;
        state->utime_offset = (send_utime/2 + recv_utime/2) - response->device_utime;
    }

    if (1) {
        sama5_sync_t sync = {};
        sync.utime = utime_now();
        sync.sync_send_utime = send_utime;
        sync.sync_recv_utime = recv_utime;
        sync.sama5_utime = response->device_utime;

        sync.best_offset = state->utime_offset;
        sync.best_offset_uncertainty = state->utime_offset_uncertainty;

        sama5_sync_t_publish(state->lcm, "SAMA5_SYNC", &sync);
    }

    pthread_mutex_unlock(&state->mutex_state);
}

static void response_status(state_t *state, sama5_response_t *response)
{
    //printf("Battery status: %d%%\n", response->data[0]);
    pthread_mutex_lock(&state->mutex_state);

    maebot_status_t msg;
    msg.utime = response->utime;
    msg.battery_percent = response->data[0]; //XXX possible loss of precision. Won't actually happen though, bounded from 0 to 100
    msg.power_button = response->data[1];
    int vlen = response->len - 2; //(msg.len - 1)
    msg.firmware_version = calloc(1, vlen);
    for (int i = 0; i < vlen; i++)
    {
        msg.firmware_version[i] = response->data[i + 2];
    }
    msg.user_button = read_gpio(state->user_button_fd);

    maebot_status_t_publish(state->lcm, MAEBOT_STATUS_CHANNEL, &msg);

    free(msg.firmware_version);

    pthread_mutex_unlock(&state->mutex_state);
}

static void response_encoders(state_t *state, sama5_response_t *response)
{
    maebot_encoders_t msg;
    uint32_t msgpos = 0;
    msg.utime = response->utime;
    msg.left_ticks_total = decode_s32(response->data, &msgpos, response->len);
    msg.right_ticks_total  = decode_s32(response->data, &msgpos, response->len);
    
    pthread_mutex_lock(&state->mutex_state);
    maebot_encoders_t_publish(state->lcm, MAEBOT_ENCODERS_CHANNEL, &msg);
    pthread_mutex_unlock(&state->mutex_state);
}

static void response_imu(state_t *state, sama5_response_t *response)
{
    maebot_imu_t msg;
    uint32_t msgpos = 0;
    msg.utime = response->utime;
    msg.raw_acceleration[0] = decode_s16(response->data, &msgpos, response->len);
    msg.raw_acceleration[1] = decode_s16(response->data, &msgpos, response->len);
    msg.raw_acceleration[2] = decode_s16(response->data, &msgpos, response->len);
    msg.raw_angular_velocity[0] = decode_s16(response->data, &msgpos, response->len);
    msg.raw_angular_velocity[1] = decode_s16(response->data, &msgpos, response->len);
    msg.raw_angular_velocity[2] = decode_s16(response->data, &msgpos, response->len);
    msg.raw_orientation[0] = decode_s64(response->data, &msgpos, response->len);
    msg.raw_orientation[1] = decode_s64(response->data, &msgpos, response->len);
    msg.raw_orientation[2] = decode_s64(response->data, &msgpos, response->len);

    pthread_mutex_lock(&state->mutex_state);
    maebot_imu_t_publish(state->lcm, MAEBOT_IMU_CHANNEL, &msg);
    pthread_mutex_unlock(&state->mutex_state);
}

static void response_range(state_t *state, sama5_response_t *response)
{
    maebot_ir_rangefinder_t msg;
    uint32_t msgpos = 0;
    msg.utime = response->utime;;
    msg.range = decode_u16(response->data, &msgpos, response->len);

    pthread_mutex_lock(&state->mutex_state);
    maebot_ir_rangefinder_t_publish(state->lcm, MAEBOT_IR_RANGEFINDER_CHANNEL, &msg);
    pthread_mutex_unlock(&state->mutex_state);
}

static void response_line_sense(state_t *state, sama5_response_t *response)
{
    maebot_line_sensors_t msg;
    uint32_t msgpos = 0;
    msg.utime = response->utime;
    msg.left_sensor = decode_u16(response->data, &msgpos, response->len);
    msg.center_sensor = decode_u16(response->data, &msgpos, response->len);
    msg.right_sensor = decode_u16(response->data, &msgpos, response->len);

    pthread_mutex_lock(&state->mutex_state);
    maebot_line_sensors_t_publish(state->lcm, MAEBOT_LINE_SENSORS_CHANNEL, &msg);
    pthread_mutex_unlock(&state->mutex_state);
}

// Threads
void *sama5_reader_thread(void *_user)
{
    state_t *state = (state_t*) _user;

    while (1) {
        // read packet from SAMA5
        uint32_t chk = 0;

        int c = fgetc(state->serial);
        if (c < 0) {
            perror("fgetc");
            exit(-1);
        }

        if (c != 0xda) {
//            printf("%c", c);
            state->sync_errors++;
            continue;
        }

        int64_t capture_utime = utime_now();

        sama5_response_t response;
        response.xid    = read_u16(state->serial, &chk);
        response.opcode = read_u16(state->serial, &chk);
        response.device_utime = read_u64(state->serial, &chk);
        response.utime  = state->utime_offset + response.device_utime;
        response.len    = read_u8(state->serial, &chk) & 0xff;
        response.data   = malloc(response.len);
        int len = fread(response.data, 1, response.len, state->serial);

        // If the number of items doesn't equal the number requested, there's an error
        if(len != response.len)
        {
            ++state->sync_errors;
            goto cleanup;
        }

        for (int i = 0; i < response.len; i++)
            chk = checksum_update(chk, response.data[i]);

        uint8_t read_checksum = read_u8(state->serial, NULL);

        uint8_t computed_checksum = checksum_finish(chk);

        state->rx_bytes += response.len + 15;

        if (read_checksum != computed_checksum) {
            state->checksum_errors++;
            goto cleanup;
        }

        state->rx_count ++;

        if (response.opcode == OPCODE_SYNC) {
            // SYNC uses the actual capture time in order to avoid any
            // delays that result from reading the rest of the buffer.
            response_sync(state, &response, capture_utime);
            goto cleanup;
        }

        if (response.opcode == OPCODE_STDOUT) {
            for (int i = 0; i < response.len; i++)
                printf("%c", response.data[i]);
            goto cleanup;
        }

        if (response.opcode == OPCODE_STDERR) {
            for (int i = 0; i < response.len; i++)
                printf("%c", response.data[i]);
            goto cleanup;
        }

        if (state->utime_offset_uncertainty > 100000) {
            printf("Dropping response due to high offset error: %f ms\n", state->utime_offset_uncertainty / 1.0E3);
            goto cleanup;
        }

        ///////////////////////////////////////////////////////////////////
        // most handlers go HERE where we know we have a reasonable
        // time offset

        if (response.opcode == OPCODE_STATUS) {
            response_status(state, &response);
            goto cleanup;
        }
        if (response.opcode == OPCODE_ENCODERS) {
            response_encoders(state, &response);
            goto cleanup;
        }
        if (response.opcode == OPCODE_IMU) {
            response_imu(state, &response);
            goto cleanup;
        }
        if (response.opcode == OPCODE_RANGE) {
            response_range(state, &response);
            goto cleanup;
        }
        if (response.opcode == OPCODE_LINE_SENSE) {
            response_line_sense(state, &response);
            goto cleanup;
        }

        printf("unknown message %d\n", response.opcode);

        // publish to SAMA5_RESPONSE
        sama5_response_t_publish(state->lcm, "SAMA5_RESPONSE", &response);

      cleanup:
        free(response.data);
    }
}

void *poll_sync_thread(void *_user)
{
    state_t *state = (state_t*) _user;

    int dutime = getopt_get_int(state->gopt, "sync-interval") * 1000;
    float error_rate = atof(getopt_get_string(state->gopt, "sync-error-rate"));

    while (1) {

        if (1) {
            // generate request
            int64_t utime = utime_now();

            uint8_t buf[8] = { (utime >> 56) & 0xff,
                               (utime >> 48) & 0xff,
                               (utime >> 40) & 0xff,
                               (utime >> 32) & 0xff,
                               (utime >> 24) & 0xff,
                               (utime >> 16) & 0xff,
                               (utime >> 8 ) & 0xff,
                               (utime >> 0 ) & 0xff };

            pthread_mutex_lock(&state->mutex_state);
            state->sync_last_xid++;
            uint16_t xid = state->sync_last_xid;
            pthread_mutex_unlock(&state->mutex_state);

            actual_send_command_to_sama5(state, xid, OPCODE_SYNC, buf, 8);
        }

        usleep(dutime);

        pthread_mutex_lock(&state->mutex_state);
        int64_t duncertainty = (int64_t) dutime * error_rate;
        state->utime_offset_uncertainty += duncertainty;
        if (state->utime_offset_uncertainty < 0)
            state->utime_offset_uncertainty = INT64_MAX;
        pthread_mutex_unlock(&state->mutex_state);
    }
}


void *poll_status_thread(void *_user)
{
    state_t *state = (state_t*) _user;
    int dutime = getopt_get_int(state->gopt, "status-interval") * 1000;

    while (1)
    {
        // Send status request
        actual_send_command_to_sama5(state, 0, OPCODE_STATUS, NULL, 0);

        usleep(dutime);
    }
}

void *poll_encoders_thread(void *_user)
{
    state_t *state = (state_t*) _user;
    int dutime = getopt_get_int(state->gopt, "encoders-interval") * 1000;

    while (1)
    {
        // Send status request
        actual_send_command_to_sama5(state, 0, OPCODE_ENCODERS, NULL, 0);

        usleep(dutime);
    }
}

void *poll_imu_thread(void *_user)
{
    state_t *state = (state_t*) _user;
    int dutime = getopt_get_int(state->gopt, "imu-interval") * 1000;

    while (1)
    {
        // Send status request
        actual_send_command_to_sama5(state, 0, OPCODE_IMU, NULL, 0);

        usleep(dutime);
    }
}

void *poll_range_thread(void *_user)
{
    state_t *state = (state_t*) _user;
    int dutime = getopt_get_int(state->gopt, "range-interval") * 1000;

    while (1)
    {
        // Send status request
        actual_send_command_to_sama5(state, 0, OPCODE_RANGE, NULL, 0);

        usleep(dutime);
    }
}

void *poll_line_sensors_thread(void *_user)
{
    state_t *state = (state_t*) _user;
    int dutime = getopt_get_int(state->gopt, "line-sensors-interval") * 1000;

    while (1)
    {
        uint8_t buf[1];
        pthread_mutex_lock(&state->mutex_state);
        buf[0] = state->line_sensor_leds_on;
        pthread_mutex_unlock(&state->mutex_state);

        // Send status request
        actual_send_command_to_sama5(state, 0, OPCODE_LINE_SENSE, buf, 1);

        usleep(dutime);
    }
}

void *lcm_handle_thread(void *_user)
{
    state_t *state = (state_t*) _user;

    while (1) {
        lcm_handle(state->lcm);
    }
}

void *leds_write_thread (void *_user)
{
    state_t *state = (state_t*) _user;

    while (1) {
        pthread_mutex_lock(&state->leds_cond_mtx);
        pthread_cond_wait(&state->leds_cond, &state->leds_cond_mtx);
        pthread_mutex_unlock(&state->leds_cond_mtx);
        leds_write(state->i2c_fd, state->leds_left, state->leds_right);
    }

    return NULL;
}

// LCM Handlers
static void write_motor_command(const maebot_motor_command_t *cmd, state_t *state)
{
    // Do NOT reverse motor direction on left side
    uint16_t left = UINT16_MAX * fabs(cmd->left_motor_speed);
    uint8_t left_rev = cmd->left_motor_speed < 0;
    uint8_t left_coast = !cmd->left_motor_enabled;

    uint16_t right = UINT16_MAX * fabs(cmd->right_motor_speed);
    uint8_t right_rev = cmd->right_motor_speed < 0;
    uint8_t right_coast = !cmd->right_motor_enabled;

    uint8_t tx[8];
    uint32_t txpos = 0;

    encode_u16(tx, &txpos, left);
    encode_u8(tx, &txpos, left_rev);
    encode_u8(tx, &txpos, left_coast);
    encode_u16(tx, &txpos, right);
    encode_u8(tx, &txpos, right_rev);
    encode_u8(tx, &txpos, right_coast);
    assert(txpos == sizeof(tx));

    actual_send_command_to_sama5(state, 0, OPCODE_DRIVE, tx, txpos);
}

static void on_diff_drive(const lcm_recv_buf_t *rbuf, const char *channel,
                          const maebot_motor_command_t *_msg, void *_user)
{
    state_t *state = (state_t*) _user;
    
    pthread_mutex_lock(&state->mutex_state);
    state->cmd.utime = utime_now();
    state->cmd.left_motor_enabled = _msg->left_motor_enabled;
    state->cmd.right_motor_enabled = _msg->right_motor_enabled;
    state->cmd.left_motor_speed = _msg->left_motor_speed;
    state->cmd.right_motor_speed = _msg->right_motor_speed;
    
    write_motor_command(_msg, state);
    pthread_mutex_unlock(&state->mutex_state);
}

static void on_leds(const lcm_recv_buf_t *rbuf, const char *channel,
                          const maebot_leds_command_t *_msg, void *_user)
{
    state_t *state = (state_t*) _user;

    // Bottom
    uint8_t tx[3];
    uint32_t txpos = 0;
    encode_u8(tx, &txpos, _msg->bottom_led_left);
    encode_u8(tx, &txpos, _msg->bottom_led_middle);
    encode_u8(tx, &txpos, _msg->bottom_led_right);
    assert(txpos == sizeof(tx));

    actual_send_command_to_sama5(state, 0, OPCODE_LEDS, tx, txpos);

    // Top
    pthread_mutex_lock(&state->mutex_state);
    state->leds_left = _msg->top_rgb_led_left;
    state->leds_right = _msg->top_rgb_led_right;
    pthread_mutex_unlock(&state->mutex_state);

    pthread_mutex_lock(&state->leds_cond_mtx);
    pthread_cond_broadcast(&state->leds_cond);
    pthread_mutex_unlock(&state->leds_cond_mtx);
}

static void on_laser(const lcm_recv_buf_t *rbuf, const char *channel,
                          const maebot_targeting_laser_command_t *_msg, void *_user)
{
    if (_msg->laser_power) {
        if (system ("echo 1 > /sys/class/gpio/gpio172/value"))
            printf ("Error writing to laser pin");
    }
    else {
        if (system ("echo 0 > /sys/class/gpio/gpio172/value"))
            printf ("Error writing to laser pin");
    }
}

// Main
int main (int argc, char *argv[])
{
    setlinebuf(stdout);
    setlinebuf(stderr);

    // Handle Options
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help");
    getopt_add_string(gopt, 'd', "device", "/dev/ttyO1", "SAMA5 comms device");
    getopt_add_int(gopt, 'b', "baud", "230400", "Baud rate");
    getopt_add_int(gopt, '\0', "sync-interval", "499", "Interval between sync requests (ms)");
    getopt_add_string(gopt, '\0', "sync-error-rate", "0.001", "Maximum rate at which SAMA5/host clock is assumed to drift");
    getopt_add_int(gopt, '\0', "status-interval", "50", "Interval between status requests (ms)");
    getopt_add_int(gopt, '\0', "encoders-interval", "20", "Interval between wheel encoder requests (ms)");
    getopt_add_int(gopt, '\0', "imu-interval", "20", "Interval between imu requests (ms)");
    getopt_add_int(gopt, '\0', "range-interval", "100", "Interval between rangefinder requests (ms)");
    getopt_add_int(gopt, '\0', "line-sensors-interval", "100", "Interval between line sensor requests (ms)");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }

    // Set up state
    state_t *state = calloc(1, sizeof(state_t));
    state->sync_last_xid = utime_now() & 0xffff;
    state->utime_offset_uncertainty = INT64_MAX;
    state->gopt = gopt;

    state->fd = serial_open(getopt_get_string(gopt, "device"),
                            getopt_get_int(gopt, "baud"), 1);
    if (state->fd < 0) {
        printf("Could not open: %s\n", getopt_get_string(gopt, "device"));
        return 1;
    }
    state->serial = fdopen(state->fd, "r");

    state->lcm = lcm_create(NULL);
    if (state->lcm == NULL) {
        printf("failed to create LCM\n");
        exit(-1);
    }

    state->user_button_fd = open ("/sys/class/gpio/gpio174/value", O_RDONLY);
	if (state->user_button_fd < 0) {
		printf("Error opening file: %m\n");
	}

    pthread_mutex_init(&state->mutex_state, NULL);
    pthread_mutex_init(&state->mutex_tx, NULL);

    // RGB Leds
    state->i2c_fd = open(I2C_DEVICE_PATH, O_RDWR);
    pthread_cond_init(&state->leds_cond, NULL);
    pthread_mutex_init(&state->leds_cond_mtx, NULL);

    // Start threads
    pthread_create(&state->sama5_reader_thread, NULL, sama5_reader_thread, state);
    pthread_create(&state->poll_sync_thread, NULL, poll_sync_thread, state);
    pthread_create(&state->poll_status_thread, NULL, poll_status_thread, state);
    pthread_create(&state->poll_encoders_thread, NULL, poll_encoders_thread, state);
    pthread_create(&state->poll_imu_thread, NULL, poll_imu_thread, state);
    pthread_create(&state->poll_range_thread, NULL, poll_range_thread, state);
    if (getopt_get_int(state->gopt, "line-sensors-interval") > 0)
    {
        state->line_sensor_leds_on = 1;
        pthread_create(&state->poll_line_sensors_thread, NULL, poll_line_sensors_thread, state);
    }
    pthread_create (&state->leds_write_thread, NULL, leds_write_thread, state);
    pthread_create(&state->lcm_handle_thread, NULL, lcm_handle_thread, state);

    // Subscribe to lcm channels
    maebot_motor_command_t_subscribe(state->lcm, MAEBOT_MOTOR_COMMAND_CHANNEL, on_diff_drive, state);
    maebot_leds_command_t_subscribe(state->lcm, MAEBOT_LEDS_COMMAND_CHANNEL, on_leds, state);
    maebot_targeting_laser_command_t_subscribe(state->lcm, MAEBOT_TARGETING_LASER_COMMAND_CHANNEL, on_laser, state);

    int64_t last_sync_msg_time = utime_now();
    
    while(1) {
        
        if (utime_now() - last_sync_msg_time > 1000000)
        {
            pthread_mutex_lock(&state->mutex_state);

            printf("SAMA5 RX [ %4d pkts, %5d B ] TX [ %4d pkts, %5d B ] ",
                state->rx_count, state->rx_bytes, state->tx_count, state->tx_bytes);
            printf("sync errs: %3d, chk errs: %3d  ",
                state->sync_errors, state->checksum_errors);
            printf("timing: %.3f ms\n", state->utime_offset_uncertainty / 1.0E3);

            state->rx_count = 0;
            state->rx_bytes = 0;
            state->tx_count = 0;
            state->tx_bytes = 0;
            state->sync_errors = 0;
            state->checksum_errors = 0;
            pthread_mutex_unlock(&state->mutex_state);
            
            last_sync_msg_time = utime_now();
        }
        
        pthread_mutex_lock(&state->mutex_state);
        // Keep publishing for 1000ms to allow smoother driving via WiFi
        if (utime_now() - state->cmd.utime < 1000000)
        {
            write_motor_command(&state->cmd, state);
        }
        pthread_mutex_unlock(&state->mutex_state);

        usleep(20000);
    }

    return 0;
}
