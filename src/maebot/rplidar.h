#ifndef _RPLIDAR_H
#define _RPLIDAR_H

#include <unistd.h>
#include <stdint.h>

#include "lcmtypes/rplidar_laser_t.h"

#define TIMEOUT_MS 50

// Request packet format. Checksum is XOR of all bytes.
// | START FLAG    | COMMAND | PAYLOAD LEN | PAYLOAD     | CHECKSUM |
// ---------------------------------------------------------------
// | 1 byte (0xA5) | 1 byte  | 1 byte      | < 256 bytes | 1 byte   |

// Response descriptor format
// | START FLAG 0  | START FLAG 1  | DATA RESP. LEN | SEND MODE | DATA TYPE |
// --------------------------------------------------------------------------
// | 1 byte (0xA5) | 1 byte (0x5A) | 30 bits        | 2 bits    | 1 byte    |

// Response format varies w/ request type

#define MAGIC_0 0xA5
#define MAGIC_1 0x5A

// 0x2 and 0x3 are reserved for future use
#define SEND_MODE_SINGLE_RESPONSE 0x0
#define SEND_MODE_MULTI_RESPONSE  0x1

#define REQUEST_STOP        0x25
#define REQUEST_RESET       0x40
#define REQUEST_SCAN        0x20
#define REQUEST_FORCE_SCAN  0x21
#define REQUEST_GET_INFO    0x50
#define REQUEST_GET_HEALTH  0x52

#define HEALTH_GOOD         0x0
#define HEALTH_WARN         0x1
#define HEALTH_ERROR        0x2

typedef struct rp_descriptor
{
    uint32_t len;
    uint8_t send_mode;
    uint8_t data_type;
} rp_descriptor_t;

/* Exit current device state */
void rp_lidar_stop(int dev);

/* Reboot device */
void rp_lidar_reset(int dev);

/* Scan when ready */
void rp_lidar_scan(int dev, lcm_t *lcm, const char *channel);

/* Force a scan regardless of rotation speed */
void rp_lidar_force_scan(int dev, lcm_t *lcm, const char *channel);

/* Check the information for the device (serial #, etc) and print it to term */
void rp_lidar_check_info(int dev);

/* Check the health of the device. Prints health status and returns status code. */
int rp_lidar_check_health(int dev);


#endif
