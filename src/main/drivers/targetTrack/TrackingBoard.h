#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "common/time.h"

#include "io/serial.h"

#ifdef USE_TARGET_TRACK

#define BUFF_SIZE 250

/*
0x00：可见光1；
0x01：可见光2；
0x02：红外1；
*/
#define TRACKING_BOARD_IMAGE_CHANNLE_LIGHT_1 0x00
#define TRACKING_BOARD_IMAGE_CHANNLE_LIGHT_2 0x01
#define TRACKING_BOARD_IMAGE_CHANNLE_INFRARED_1 0x02

#define TASK_TRACKING_RATE 500       // default update rate of GPS task
#define TASK_TRACKING_RATE_FAST 1000 // update rate of GPS task while Rx buffer is not empty

#define TRACKING_TELEM_STATUS_FRAME_S1_ID 0X11

void TrackingBoardInit(void);

void TrackingBoardUpdate(timeUs_t currentTimeUs);

#pragma pack(push, 1)

typedef struct
{
    uint8_t head1; // 0x55
    uint8_t head2; // 0xaa
    uint8_t len;
    uint8_t num;
    uint8_t c1_id;
    uint8_t c1_cmd1;
    uint8_t c1_cmd2;
    uint8_t c1_params[10];
    uint8_t c2_id;
    uint8_t c2_cmd1;
    uint8_t c2_cmd2;
    uint8_t c2_params[10];
} Tracking_board_packet_send_head;

typedef struct
{
    uint16_t date;
    uint32_t time;
    uint8_t reserve_0[27];
    uint32_t image_frame_id;
    uint8_t reserve_1[6];
} Tracking_board_packet_send_data;

typedef struct
{
    Tracking_board_packet_send_head head;
    Tracking_board_packet_send_data data;
    uint8_t crc;
} Tracking_board_packet_send;

typedef struct
{
    uint8_t head1;
    uint8_t head2;
    uint8_t len;
    uint8_t indentify_id;
    uint8_t system_mode;
} Tracking_board_packet_head_s1;

typedef struct
{
    uint8_t reserve_0[37];
    uint8_t track_status;
    uint16_t target_center_x;
    uint16_t target_center_y; 
    uint16_t target_width;   
    uint16_t target_height;   
    uint16_t track_area_size_width;
    uint16_t track_area_size_height;
    uint16_t max_target_width;
    uint16_t max_target_height;
    uint16_t target_confidence_threshold;   
    uint8_t reserve_1[2];
    uint8_t primary_sensor_chan;
    uint8_t primary_sensor_enhan;
    uint16_t primary_sensor_fov_mag;
    uint16_t primary_sensor_fovh;
    uint16_t primary_sensor_fovv;
    uint8_t reserve_2[2];
    uint8_t encoding_info;
    uint8_t picture_count;
    uint16_t storage;
    uint16_t bitstream;
    uint8_t fps;
    uint8_t gop;
    int8_t temperature;
} frame_s1_t;

typedef struct
{
    Tracking_board_packet_head_s1 head;
    frame_s1_t s1;
} Tracking_board_packet_s1;


#pragma pack(pop)

int Tracking_cmd_send(Tracking_board_packet_send data);

uint8_t sum_low_8bits(const uint8_t *byte_array, uint8_t length);

bool check_bit(uint8_t value, uint8_t position);

int Tracking_cmd_send_return(void);

int Tracking_cmd_send_image_tracking(uint16_t azimuth_coordinate, uint16_t pitch_coordinate, uint16_t target_id);

int Tracking_cmd_send_image_optional_tracking(uint16_t azimuth_coordinate, uint16_t pitch_coordinate, uint16_t width, uint16_t height);

int Tracking_cmd_send_auto_tracking(void);

int Tracking_cmd_send_pre_tracking(uint16_t azimuth_coordinate, uint16_t pitch_coordinate, uint16_t width, uint16_t height);

int Tracking_cmd_send_cmd_self_check(void);

int Tracking_cmd_send_priority_mode(uint16_t status, uint16_t type, uint16_t priority);

int Tracking_cmd_send_adsorption_area_size(uint16_t area_width, uint16_t area_height);

int Tracking_cmd_send_adsorption_param(uint16_t template_width, uint16_t template_height, uint16_t threshold);

int Tracking_cmd_send_image_channle(uint16_t ch, uint16_t status);

int Tracking_cmd_send_template_size(uint16_t size);

bool get_target_width_height(uint16_t *width, uint16_t *height);

bool get_target_center_xy(uint16_t *target_center_x, uint16_t *target_center_y);

#endif // USE_TARGET_TRACK