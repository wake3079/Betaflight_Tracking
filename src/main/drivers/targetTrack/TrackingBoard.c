/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <ctype.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_TARGET_TRACK
#include "cli/cli.h"
#include "io/serial.h"
#include "drivers/targetTrack/TrackingBoard.h"

#include "common/maths.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/rx.h"
#include "rx/rx.h"

#include "fc/rc_controls.h"

#include "osd/osd.h"
#include "drivers/time.h"
#include "TrackingBoard.h"

#define FRAME_HEADER1 0xAA
#define FRAME_HEADER2 0x55

#define IS_HI(X) (rcData[X] > 1750)
#define IS_LO(X) (rcData[X] < 1250)
#define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)

union
{
    Tracking_board_packet_s1 _packet_s1;
    uint8_t buff[BUFF_SIZE + 8];
} Tracking_board_packet_union;

static serialPort_t *TrackingPort;

frame_s1_t frame_s1;

enum
{
    TRACKING_BOARD_FRAME_TYPE_S11 = 0x00,

    TRACKING_BOARD_FRAME_TYPE_UNKNOWN = 0xff,
};
uint8_t current_frame_type = TRACKING_BOARD_FRAME_TYPE_UNKNOWN;
uint16_t total_frame_length = 0;
uint8_t buff_length;

void TrackingBoardInit(void)
{
#if defined(USE_TARGET_TRACK)
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LIDAR_TF);

    if (!portConfig)
    {
        return;
    }

    portMode_e mode = MODE_RXTX;
    portOptions_e options = SERIAL_NOT_INVERTED;
    // no callback - buffer will be consumed in gpsUpdate()
    TrackingPort = openSerialPort(portConfig->identifier, FUNCTION_LIDAR_TF, NULL, NULL, 115200, mode, options);
    if (!TrackingPort)
    {
        return;
    }
#endif
}
void TrackingBoardUpdate(timeUs_t currentTimeUs)
{
    if (TrackingPort)
    {
        while (serialRxBytesWaiting(TrackingPort))
        {
            currentTimeUs = currentTimeUs;

            // Add every byte to _buffer, when enough bytes are received, convert data to values
            uint8_t temp = serialRead(TrackingPort);

            Tracking_board_packet_union.buff[buff_length] = temp;
            buff_length++;

            // head check
            if (buff_length == 1)
            {
                if (FRAME_HEADER1 != temp)
                {
                    buff_length = 0;
                    continue;
                }
            }

            if (buff_length == 2)
            {
                if (FRAME_HEADER2 != temp)
                {
                    buff_length = 0;
                    continue;
                }
            }

            // id check
            if (buff_length == 5)
            {
                if (Tracking_board_packet_union.buff[3] == TRACKING_TELEM_STATUS_FRAME_S1_ID)
                {
                    // probably frame s11
                    current_frame_type = TRACKING_BOARD_FRAME_TYPE_S11;
                    total_frame_length = Tracking_board_packet_union._packet_s1.head.len;
                }
                else
                {
                    buff_length = 0;
                    current_frame_type = TRACKING_BOARD_FRAME_TYPE_UNKNOWN;
                    continue;
                }
            }

            // full frame check
            if (buff_length != total_frame_length)
            {
                continue;
            }

            // sum check
            uint8_t sum = 0;
            for (uint8_t i = 0; i < buff_length - 1; i++)
            {
                sum += Tracking_board_packet_union.buff[i];
            }

            if (sum != Tracking_board_packet_union.buff[buff_length - 1])
            {
                buff_length = 0;
                current_frame_type = TRACKING_BOARD_FRAME_TYPE_UNKNOWN;
                continue;
            }

            switch (current_frame_type)
            {
            case TRACKING_BOARD_FRAME_TYPE_S11:
            {
                frame_s1 = Tracking_board_packet_union._packet_s1.s1;
                break;
            }
            default:
            {
                break;
            }
            }

            buff_length = 0;
        }
    }
}

int Tracking_cmd_send(Tracking_board_packet_send data)
{
    data.head.head1 = FRAME_HEADER2;
    data.head.head2 = FRAME_HEADER1;
    data.head.len = sizeof(Tracking_board_packet_send);
    data.head.num = 0;
    data.data.date = 0;
    data.data.time = 0;

    data.crc = sum_low_8bits((uint8_t *)&data, (sizeof(Tracking_board_packet_send) - 1));
    if (!TrackingPort)
    {
        return -1;
    }

    serialWriteBuf(TrackingPort, (uint8_t *)&data, sizeof(Tracking_board_packet_send));
    return 0;
}

uint8_t sum_low_8bits(const uint8_t *byte_array, uint8_t length)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += byte_array[i];
    }

    return (sum & 0xff);
}

bool check_bit(uint8_t value, uint8_t position)
{
    return (value & (1 << position)) != 0;
}

int Tracking_cmd_send_return(void)
{
    Tracking_board_packet_send send;
    memset(&send, 0, sizeof(send));
    send.head.c1_id = 0x01;
    send.head.c1_cmd1 = 0x03;
    Tracking_cmd_send(send);
    return 0;
}

int Tracking_cmd_send_image_tracking(uint16_t azimuth_coordinate, uint16_t pitch_coordinate, uint16_t target_id)
{
    Tracking_board_packet_send send;
    memset(&send, 0, sizeof(send));
    send.head.c1_id = 0x01;
    send.head.c1_cmd1 = 0x10;
    memcpy(send.head.c1_params, (uint8_t *)&azimuth_coordinate, sizeof(azimuth_coordinate));
    memcpy(&send.head.c1_params[2], (uint8_t *)&pitch_coordinate, sizeof(pitch_coordinate));
    memcpy(&send.head.c1_params[4], (uint8_t *)&target_id, sizeof(target_id));
    return Tracking_cmd_send(send);
}

int Tracking_cmd_send_image_optional_tracking(uint16_t azimuth_coordinate, uint16_t pitch_coordinate, uint16_t width, uint16_t height)
{
    Tracking_board_packet_send send;
    memset(&send, 0, sizeof(send));
    send.head.c1_id = 0x01;
    send.head.c1_cmd1 = 0x11;
    memcpy(send.head.c1_params, (uint8_t *)&azimuth_coordinate, sizeof(azimuth_coordinate));
    memcpy(&send.head.c1_params[2], (uint8_t *)&pitch_coordinate, sizeof(pitch_coordinate));
    memcpy(&send.head.c1_params[4], (uint8_t *)&width, sizeof(width));
    memcpy(&send.head.c1_params[6], (uint8_t *)&height, sizeof(height));
    return Tracking_cmd_send(send);
}

int Tracking_cmd_send_auto_tracking(void)
{
    Tracking_board_packet_send send;
    memset(&send, 0, sizeof(send));
    send.head.c1_id = 0x01;
    send.head.c1_cmd1 = 0x12;
    return Tracking_cmd_send(send);
}

int Tracking_cmd_send_pre_tracking(uint16_t azimuth_coordinate, uint16_t pitch_coordinate, uint16_t width, uint16_t height)
{
    Tracking_board_packet_send send;
    memset(&send, 0, sizeof(send));
    send.head.c1_id = 0x01;
    send.head.c1_cmd1 = 0x13;
    memcpy(send.head.c1_params, (uint8_t *)&azimuth_coordinate, sizeof(azimuth_coordinate));
    memcpy(&send.head.c1_params[2], (uint8_t *)&pitch_coordinate, sizeof(pitch_coordinate));
    memcpy(&send.head.c1_params[4], (uint8_t *)&width, sizeof(width));
    memcpy(&send.head.c1_params[6], (uint8_t *)&height, sizeof(height));
    return Tracking_cmd_send(send);
}

int Tracking_cmd_send_cmd_self_check(void)
{
    Tracking_board_packet_send send;
    memset(&send, 0, sizeof(send));
    send.head.c1_id = 0x01;
    send.head.c1_cmd1 = 0x20;
    return Tracking_cmd_send(send);
}

int Tracking_cmd_send_priority_mode(uint16_t status, uint16_t type, uint16_t priority)
{
    Tracking_board_packet_send send;
    memset(&send, 0, sizeof(send));
    send.head.c2_id = 0x41;
    send.head.c2_cmd1 = 0x30;
    memcpy(send.head.c2_params, (uint8_t *)&status, sizeof(status));
    memcpy(&send.head.c2_params[2], (uint8_t *)&type, sizeof(type));
    memcpy(&send.head.c2_params[4], (uint8_t *)&priority, sizeof(priority));
    return Tracking_cmd_send(send);
}

int Tracking_cmd_send_adsorption_area_size(uint16_t area_width, uint16_t area_height)
{
    Tracking_board_packet_send send;
    memset(&send, 0, sizeof(send));
    send.head.c2_id = 0x41;
    send.head.c2_cmd1 = 0x31;
    memcpy(send.head.c2_params, (uint8_t *)&area_width, sizeof(area_width));
    memcpy(&send.head.c2_params[2], (uint8_t *)&area_height, sizeof(area_height));
    return Tracking_cmd_send(send);
}

int Tracking_cmd_send_adsorption_param(uint16_t template_width, uint16_t template_height, uint16_t threshold)
{
    Tracking_board_packet_send send;
    memset(&send, 0, sizeof(send));
    send.head.c2_id = 0x41;
    send.head.c2_cmd1 = 0x32;
    memcpy(send.head.c2_params, (uint8_t *)&template_width, sizeof(template_width));
    memcpy(&send.head.c2_params[2], (uint8_t *)&template_height, sizeof(template_height));
    memcpy(&send.head.c2_params[4], (uint8_t *)&threshold, sizeof(threshold));
    return Tracking_cmd_send(send);
}

int Tracking_cmd_send_image_channle(uint16_t ch, uint16_t status)
{
    Tracking_board_packet_send send;
    memset(&send, 0, sizeof(send));
    send.head.c2_id = 0x51;
    send.head.c2_cmd1 = 0x01;
    memcpy(send.head.c2_params, (uint8_t *)&ch, sizeof(ch));
    memcpy(&send.head.c2_params[2], (uint8_t *)&status, sizeof(status));
    return Tracking_cmd_send(send);
}

int Tracking_cmd_send_template_size(uint16_t size)
{
    Tracking_board_packet_send send;
    memset(&send, 0, sizeof(send));
    send.head.c2_id = 0x51;
    send.head.c2_cmd1 = 0x10;
    memcpy(send.head.c2_params, (uint8_t *)&size, sizeof(size));
    return Tracking_cmd_send(send);
}

bool get_target_width_height(uint16_t *width, uint16_t *height)
{
    if (check_bit(frame_s1.track_status, 1))
    {
        *width = frame_s1.target_width;
        *height = frame_s1.target_height;
        return true;
    }

    return false;
}

bool get_target_center_xy(uint16_t *target_center_x, uint16_t *target_center_y)
{
    if (check_bit(frame_s1.track_status, 1))
    {
        *target_center_x = frame_s1.target_center_x;
        *target_center_y = frame_s1.target_center_y;

        return true;
    }

    return false;
}

#endif // USE_TARGET_TRACK
