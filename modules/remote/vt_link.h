#ifndef VT_LINK_H
#define VT_LINK_H

#include <stdint.h>
#include "usart.h"

typedef struct
{
    uint32_t session;
    uint32_t age_ms;
    uint16_t keys;
    int16_t mouse_x;
    uint8_t online;
    uint8_t stop_pending;
} VTLinkInput;

void VTLinkInit(UART_HandleTypeDef *handle);
/* RobotCMDTask 单消费者读取。急停事件在复制快照时取走。 */
VTLinkInput VTLinkRead(void);

extern volatile uint32_t vt_link_frame_cnt;
extern volatile uint32_t vt_link_crc_err_cnt;
extern volatile uint8_t vt_link_online;

#endif
