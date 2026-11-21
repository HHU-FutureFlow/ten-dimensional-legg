#include "vt_link.h"
#include <string.h>
#include "bsp_usart.h"
#include "crc_ref.h"
#include "referee_protocol.h"
#include "remote_control.h"

#define VT_LINK_CMD_KEYMOUSE 0x0304u
#define VT_LINK_PAYLOAD_SIZE 12u
#define VT_LINK_FRAME_SIZE (LEN_HEADER + LEN_CMDID + VT_LINK_PAYLOAD_SIZE + LEN_TAIL)
#define VT_LINK_TIMEOUT_MS 150u
#define VT_LINK_RX_SIZE 128u
#define VT_LINK_BAUDRATE 115200u

static USARTInstance *vt_usart;
static VTLinkInput latest;
static uint32_t last_frame_ms;
static uint32_t last_chunk_ms;
static uint8_t have_frame;
static uint8_t frame[VT_LINK_FRAME_SIZE];
static uint16_t frame_used;

volatile uint32_t vt_link_frame_cnt;
volatile uint32_t vt_link_crc_err_cnt;
volatile uint8_t vt_link_online;

static uint16_t ReadU16LE(const uint8_t *bytes)
{
    return (uint16_t)(bytes[0] | ((uint16_t)bytes[1] << 8));
}

static void VTLinkAcceptInput(const uint8_t *data, uint32_t now)
{
    /* 收帧也检查空档，防止一拍内重连时 CMD 漏掉会话变化。 */
    if (!have_frame || now - last_frame_ms >= VT_LINK_TIMEOUT_MS)
        ++latest.session;

    latest.mouse_x = (int16_t)ReadU16LE(data);
    latest.keys = ReadU16LE(data + 8u);
    latest.stop_pending |= (latest.keys & (1u << Key_B)) != 0u;
    last_frame_ms = now;
    have_frame = 1u;
    ++vt_link_frame_cnt;
}

static void VTLinkResync(void)
{
    /* 坏帧可能已吞入下一帧开头，保留内部 SOF 起始的后缀。 */
    uint16_t start = 1u;
    while (start < frame_used && frame[start] != REFEREE_SOF)
        ++start;
    frame_used -= start;
    memmove(frame, frame + start, frame_used);
}

static void VTLinkPushByte(uint8_t value, uint32_t now)
{
    if (frame_used == 0u && value != REFEREE_SOF)
        return;

    frame[frame_used++] = value;
    while (frame_used >= LEN_HEADER)
    {
        if (ReadU16LE(frame + DATA_LENGTH) != VT_LINK_PAYLOAD_SIZE)
        {
            VTLinkResync();
            continue;
        }
        if (!Verify_CRC8_Check_Sum(frame, LEN_HEADER))
        {
            ++vt_link_crc_err_cnt;
            VTLinkResync();
            continue;
        }
        if (frame_used < CMD_ID_Offset + LEN_CMDID)
            return;
        if (ReadU16LE(frame + CMD_ID_Offset) != VT_LINK_CMD_KEYMOUSE)
        {
            VTLinkResync();
            continue;
        }
        if (frame_used < VT_LINK_FRAME_SIZE)
            return;

        if (Verify_CRC16_Check_Sum(frame, VT_LINK_FRAME_SIZE))
        {
            VTLinkAcceptInput(frame + DATA_Offset, now);
            frame_used = 0u;
            return;
        }
        ++vt_link_crc_err_cnt;
        VTLinkResync();
    }
}

static void VTLinkRxCallback(void)
{
    if (vt_usart == NULL)
        return; /* 注册会启动 DMA，首拍可能早于句柄赋值。 */

    uint32_t now = HAL_GetTick();
    if (now - last_chunk_ms >= VT_LINK_TIMEOUT_MS)
        frame_used = 0u;
    last_chunk_ms = now;

    for (uint16_t i = 0u; i < vt_usart->recv_size; ++i)
        VTLinkPushByte(vt_usart->recv_buff[i], now);
}

void VTLinkInit(UART_HandleTypeDef *handle)
{
    if (vt_usart != NULL)
        return;

    /* USART1 由 CubeMX 初始化，准备接图传或串口桥时再设置链路速率。 */
    if (USARTConfigure(handle, VT_LINK_BAUDRATE, UART_WORDLENGTH_8B,
                       UART_PARITY_NONE, UART_STOPBITS_1) != HAL_OK)
    {
        Error_Handler();
        return;
    }

    USART_Init_Config_s config = {
        .recv_buff_size = VT_LINK_RX_SIZE,
        .usart_handle = handle,
        .module_callback = VTLinkRxCallback,
    };
    vt_usart = USARTRegister(&config);
}

VTLinkInput VTLinkRead(void)
{
    /* USART IRQ 写入，CMD 任务整份读取并消费急停。临界区只做快照。 */
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    VTLinkInput input = latest;
    input.age_ms = HAL_GetTick() - last_frame_ms;
    have_frame = have_frame && input.age_ms < VT_LINK_TIMEOUT_MS;
    input.online = have_frame;
    latest.stop_pending = 0u;
    __set_PRIMASK(irq_state);

    vt_link_online = input.online;
    return input;
}
