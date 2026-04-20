#include "sysTime.h"
#include "can.h"
#include "utils.h"

void can_head_init()
{
  CANAll_Init(1, 2, 0x00000009);   // Init CAN port 1, IRQ 2, 1 Mbps @ PCLK 30 MHz
  CANAll_SetErrIRQ(3);
}

uint8_t set_reg_value_b(uint8_t dest, uint8_t reg, uint8_t val)
{
  CANALL_MSG buf;
  struct can_frame* tx;
  uint8_t cnt = 0;

  tx = (struct can_frame*) &buf.DatA;
  buf.Frame = 0x00040000;   // 4 bytes
  buf.MsgID = dest;
  tx->write = 1;
  tx->size = 1;
  tx->reply = 0;
  tx->sendack = 1;
  tx->reg = reg;
  tx->snd = LOCAL_ADDR;
  tx->data[0] = val;

  while (1) {
    if (CANAll_PushMessage(1, &buf)) {
      while (!CANAll_PullMessage(1, &buf)) {
        cnt++;
        if (cnt==0x00) return 0;
        pause(CAN_TIMEOUT);
      }
      if (buf.MsgID==(LOCAL_ADDR | 0x100)) break;
    } else pause(CAN_TIMEOUT);
  }
  return 1;
}

uint8_t set_reg_value_w(uint8_t dest, uint8_t reg, uint16_t val)
{
  CANALL_MSG buf;
  struct can_frame* tx;
  uint8_t cnt = 0;

  tx = (struct can_frame*) &buf.DatA;
  buf.Frame = 0x00050000;   // 5 bytes
  buf.MsgID = dest;
  tx->write = 1;
  tx->size = 2;
  tx->reply = 0;
  tx->sendack = 1;
  tx->reg = reg;
  tx->snd = LOCAL_ADDR;
  unaligned_write_16(tx->data, val);

  while (1) {
    if (CANAll_PushMessage(1, &buf)) {
      while (!CANAll_PullMessage(1, &buf)) {
        cnt++;
        if (cnt==0x00) return 0;
        pause(CAN_TIMEOUT);
      }
      if (buf.MsgID==(LOCAL_ADDR | 0x100)) break;
    } else pause(CAN_TIMEOUT);
  }
  return 1;
}

uint8_t set_reg_value_dw(uint8_t dest, uint8_t reg, uint32_t val)
{
  CANALL_MSG buf;
  struct can_frame* tx;
  uint8_t cnt = 0;

  tx = (struct can_frame*) &buf.DatA;
  buf.Frame = 0x00070000;   // 7 bytes
  buf.MsgID = dest;
  tx->write = 1;
  tx->size = 3;
  tx->reply = 0;
  tx->sendack = 1;
  tx->reg = reg;
  tx->snd = LOCAL_ADDR;
  unaligned_write_32(tx->data, val);

  while (1) {
    if (CANAll_PushMessage(1, &buf)) {
      while (!CANAll_PullMessage(1, &buf)) {
        cnt++;
        if (cnt==0x00) return 0;
        pause(CAN_TIMEOUT);
      }
      if (buf.MsgID==(LOCAL_ADDR | 0x100)) break;
    } else pause(CAN_TIMEOUT);
  }
  return 1;
}

uint8_t get_reg_value_b(uint8_t dest, uint8_t reg)
{
  CANALL_MSG buf;
  struct can_frame *tx;
  struct can_frame_b *rx;
  uint8_t cnt = 0;

  tx = (struct can_frame*) &buf.DatA;
  rx = (struct can_frame_b *)tx;
  buf.Frame = 0x00030000;   // 3 bytes
  buf.MsgID = dest;
  tx->write = 0;
  tx->size = 1;
  tx->reply = 0;
  tx->reg = reg;
  tx->snd = LOCAL_ADDR;

  while (!CANAll_PushMessage(1, &buf)) {
    cnt++;
    if (cnt == 0x00) return 0xFF;
    pause(CAN_TIMEOUT);
  }
  
  cnt = 0;

  do {
    while (!CANAll_PullMessage(1, &buf)) {
      cnt++;
      if (cnt==0x00) return 0xFF;
      pause(CAN_TIMEOUT);
    }
  } while (!rx->reply);
  return rx->data[0];
}

uint16_t get_reg_value_w(uint8_t dest, uint8_t reg)
{
  CANALL_MSG buf;
  struct can_frame *tx;
  struct can_frame_b *rx;
  uint8_t cnt = 0;

  tx = (struct can_frame*) &buf.DatA;
  rx = (struct can_frame_b*) tx;
  buf.Frame = 0x00030000;   // 3 bytes
  buf.MsgID = dest;
  tx->write = 0;
  tx->size = 2;
  tx->reply = 0;
  tx->reg = reg;
  tx->snd = LOCAL_ADDR;


  while (!CANAll_PushMessage(1, &buf)) {
    cnt++;
    if (cnt == 0x00) return 0xFFFF;
    pause(CAN_TIMEOUT);
  }

  do {
    while (!CANAll_PullMessage(1, &buf)) {
      cnt++;
      if (cnt==0x00) return 0xFFFF;
      pause(CAN_TIMEOUT);
    }
  } while (!rx->reply);
  return unaligned_read_16(rx->data);
}

uint32_t get_reg_value_dw(uint8_t dest, uint8_t reg)
{
  CANALL_MSG buf;
  struct can_frame *tx;
  struct can_frame_b *rx;
  uint8_t cnt = 0;

  tx = (struct can_frame*) &buf.DatA;
  rx = (struct can_frame_b*) tx;
  buf.Frame = 0x00030000;   // 3 bytes
  buf.MsgID = dest;
  tx->write = 0;
  tx->size = 3;
  tx->reply = 0;
  tx->reg = reg;
  tx->snd = LOCAL_ADDR;

  while (!CANAll_PushMessage(1, &buf)) {
    cnt++;
    if (cnt == 0x00) return 0xFFFFFFFF;
    pause(CAN_TIMEOUT);
  }

  do {
    while (!CANAll_PullMessage(1, &buf)) {
      cnt++;
      if (cnt==0x00) return 0xFFFFFFFF;
      pause(CAN_TIMEOUT);
    }
  } while (!rx->reply);
  return unaligned_read_32(rx->data);
}
