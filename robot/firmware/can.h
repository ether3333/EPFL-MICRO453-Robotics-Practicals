#ifndef __CAN_H
#define __CAN_H

/**
 * \file   can.h
 * \brief  Routines for accessing the remote elements on the CAN bus using the specific protocol
 * \author Alessandro Crespi
 * \date   July 2010
 */

#include <stdint.h>
#include "LPC_CANAll.h"

#define CAN_TIMEOUT 500     // timer resolution is 100 ns
#define LOCAL_ADDR  0

/// CAN packets sent from head
struct can_frame {
  int write:1;         //< 1 if writing a register
  int reply:1;         //< Always 0
  int size:2;          //< Size of the data (1 = 8-bit, 2 = 16-bit, 3 = 32-bit)
  int sendack:1;       //< 1 if remote node is requested to send an ACK
  uint8_t snd;         //< Address of the sender
  uint8_t reg;         //< Register number
  uint8_t data[4];     //< Data (for write operations)
} __attribute__ ((packed));

/// CAN answers sent from elements
struct can_frame_b {
  int nothing:1;       //< Unused
  int reply:1;         //< Always 1
  int ack:1;           //< 1 if this is an ACK packet, 0 if this is a register read result
  int size:2;          //< Size of the data (0 = no data, 1 = 8-bit, 2 = 16-bit, 3 = 32-bit)
  uint8_t data[4];     //< Actual data for register read results
} __attribute__ ((packed));

/// Initialises the CAN peripherals/library
void can_head_init(void);

/** \brief  Writes a 8-bit register on a remote element
 *  \return 0 on failure, 1 on success
 */
uint8_t set_reg_value_b(uint8_t dest, uint8_t reg, uint8_t val);

/** \brief  Writes a 16-bit register on a remote element
 *  \return 0 on failure, 1 on success
 */
uint8_t set_reg_value_w(uint8_t dest, uint8_t reg, uint16_t val);

/** \brief  Writes a 32-bit register on a remote element
 *  \return 0 on failure, 1 on success
 */
uint8_t set_reg_value_dw(uint8_t dest, uint8_t reg, uint32_t val);

/** \brief  Reads a 8-bit register from a remote element
 *  \return The read value on success, or 0xFF in case of failure
 */
uint8_t get_reg_value_b(uint8_t dest, uint8_t reg);

/** \brief  Reads a 16-bit register from a remote element
 *  \return The read value on success, or 0xFFFF in case of failure
 */
uint16_t get_reg_value_w(uint8_t dest, uint8_t reg);

/** \brief  Reads a 32-bit register from a remote element
 *  \return The read value on success, or 0xFFFFFFFF in case of failure
 */
uint32_t get_reg_value_dw(uint8_t dest, uint8_t reg);

#endif
