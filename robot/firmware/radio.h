#ifndef __RADIO_H
#define __RADIO_H

#include <stdint.h>

/// 8-bit register read
#define ROP_READ_8   0
/// 16-bit register read
#define ROP_READ_16  1
/// 32-bit register read
#define ROP_READ_32  2
/// multibyte register read
#define ROP_READ_MB  3
/// 8-bit register write
#define ROP_WRITE_8  4
/// 16-bit register write
#define ROP_WRITE_16 5
/// 32-bit register write
#define ROP_WRITE_32 6
/// multibyte register write
#define ROP_WRITE_MB 7

/// maximal size (in bytes) of a multibyte register
#define MAX_MB_SIZE  29

/// Structure used for the register callbacks
typedef union {
	uint8_t byte;      /// register contents as 8-bit
	uint16_t word;     /// register contents as 16-bit
	uint32_t dword;    /// register contents as 32-bit

  /// structure for multibyte registers
	struct {
		uint8_t size;    /// size of the data
		uint8_t data[MAX_MB_SIZE];  /// the actual data
	} multibyte;

	uint8_t bytes[30]; /// raw access to the buffer (internal use)
} RadioData;

/** \brief Type definition for register read/write callback functions
  * \param operation Operation. Operations are currently: ROP_READ_8 = 8-bit read,
  * ROP_READ_16 = 16-bit read, ROP_READ_32 = 32-bit read, ROP_READ_MB = multibyte
  * read, ROP_WRITE_8 = 8-bit write, ROP_WRITE_16 = 16-bit write,  ROP_WRITE_32 =
  * 32-bit write, ROP_WRITE_MB = multibyte write.
  * \param address Address of the register to write to or read from.
  * \param radio_data A pointer to the data to be read/written. Data can be
  * accessed through different fields of the RadioData union, depending on the
  * operation type. Note that when handling a ROP_READ_MB operation, the user must
  * also set radio_data->multibyte.size.
  * \return TRUE if the operation was handled by the callback, FALSE otherwise.
  */
typedef int8_t (*register_callback_t)
               (uint8_t operation, uint8_t address, RadioData* radio_data);

/// \brief Function called by the UART ISR when receiving data
/// \warning This function should never be called by the user.
void process_UART_in(void);

/// Initialises the radio system (UART and synchronization with the PIC that
/// controls the transceiver).
void radio_init(void);

/// Adds a register callback function.
void radio_add_reg_callback(register_callback_t addr);

/// Removes a register callback function.
void radio_del_reg_callback(register_callback_t addr);

#endif
