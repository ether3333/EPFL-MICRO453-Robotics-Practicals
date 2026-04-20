#ifndef __REMREGS_H
#define __REMREGS_H

#include <windows.h>

#if defined(__GNUC__)
#include <stdint.h>
#else
typedef unsigned __int8 uint8_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int32 uint32_t;
#endif

const uint8_t ACK = 6;
const uint8_t NAK = 15;

class CRemoteRegs {

public:

  /// Constructor
  CRemoteRegs();
  
  /// Destructor
  ~CRemoteRegs();
  
  /// Opens the radio interface on the specified port with the given baudrate
  bool open(const char* portname, int spd);
  
  /// Synchronizes the communication between the PC and the radio interface
  bool sync();
  
  /// Closes the connection to the radio interface
  void close();

  /** \brief Reads a 8-bit register
    * \param addr The address of the register (0 - 1023)
    * \param res Reference to a variable that will contain the read value
    * \return true if the operation suceeded, false if not
    */
  bool get_reg_b(const uint16_t addr, uint8_t& res);

  /** \brief Reads a 16-bit register
    * \param addr The address of the register (0 - 1023)
    * \param res Reference to a variable that will contain the read value
    * \return true if the operation suceeded, false if not
    */
  bool get_reg_w(const uint16_t addr, uint16_t& res);

  /** \brief Reads a 32-bit register
    * \param addr The address of the register (0 - 1023)
    * \param res Reference to a variable that will contain the read value
    * \return true if the operation suceeded, false if not
    */
  bool get_reg_dw(const uint16_t addr, uint32_t& res);

  /** \brief Reads a multibyte register
    * \param addr The address of the register (0 - 1023)
    * \param data A pointer to the output buffer (at least 29 bytes long)
    * \param len Reference to a variable that will contain the length of the returned data
    * \return true if the operation suceeded, false if not
    */
  bool get_reg_mb(const uint16_t addr, uint8_t* data, uint8_t& len);

  /** \brief Reads a 8-bit register
    * \param addr The address of the register (0 - 1023)
    * \return The read value (0x00 - 0xff) or 0xff on failure
    */
  uint8_t get_reg_b(const uint16_t addr);

  /** \brief Reads a 16-bit register
    * \param addr The address of the register (0 - 1023)
    * \return The read value (0x0000 - 0xffff) or 0xffff on failure
    */
  uint16_t get_reg_w(const uint16_t addr);

  /** \brief Reads a 32-bit register
    * \param addr The address of the register (0 - 1023)
    * \return The read value (0x00000000 - 0xffffffff) or 0xffffffff on failure
    */
  uint32_t get_reg_dw(const uint16_t addr);

  /** \brief Writes a 8-bit register
    * \param addr The address of the register (0 - 1023)
    * \param val The value to write to the register
    * \return true if the operation suceeded, false if not
    */
  bool set_reg_b(const uint16_t addr, const uint8_t val);

  /** \brief Writes a 16-bit register
    * \param addr The address of the register (0 - 1023)
    * \param val The value to write to the register
    * \return true if the operation suceeded, false if not
    */
  bool set_reg_w(const uint16_t addr, const uint16_t val);

  /** \brief Writes a 32-bit register
    * \param addr The address of the register (0 - 1023)
    * \param val The value to write to the register
    * \return true if the operation suceeded, false if not
    */
  bool set_reg_dw(const uint16_t addr, const uint32_t val);
  
  /** \brief Writes a multibyte register
    * \param addr The address of the register (0 - 1023)
    * \param data Pointer to the data to write to the register
    * \param len Length of the data to write (0 - 29 bytes)
    * \return true if the operation suceeded, false if not
    */
  bool set_reg_mb(const uint16_t addr, const uint8_t* data, const uint8_t len);

private:

  struct wl_debug {
    uint8_t op;
    uint16_t addr;
    uint8_t rlen;
    const void* rdata;
    uint8_t alen;
    const void* adata;
    bool result;
  };

  /// Internal function for register operations
  bool reg_op(const uint8_t op, const uint16_t addr, const uint8_t *data, const int len, const bool lock = false);
  
  /// Displays debug trace for a register operation
  void debug_dump(const wl_debug* dbg);

  /// Handle to the serial port
  HANDLE hSer;
  
  /// Mutex to avoid simulataneous accesses to the serial port
  CRITICAL_SECTION mutex;

};

#endif
