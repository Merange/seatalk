#include "seatalk_transport_layer.h"
#include "seatalk_hardware_layer.h"
#include "seatalk_protocol.h"
#include "boat_status.h"
#include "seatalk_datagram.h"
#include "seatalk_command.h"
#include "logger.h"

#define IDLE_BITS 10

// receive data
int rx_bit_number = -1;
char rx_byte = 0;
int rx_command_bit = 0;
char receive_buffer[256];
int receive_buffer_position = 0;
int receive_datagram_bytes_remaining = 255;

// transmit data
int tx_bit_number = -1;
char tx_byte = 0;
int tx_command_bit = 1;
unsigned char transmit_buffer[256];
int transmit_buffer_position = 0;
int transmit_datagram_bytes_remaining = 0;

int data_enqueued = 0;
BUS_STATE bus_state = BUS_STATE_IDLE;

void set_bus_state(BUS_STATE new_state) {
  bus_state = new_state;
}

// confirm no collision; ready next byte
// note: pattern is send-byte, receive-byte. Next transmit cannot take place
//   before receive because of event timing.
int seatalk_byte_transmitted(void) {
#ifdef DEBUG
  LOG("seatalk_byte_transmitted: %x\n", tx_byte);
#endif
  transmit_buffer_position++;
  if (--transmit_datagram_bytes_remaining == 0) {
    return 1;
  } else {
    return 0;
  }
}

// collision detected; cancel current datagram.
void cancel_datagram(void) {
  transmit_buffer_position += transmit_datagram_bytes_remaining;
  transmit_datagram_bytes_remaining = 0;
}

int seatalk_byte_received(char data_byte, int command_bit) {
  BUS_STATE current_state = bus_state;
  set_bus_state(BUS_STATE_WAIT_FOR_IDLE);
#ifdef DEBUG
  LOG("byte received: %x command_bit: %d bytes_remaining: %d", data_byte, command_bit, receive_datagram_bytes_remaining);
#endif
  if (current_state == BUS_STATE_TRANSMITTING) {
    // collision; cancel current datagram and idle the bus for 10 cycles
    if (data_byte != tx_byte) {
#ifdef DEBUF
      LOG("data byte (%2x) != tx_byte (%2x)", data_byte, tx_byte);
#endif
      cancel_datagram();
      return 1;
    }
  }
  if (command_bit) { // parity (command) bit is high: start new command
#ifdef DEBUG
    LOG("command bit received; emptying buffer\n");
#endif
    receive_buffer_position = 0;
    receive_datagram_bytes_remaining = 255;
  }
  receive_buffer[receive_buffer_position++] = data_byte;
  if (receive_datagram_bytes_remaining == 254) { // second character
    receive_datagram_bytes_remaining = get_datagram_length(data_byte); // datagram length
  } else if (!(receive_datagram_bytes_remaining)) {
    handle_seatalk_datagram(receive_buffer);
  } else {
    receive_datagram_bytes_remaining -= 1;
  }
  if (current_state == BUS_STATE_TRANSMITTING) {
    if (seatalk_byte_transmitted()) { // confirms no collision; ready next byte
      return 1;
    } else {
      return 0;
    }
  } else {
    return 1;
  }
}

int initiate_seatalk_receive_character(void) {
  BUS_STATE current_state = bus_state;
  // rx_bit_number is -1 if we are not currently receiving a byte
  if (rx_bit_number != -1) { // are we mid-character?
    return 0;                // exit and tell receiver to ignore this 1 to 0 transition
  }
#ifdef DEBUG
  if (tx_bit_number != 0) {
   LOG("starting new character after tx bit %d (should be 0)\n", tx_bit_number);
  }
#endif
  if ((current_state == BUS_STATE_IDLE) || (current_state == BUS_STATE_WAIT_FOR_IDLE)) {
    set_bus_state(BUS_STATE_RECEIVING);
  }
  return 1;
}

int seatalk_receive_bit() {
  int bit_value = get_seatalk_hardware_bit_value();
  if (++rx_bit_number == 0) {
#ifdef DEBUG
    LOG("rx beginning new character\n");
#endif
    rx_byte = 0;
    rx_command_bit = 0;
  }
  if (rx_bit_number <= 7) {
#ifdef DEBUG
    if (bit_value != ((tx_byte >> rx_bit_number) & 0x1)) {
      LOG("received %d but expected %d\n", bit_value, !bit_value);
    }
#endif
    rx_byte |= (bit_value << rx_bit_number);
    return 1;
  } else if (rx_bit_number == 8) {
    // we have now received all 8 data bits
    // check command (parity) bit status
    rx_command_bit = bit_value;
    return 1;
  } else { // by process of elimination this must be bit 9, stop bit
    rx_bit_number = -1;
    if (!bit_value) {
      // stop bit needs to be high. If it is not then we are misaligned
      // with the actual bytes and must discard this byte. We should
      // eventually "walk" around to correct start bit/stop bit alignment.
//      LOG("%x/%d received but stop_bit low", rx_byte, rx_command_bit);
      set_bus_state(BUS_STATE_IDLE);
      return 0;
    }
    if (seatalk_byte_received(rx_byte, rx_command_bit)) {
      initiate_seatalk_hardware_transmitter(IDLE_BITS);
    }
    return 0;
  }
}

void initiate_seatalk_transmitter(void) {
  if (bus_state == BUS_STATE_IDLE) {
    initiate_seatalk_hardware_transmitter(1);
  }
}

int seatalk_can_transmit(void) {
  int will_transmit = transmit_datagram_bytes_remaining || data_enqueued;
  SENSOR_ID pending_sensor_id;
  if (!will_transmit) {
    will_transmit = seatalk_command_pending() || seatalk_sensor_pending(&pending_sensor_id);
  }
  return will_transmit;
}

// get next datagram byte
int seatalk_byte_to_send(char *data_byte, int *command_bit) {
  if (transmit_datagram_bytes_remaining == 0) {
    transmit_datagram_bytes_remaining = get_pending_datagram(transmit_buffer);
    transmit_buffer_position = 0;
  }
  if (transmit_datagram_bytes_remaining) {
    *data_byte = transmit_buffer[transmit_buffer_position];
    *command_bit = transmit_buffer_position == 0;
    return 1;
  } else {
    return 0;
  }
}

int seatalk_transmit_bit() {
  int current_bit = tx_bit_number;
  if (current_bit == -1) {
    if (bus_state == BUS_STATE_WAIT_FOR_IDLE) {
      set_bus_state(BUS_STATE_IDLE);
    }
    if (!seatalk_can_transmit()) { // nothing to send; exit and wait for wake
      return 0;
    }
    if (!seatalk_byte_to_send(&tx_byte, &tx_command_bit)) {
      // this should be impossible since we just confirmed data was pending
      return 0;
    }
    set_bus_state(BUS_STATE_TRANSMITTING);
#ifdef DEBUG
    LOG("character to send: %2x; current RxD value: %d; sending start bit\n", tx_byte, get_seatalk_hardware_bit_value());
#endif
    set_seatalk_hardware_bit_value(0); // start bit draws line down to zero
  } else if (current_bit <= 7) {
#ifdef DEBUG
    if ((tx_bit_number > 0) && (rx_bit_number == -1)) {
      LOG("rx didn't start on time\n");
    }
#endif
    set_seatalk_hardware_bit_value((tx_byte >> current_bit) & 0x01); // data bit
  } else if (current_bit == 8) {
    set_seatalk_hardware_bit_value(tx_command_bit); // send command bit
  } else {
    tx_bit_number = -1; // start new character on next iteration
    set_seatalk_hardware_bit_value(1); // set high for stop bit
    return 1; // transmit loop returns to bit zero and stops there if nothing to send
  }
  tx_bit_number++;
  return 1;
}
