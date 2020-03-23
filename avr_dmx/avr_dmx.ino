
#define error_exit(x) do{current_error=x;state=STATE_ERROR;return;}while(0);

// Protocol values for communicating via serial.
enum {
  PC_PROT_PROMPT = 0x12,
  PC_PROT_PROMPT_2 = 0x78,
  PC_PROT_RESPONSE = 0x33,
  PC_PROT_READY_FOR_PACKET = 0x44,
  PC_PROT_SENDING = 0x55,
  PC_PROT_ERROR = 0x66,
  PC_PROT_SENT = 0x99,
};

// State machine states enum.
typedef enum {
  STATE_WAIT_FOR_CONNECTION,
  STATE_RECEIVE_PACKET,
  STATE_SEND_PACKET,
  STATE_DONE_PACKET,
  STATE_ERROR,
} StateMachine;

// Control Codes
typedef enum {
  CC_NONE         = 0x00,
  CC_INFO         = 0xff,
  CC_SET_PBM_OFF  = 0x10,
  CC_SET_PBM_ON   = 0x11,
  CC_RESET_BR     = 0x20,
  CC_SET_BR       = 0x21,
  CC_SET_BR_SLOW  = 0x22,
} ControlCode;

// Packet type enum.
typedef enum {
  PACKET_T_DATA          = 0x00, // Uncompressed
  PACKET_T_COMP_DATA_RLE = 0x01, // Run Length Encoding
  PACKET_T_COMP_DATA_SRE = 0x02, // Self-Referential Encoding
  PACKET_T_COMP_DATA_1BP = 0x03, // 1-bit precision
  PACKET_T_COMP_DATA_2BP = 0x04, // 2-bit precision
  PACKET_T_COMP_DATA_4BP = 0x05, // 4-bit precision
  PACKET_T_COMP_DATA_SUM = 0x06, // Subset update data, this is only really useful when in persistent buffer mode.
  PACKET_T_META          = 0xff  // Control Signal
} PacketType;

// Packet header structure
typedef struct {
  PacketType packet_type         : 8;
  unsigned short packet_length   : 16;

} PCPacketHeader;

// Packet buffer type
typedef struct {
  PCPacketHeader packet_header;
  byte packet_data[513];
} PCDataPacket;

byte current_state[513];

// The packet buffer and a pointer for writing raw bytes to it.
PCDataPacket packet_read_buffer;
byte* packet_read_buffer_pointer = (byte*)&packet_read_buffer;

// Number of times to repeat the data in the handshake.
const int REPEAT_VALUE = 7;

// This is the error code of the last error.
byte current_error = 0x00;

boolean persistent_buffer_mode = true;

// This controls the global state machine.
StateMachine state = STATE_WAIT_FOR_CONNECTION;

// This is the fastest baudrate that the Arduino Uno seems happy with.
const long DEFAULT_BAUDRATE = 9600;

long baudrate = DEFAULT_BAUDRATE;

const char* INFO_STRING_START = "AVR DMX V1\n"
                                "By Jacob Allen (polyomino.xyz)\n"
                                "---INFO---\n";
const char* INFO_STRING_DATA  = "  BAUDRATE=%020lu\n"
                                "  PBM=%s\n";
const size_t INFO_STRING_DYNAMIC_LENGTH = 12 + 20 + 3 + 7 + 1;
const char* INFO_STRING_END   = "---END---\n\n";

void start_serial() {
  // Start serial at the max baudrate.
  Serial.begin(baudrate);

  // Wait for serial to initialise...
  while (!Serial);
}

void stop_serial() {
  Serial.end();
}

/**
   Setup everything.
*/
void setup() {
  // Enable the LED, this means it will match the state of pin 13 due to the way ports are set.
  pinMode(LED_BUILTIN, OUTPUT);

  // Enable pin 13 for output. This is the pin DMX data will use. It is known has port B pin 5 to Atmel.
  pinMode(13, OUTPUT);

  // Reset the packet type.
  packet_read_buffer.packet_header = {PACKET_T_DATA, 0};

  // Reset the read buffer.
  memset(packet_read_buffer.packet_data, 0, sizeof(packet_read_buffer.packet_data));

  memset(current_state, 0, sizeof(current_state));

  start_serial();
}

/**
   Connection wait state, waits for a connection on the serial port.

   If a connection is found, transitions to the receive packet state,
   otherwise loops.
*/
void handle_state_wait_for_connection() {
  if (baudrate != DEFAULT_BAUDRATE) {
    stop_serial();
    baudrate = DEFAULT_BAUDRATE;
    start_serial();
  }

  byte handshake_buffer[REPEAT_VALUE * 2];
  byte check_index = 0;
  byte read_count = 0;

  // Prompt for connection.
  Serial.write(PC_PROT_RESPONSE);

  // Try and read handshake part 1.
  read_count = Serial.readBytes(handshake_buffer, REPEAT_VALUE);

  // If we didn't get enough data for the handshake, loop.
  if (read_count != REPEAT_VALUE) return;

  // Check the data we received to make sure it's correct for the handshake.
  while (check_index < REPEAT_VALUE) {
    if (handshake_buffer[check_index] != PC_PROT_PROMPT) error_exit(0x01);
    check_index++;
  };

  // Ask for the second part of the handshake.
  Serial.write(PC_PROT_RESPONSE);

  // Try and read handshake part 2.
  read_count = Serial.readBytes(&(handshake_buffer[REPEAT_VALUE]), REPEAT_VALUE);

  // If we didn't get enough data for the handshake, loop.
  if (read_count != REPEAT_VALUE) return;

  // Check the data for the second half of the handshake.
  while (check_index < REPEAT_VALUE * 2) {
    if (handshake_buffer[check_index] != PC_PROT_PROMPT_2) error_exit(0x02);
    check_index++;
  };

  // Finally, if all is good, start waiting for instructions.
  state = STATE_RECEIVE_PACKET;
}

/**
   Decode encoded data.
*/
void handle_data_decode() {
  unsigned short packet_length = packet_read_buffer.packet_header.packet_length;
  // If we have more data than is possible, set to the max we think is possible.
  if (packet_length > sizeof(current_state)) packet_length = sizeof(current_state);
  // This shouldn't happen, but as the rest of the method assumes that packet_length > 0, check to make sure here.
  else if (packet_length == 0) return;

  switch (packet_read_buffer.packet_header.packet_type) {
    case PACKET_T_COMP_DATA_1BP:
      {
        /**
           1 bit data encoding format.
        */
        current_state[0] = packet_read_buffer.packet_data[0];
        for (unsigned short i = 1; i < packet_length; i++) {
          const unsigned short destination_base_index = ((i - 1) * 8) + 1;
          const byte current_byte = packet_read_buffer.packet_data[i];

          for (byte bit_index = 0; bit_index < 8; bit_index++) {
            current_state[destination_base_index + bit_index] = ((current_byte >> bit_index) & 0x01) * 0xff;
          }
        }
      }
      break;
    case PACKET_T_COMP_DATA_2BP:
      {
        /**
           2 bit data depth encoding format.
        */
        current_state[0] = packet_read_buffer.packet_data[0];
        for (unsigned short i = 1; i < packet_length; i++) {
          const unsigned short destination_base_index = ((i - 1) * 4) + 1;
          const byte current_byte = packet_read_buffer.packet_data[i];

          for (byte bit_index = 0; bit_index < 4; bit_index++) {
            current_state[destination_base_index + bit_index] = ((current_byte >> (2 * bit_index)) & 0x03) * 0x55;
          }
        }
      }
      break;
    case PACKET_T_COMP_DATA_4BP:
      {
        /**
           4 bit data depth encoding format.
        */
        current_state[0] = packet_read_buffer.packet_data[0];
        for (unsigned short i = 1; i < packet_length; i++) {
          const unsigned short destination_base_index = ((i - 1) * 2) + 1;
          const byte current_byte = packet_read_buffer.packet_data[i];

          current_state[destination_base_index] = (current_byte & 0x0f) * 0x11;
          current_state[destination_base_index + 1] = ((current_byte >> 4) & 0x0f) * 0x11;
        }
      }
      break;
    case PACKET_T_COMP_DATA_RLE:
      {
        /**
           Run length encoding format.
        */
        unsigned short run_count = 0;
        unsigned short output_index = 1;
        current_state[0] = packet_read_buffer.packet_data[0];

        for (unsigned short i = 1; i < packet_length; i++) {
          if (run_count == 0) {
            run_count = packet_read_buffer.packet_data[i];
          } else {
            while (run_count > 0 && output_index < sizeof(current_state)) {
              current_state[output_index++] = packet_read_buffer.packet_data[i];
              run_count++;
            }
          }
        }
      }
      break;
    case PACKET_T_COMP_DATA_SUM:
      /**
         Set range encoding format.
      */
      {
        unsigned short start_index = (((unsigned short)packet_read_buffer.packet_data[0]) << 8)
                                     | ((unsigned short)packet_read_buffer.packet_data[1]);

        for (unsigned short i = 2; i < packet_length && (i + start_index) < sizeof(current_state); i++)
          current_state[start_index + i - 2] = packet_read_buffer.packet_data[i];
      }
      break;
    case PACKET_T_DATA:
    default:
      /**
         Defaults to uncompressed.
      */
      memcpy(current_state, packet_read_buffer.packet_data, packet_length);
      break;
  }
}

/**
   Send ascii info over serial.
*/
void handle_info_control_code() {
  char print_buffer[INFO_STRING_DYNAMIC_LENGTH];
  sprintf(print_buffer, INFO_STRING_DATA, baudrate, persistent_buffer_mode ? "ON " : "OFF");

  Serial.print(INFO_STRING_START);
  Serial.print(print_buffer);
  Serial.print(INFO_STRING_END);
};

/**
   Handle a response to a control packet.
*/
void handle_control_packet() {
  const ControlCode control_code = (ControlCode) packet_read_buffer.packet_data[0];

  switch (control_code) {
    case CC_INFO:
      handle_info_control_code();
      break;
    case CC_SET_PBM_OFF:
      persistent_buffer_mode = false;
      break;
    case CC_SET_PBM_ON:
      persistent_buffer_mode = true;
      break;
    case CC_RESET_BR:
      baudrate = DEFAULT_BAUDRATE;
      Serial.write(PC_PROT_RESPONSE);
      stop_serial();
      delay(100);
      start_serial();
      break;
    case CC_SET_BR:
      baudrate = ((((long)packet_read_buffer.packet_data[1]) << 24)
                  +  (((long)packet_read_buffer.packet_data[2]) << 16)
                  +  (((long)packet_read_buffer.packet_data[3]) << 8)
                  +  (((long)packet_read_buffer.packet_data[4])));
      Serial.write(PC_PROT_RESPONSE);
      stop_serial();
      delay(100);
      start_serial();
      break;
    case CC_SET_BR_SLOW:
      /*
       * Windows takes dramatically more time to change baudrate, so we need a longer delay.
       */
      baudrate = ((((long)packet_read_buffer.packet_data[1]) << 24)
                  +  (((long)packet_read_buffer.packet_data[2]) << 16)
                  +  (((long)packet_read_buffer.packet_data[3]) << 8)
                  +  (((long)packet_read_buffer.packet_data[4])));
      Serial.write(PC_PROT_RESPONSE);
      stop_serial();
      delay(2000);
      start_serial();
      break;
    case CC_NONE:
    default:
      return;
  }
  Serial.write(PC_PROT_RESPONSE);
}

/**
   The receive packet state, reads a packet from the serial port.

   No meta packet functions have been implemented yet.

   This state transitions to the send packet state if a data packet is received,
   otherwise this state loops.
*/
void handle_state_receive_packet() {
  unsigned short read_bytes = 0;
  unsigned short packet_length = 0;

  // Ask for data
  Serial.write(PC_PROT_READY_FOR_PACKET);

  read_bytes = Serial.readBytes(packet_read_buffer_pointer, sizeof(PCPacketHeader));
  if (read_bytes != sizeof(PCPacketHeader)) {
    error_exit(0x03);
  }

  packet_length = packet_read_buffer.packet_header.packet_length;

  // Read provided data
  read_bytes = Serial.readBytes(&(packet_read_buffer_pointer[sizeof(PCPacketHeader)]), packet_length);

  // If we didn't get enough data, something must have gone wrong.
  // This could happen if the timeout is exceeded (1000ms).
  if (read_bytes > packet_length) {
    error_exit(0x04);
  }
  else if (read_bytes < packet_length) {
    error_exit(0x05);
  }

  // If it's a meta packet, do something else.
  if (packet_read_buffer.packet_header.packet_type == PACKET_T_META) {
    handle_control_packet();
    state = STATE_RECEIVE_PACKET;
  }
  // If it's encoded, decode it.
  else {
    handle_data_decode();
    state = STATE_SEND_PACKET;
  }
}

/**
   Send packet state, sends the packet in the packet buffer.

   The majority of this function is written in assembly to allow fine grain control of the exact
   amount of time which the function will take for each step. This ensures that each bit is sent
   accurately (each bit being 64 clock cycles long).

   This is based on the assumption that 1 microsecond = 16 clock cycles, on platforms where this
   is not the case this code will not work.

   This state always transitions to the send done state.
*/
void handle_state_send_packet() {
  // Indicate sending has started
  Serial.write(PC_PROT_SENDING);
  // Disable interrupts and send the break and MAB.
  asm volatile(
    "cli ; clear interrupts\n"
    "cbi %[port],%[pin] ; break"
    "mov R16,__zero_reg__ ; 1 once\n"
    "padding_loop_4: inc R16 ; 1 each time\n"
    "cpi R16,0xd7 ; 1 each time, should be 0xC8 but hand tuned\n"
    "brne padding_loop_4 ; 2 when jump back, else 1\n"
    "; padding loop end, has taken 200 cycles ;\n"
    "mov R16,__zero_reg__ ; 1 once\n"
    "padding_loop_5: inc R16 ; 1 each time\n"
    "cpi R16,0xd7 ; 1 each time, should be 0xC8 but hand tuned\n"
    "brne padding_loop_5 ; 2 when jump back, else 1\n"
    "; padding loop end, has taken 200 cycles ;\n"
    "sbi %[port],%[pin] ; MAB\n"
    "mov R16,__zero_reg__ ; 1 once\n"
    "padding_loop_6: inc R16 ; 1 each time\n"
    "cpi R16,0x2E ; 1 each time, this should be 0x30 but was hand tuned\n"
    "brne padding_loop_6 ; 2 when jump back, else 1\n"
    "; padding loop end, has taken 48 cycles ;\n"
    :
    : [port] "I" (_SFR_IO_ADDR (PORTB)), [pin] "I" (PORTB5)
    : "r16"
  );
  // Main send loop, should be changed to entirely assembly at some point.
  // Right now it's close enough with half a microsecond drift here and there.
  for (unsigned int i = 0; i < sizeof(current_state); i++) {
    byte to_send = current_state[i];
    asm volatile(
      "cbi %[port],%[pin] ; start start bit (2,C2)\n"
      "mov R17,__zero_reg__ ; (1,C3)\n"
      "mov __tmp_reg__,%[slot] ; load slot into a temp register (1,C4)\n"
      "; padding loop start ;\n"
      "mov R16,__zero_reg__ ; ; 1 once\n"
      "padding_loop_1: inc R16 ; 1 each time\n"
      "cpi R16,0x0E ; 1 each time\n"
      "brne padding_loop_1 ; 2 when jump back, else 1\n"
      "; padding loop end, has taken 56 cycles ;\n"
      "nop ; single extra padding bit (logic analysis says it should be here)\n"
      "start_of_loop: sbrs __tmp_reg__,0 ; skip next instruction if bit set (B60)\n"
      "; if bit in register not set (1,C61)\n"
      "; if bit in register is set (2,C62)\n"
      "rjmp set_out_off ; jump to clear bit (2,C62)\n"
      "nop ; padding to ensure same timing (1,C62)\n"
      "sbi %[port],%[pin] ; set pin on (2,C0/64)\n"
      "rjmp end_of_branch ; jump to end of branch (2,C2)\n"
      "set_out_off: cbi %[port],%[pin] ; set pin off (2,C0/64)\n"
      "nop ; (1,C1)\n"
      "nop ; padding to ensure same timing (1,C2)\n"
      "end_of_branch: lsr __tmp_reg__ ; shift temp register left to get next bit (1,C3)\n"
      "; padding loop start ;\n"
      "mov R16,__zero_reg__ ; ; 1 once\n"
      "padding_loop_2: inc R16 ; 1 each time\n"
      "cpi R16,0x0D ; 1 each time\n"
      "brne padding_loop_2 ; 2 when jump back, else 1\n"
      "nop\n"
      "; padding loop end, has taken 53 cycles ;\n"
      "inc R17 ; (1,C57)\n"
      "cpi R17,0x08 ; (1,C58)\n"
      "brne start_of_loop\n"
      "; if not equal so jumping back (2,C60)\n"
      "; if equal so continueing forward (1,C59)\n"
      "nop ; (1,C60)\n"
      "nop ; (1,C61)\n"
      "nop ; (1,C62)\n"
      "sbi %[port],%[pin] ; start stop bits (2, C0/64)\n"
      "; 126 nops? Maybe less. We just need to ensure that by the time this assembly\n"
      "; is called again it has been at least 128 cycles or 8 microseconds.\n"
      "; padding loop start ;\n"
      "mov R16,__zero_reg__ ; ; 1 once\n"
      "padding_loop_3: inc R16 ; 1 each time\n"
      "cpi R16,0x1D ; 1 each time, 0x1F is strickly correct but the compiler takes longer so we account for that in a very\n"
      "; tial and error method here. The whole section should be re-written in assembly.\n"
      "brne padding_loop_3 ; 2 when jump back, else 1\n"
      "nop\n"
      "nop\n"
      "; padding loop end, has taken 126 cycles ;\n" /* Code */ :
      /* Output */ :
      [slot] "l" (to_send), [port] "I" (_SFR_IO_ADDR (PORTB)), [pin] "I" (PORTB5) /* input */  :
      "r17", "r16" /* Clobbers */
    );
  }
  // Enable interrupts
  asm volatile (
    "sei ; set interrupts\n" : :
  );
  state = STATE_DONE_PACKET;
}

/**
   Packet done state, handles indicating that a packet was sent.

   This state always transitions to the receive packet state.
*/
void handle_state_done_packet() {
  Serial.write(PC_PROT_SENT);
  if (!persistent_buffer_mode)
    memset(current_state, 0x00, sizeof(current_state));
  state = STATE_RECEIVE_PACKET;
}

/**
   Error state, writes the error signal then the error code to the serial.

   This state always transitions to the wait for connection state.
*/
void handle_state_error() {
  Serial.write(PC_PROT_ERROR);
  Serial.write(current_error);
  state = STATE_WAIT_FOR_CONNECTION;
}

void loop() {
  // Core loop is a simple state machine.
  switch (state) {
    case STATE_WAIT_FOR_CONNECTION:
      handle_state_wait_for_connection();
      break;
    case STATE_RECEIVE_PACKET:
      handle_state_receive_packet();
      break;
    case STATE_SEND_PACKET:
      handle_state_send_packet();
      break;
    case STATE_DONE_PACKET:
      handle_state_done_packet();
      break;
    case STATE_ERROR:
      handle_state_error();
      break;
    default:
      state = STATE_ERROR;
      break;
  }
}
