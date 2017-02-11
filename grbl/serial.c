/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


uint8_t serial_rx_buffer[RX_BUFFER_SIZE];
uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

uint8_t serial_tx_buffer[TX_BUFFER_SIZE];
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;


// Returns the number of bytes used in the RX serial buffer.
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
  uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
  return (TX_BUFFER_SIZE - (ttail-serial_tx_buffer_head));
}


void serial_init()
{
  /*
  // Set baud rate
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;
            
  // enable rx and tx
  UCSR0B |= 1<<RXEN0;
  UCSR0B |= 1<<TXEN0;
	
  // enable interrupt on complete reception of a byte
  UCSR0B |= 1<<RXCIE0;
  */
  // defaults to 8-bit, no parity, 1 stop bit
  //UARTClass Serial(UART, UART_IRQn, ID_UART, &rx_buffer1, &tx_buffer1);  
  
  // Configure PMC
  pmc_enable_periph_clk( ID_UART );
  
  // Disable PDC channel
  UART->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

  // Reset and disable receiver and transmitter
  UART->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;

  // Configure mode
  UART->UART_MR = ((US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_NO) & 0x00000E00) | UART_MR_CHMODE_NORMAL;

  // Configure baudrate (asynchronous, no oversampling)
  UART->UART_BRGR = (SystemCoreClock / BAUD_RATE) >> 4;

  // Configure interrupts
  UART->UART_IDR = 0xFFFFFFFF;
  UART->UART_IER = UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME;

  // Enable UART interrupt in NVIC
  NVIC_SetPriority(UART_IRQn, 2);
  NVIC_EnableIRQ(UART_IRQn);

  // Enable receiver and transmitter
  UART->UART_CR = UART_CR_RXEN | UART_CR_TXEN;  
  
}


// Writes one byte to the TX serial buffer. Called by main program.
// TODO: Check if we can speed this up for writing strings, rather than single bytes.
void serial_write(uint8_t data) {
  // Calculate next head
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_BUFFER_SIZE) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == serial_tx_buffer_tail) { 
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.    
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;
  
  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  //UCSR0B |=  (1 << UDRIE0); 
  UART->UART_IER = UART_IER_TXRDY;
}




// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
  uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail];
    
    tail++;
    if (tail == RX_BUFFER_SIZE) { tail = 0; }
    serial_rx_buffer_tail = tail;

    #ifdef ENABLE_XONXOFF
      if ((serial_get_rx_buffer_count() < RX_BUFFER_LOW) && flow_ctrl == XOFF_SENT) { 
        flow_ctrl = SEND_XON;
        UCSR0B |=  (1 << UDRIE0); // Force TX
      }
    #endif
    
    return data;
  }
}



void serial_reset_read_buffer() 
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}





// IT handlers
void UART_Handler(void)
{
  uint32_t status = UART->UART_SR;

  // Did we receive data?
  if ((status & UART_SR_RXRDY) == UART_SR_RXRDY){
    uint8_t data = UART->UART_RHR;
    uint8_t next_head;
    
    // Pick off realtime command characters directly from the serial stream. These characters are
    // not passed into the buffer, but these set system state flag bits for realtime execution.
    switch (data) {
      case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
      case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
      case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
      case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
      case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
      default: // Write character to buffer    
        next_head = serial_rx_buffer_head + 1;
        if (next_head == RX_BUFFER_SIZE) { next_head = 0; }
      
        // Write data to buffer unless it is full.
        if (next_head != serial_rx_buffer_tail) {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;    
                    
        }
        //TODO: else alarm on overflow?
    }
    
  }

  // Do we need to keep sending data?
  if ((status & UART_SR_TXRDY) == UART_SR_TXRDY) 
  {
    if (serial_tx_buffer_tail == serial_tx_buffer_head) {
      // Mask off transmit interrupt so we don't get it anymore
      UART->UART_IDR = UART_IDR_TXRDY;
    }
    else
    {
      uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)
      
      // Send a byte from the buffer	
      UART->UART_THR = serial_tx_buffer[tail];
    
      // Update tail position
      tail++;
      if (tail == TX_BUFFER_SIZE) { tail = 0; }
    
      serial_tx_buffer_tail = tail;
      
      // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
      if (tail == serial_tx_buffer_head) {
        // Mask off transmit interrupt so we don't get it anymore
        UART->UART_IDR = UART_IDR_TXRDY;
      }      
    }
  }

  // Acknowledge errors
  if ((status & UART_SR_OVRE) == UART_SR_OVRE || (status & UART_SR_FRAME) == UART_SR_FRAME)
  {
    // TODO: error reporting outside ISR
    UART->UART_CR |= UART_CR_RSTSTA;
  }
}

