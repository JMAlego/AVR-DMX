# AVR-DMX

Turn an Arduino into a DMX interface.

## Introduction

AVR-DMX was born out of one question: can an Arduino Uno send DMX data? Now it's pretty obvious the answer is going to be yes, the question then is how? The main hurdle is that the UART on the Uno is connected (via another chip) to the USB which is (typically) connected to a computer. This means that we can't use the UART to send DMX data while the Uno is connected to a computer. This means we either have to use some other method to connect to the computer, or we have to use something other than the UART to send DMX.

The solution used in this project is to write the code to send the data in assembly. This allows the timing to be made consistent by counting CPU cycles and ensuring that all code paths take the correct number of CPU cycles to execute. For the Uno (clocked at the typical 16 MHz) this is 16 CPU cycles per microsecond. As it turns out the Uno is more than capable of this task.

It is also worth noting that the output of the Uno is single-ended DMX and not the differential signal required to send over a DMX bus. Many chips, such as the MAX485, exist to convert between single-ended and DMX bus compatible differential signals. It is therefore simple to create a singled-ended to differential signal converter which will work with this project.

My other, inventively named, [PyDMX](https://github.com/JMAlego/PyDMX) project features a working driver implementation for the AVR-DMX.

## Features

A high-level overview of the features currently implemented by AVR-DMX is as follows:

### Complete

- Send DMX packets.
- Select transfer baud rate.
- Select transfer encoding.
- select buffer persistence.
- Consistent and accurate slot sending.

### Partial

- Efficient packet encoding for data transfer.
- Transfer debug info.

### Todo

- Improve inter-slot timing by implementing it entirely in assembly.
- Receive DMX packets.

## Design Overview

The code for AVR-DMX is relatively simple, with one core state-machine controlling the overall status of the project.

### States

Below are the states of the top-level state machine.

#### `STATE_WAIT_FOR_CONNECTION`

In this state the AVR-DMX will wait for a handshake for 1 second.

If no connection is received during this time, or the handshake is failed, an error is returned and it moves into `STATE_ERROR`.

If a connection is received and the handshake succeeds then we move into the `STATE_RECEIVE_PACKET` state.

#### `STATE_RECEIVE_PACKET`

In this state the AVR-DMX waits for a packet from the connected device.

If the packet it receives is a DMX packet it moves to the `STATE_SEND_PACKET` state.

If the packet received is a control packet, the associated action is taken then, it moves to the start of the `STATE_RECEIVE_PACKET`state again.

#### `STATE_SEND_PACKET`

In this state the AVR-DMX is sending a DMX packet over the DMX bus.

This state will always move to `STATE_DONE_PACKET`.

#### `STATE_DONE_PACKET`

In this state sends a "sent" notification to the connected computer and clears the buffer if persistent buffer mode is off.

This state will always move to `STATE_RECEIVE_PACKET`.

#### `STATE_ERROR`

This state sends a message to the connected device (if any) informing of an error.

This state will always move to `STATE_WAIT_FOR_CONNECTION`.

### Configuration

#### Baudrate

The AVR-DMX defaults to 9600 baud, but can be configured to run at a specified baud by sending the relevant control code.

#### Transfer Encoding

Each packet can use a different transfer encoding. There are a few different encodings to pick from, each of which have positives and negatives. By default transfers happen in "raw" encoding.

#### Persistent Buffer Mode

The AVR-DMX defaults to having persistent buffer mode on. When this feature is enabled the DMX packet buffer is not zeroed after each DMX packet is sent. This can allow increment changes to the DMX message to be sent for more efficient transfer encodings. This feature can be set by sending the relevant control code.

## Get

To get the latest version of the project run:

```bash
git clone https://github.com/JMAlego/AVR-DMX.git
cd AVR-DMX
```

## Build

Build the project for Arduino Uno compatible board:

```bash
make build
```

## Upload

Upload the project to an Arduino Uno compatible board:

```bash
make upload
```
