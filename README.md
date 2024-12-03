# Meshtastic Firmware

![GitHub release downloads](https://img.shields.io/github/downloads/meshtastic/firmware/total)
[![CI](https://img.shields.io/github/actions/workflow/status/meshtastic/firmware/main_matrix.yml?branch=master&label=actions&logo=github&color=yellow)](https://github.com/meshtastic/firmware/actions/workflows/ci.yml)
[![CLA assistant](https://cla-assistant.io/readme/badge/meshtastic/firmware)](https://cla-assistant.io/meshtastic/firmware)
[![Fiscal Contributors](https://opencollective.com/meshtastic/tiers/badge.svg?label=Fiscal%20Contributors&color=deeppink)](https://opencollective.com/meshtastic/)
[![Vercel](https://img.shields.io/static/v1?label=Powered%20by&message=Vercel&style=flat&logo=vercel&color=000000)](https://vercel.com?utm_source=meshtastic&utm_campaign=oss)

## Overview

Class project for the University of Michigan's EECS 507 - Introduction to Embedded Systems Research. 

Focused on expanding Meshtastic to support other transmission mediums, with the goal of providing an easy framework for building quick mesh networks off of existing routing and RTOS code.

## Project Details

Project scope begins with replacing Meshtastic's router to deal with interfaces for simple 433MHz Amplitude Shift Keying devices. This particular build tested on the NRF52 chipset, particularly in the RAK4631 boards. To make this possible, the following was done:

 1. Ported Paul Stoffregen's RadioHead library, particularly the ASK driver, to the NRF52 chip. Timer3 is used for interrupts.
 2. Replaced RadioLibInterface with a generic RF433Interface to implement sending
 3. Filled in missing RadioLibInterface functions dealing with recieving. Currently handled via threaded polling.

Use of the transcievers requires two GPIO ports. References are made towards the free GPIO pins being used for GPS functions, so the GPS has been tentatively disabled. Once the appropriate functions for RadioLib have been suitably implemented in an inheritor, Meshtastic takes over and provides easy routing and retransmission for sent packets. 

## Component Details

### Transcievers
Any mix and match of tx + rx chip *should* theoretically work. My conclusion? Not exactly.

#### FS100A (tx) + MK-RM-5V (rx)
Mixed bag, quite literally. The FS100A works perfectly fine. 

The MK-RM-5V? Horrible. Spent 2 weeks before looking up why I was getting only *centimeters* of range after thinking it was an antenna issue - [apparently this is the consensus.](https://forum.arduino.cc/t/433mhz-transmitter-coil-missing-solved/581338/2)  

#### SYN480R + SYN115
Purchased these as well when the above chipset was wonky. You may find they're fairly poorly labelled. The one with the smaller 6 pin chip is the [SYN115](https://www.rhydolabz.com/documents/33/SYN113-SYN115-datasheet-version-1-1-.0.pdf) transmitter. The quartz clock *may* be labelled 13.560.

The 4 pin chip will be the [SYN480R](https://voltiq.ru/datasheets/SYN480R.pdf). Quartz clock for me was labelled 6.745. 

These things are much better and a lot simpler than the above combo. Would reccommend. Also recommended from the RadioHead library.


## General Meshtastic Links
- **[Building Instructions](https://meshtastic.org/docs/development/firmware/build)**
- **[Flashing Instructions](https://meshtastic.org/docs/getting-started/flashing-firmware/)**

## MeshtASKtic Info
