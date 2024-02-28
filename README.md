# ADCS

## Docs

- Power Architecture
- RT1170 and RT1060 pin map
- slides and other notes
- Tim's review
- TODO move other notes here

## Hardware

### Soldermander

CPU (RT1170) + RAM (MRAM - SEMC and xSPI) + power breakout board, with 2x 100-pin board to board connectors.

### Sqrt of G

Eventual functional ADCS controller and sensor mainboard that the CPU board will sit on top of

### PCBranch

Development breakout board that breaks out all the pins of the RT1170 CPU

---

## Code

### flight

Rust code with a Teensy target, RT1170 target (not yet implemented), etc.

### cpp-code

Useful for checking that I implemented my Rust drivers correctly
