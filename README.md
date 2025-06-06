# ESP32-Environment-Sensor

## Information
 
This is the codebase for the permanent sensor with temperature, humidity and other sensors

Made for ESP32 Devkit V1 boards. Might work on other boards but I cannot guarantee full functionality.

## Hardware

### Required hardware

- ESP32
- BME 280
- LDR
- Wires
- Power source

### Connection diagram

![Image of the connection diagram](https://github.com/VaeluxV/ESP32-Environment-Sensor/blob/ea3dedadc0790e0851738116896dd30de8186c68/img/Permanent%20Sensor%20Diagram.jpg)

## Software

### Secrets file
A `secrets-template.h` file is included in the `include/` folder, copy this file and rename it to `secrets.h` then in this file edit the defined variables to enter your passwords and other important information.

This is done this way to hide credentials from source control (GitHub/Git) without losing these credentials alltogether. The gitignore file is set up to ignore `secrets.h` out of the box.

---

All other (non-sensitive) variables are at the beginning of `main.cpp` ind the `src/` folder. You can change these to what you need.

## Questions

If you have questions or are unsure about something feel free to shoot me an email at the address listed in my profile or open an "issue" with the "question" label. I will do my best to espond as quickly as I can. Important questions should be asked via email as I read those often.

More info will be added at later date.

---

~ Vaelux
