<h1 align="center">
  <br>
  <img src="./assets/FreqCountESP-logo.png" alt="FreqCountESP" width="128">
  <br>
  FreqCountESP
  <br>
</h1>

<h4 align="center">A frequency counter library for esp32</h4>

<p align="center">
  <a href="https://img.shields.io/github/v/release/kapraran/FreqCountESP"><img src="https://img.shields.io/github/v/release/kapraran/FreqCountESP"></a>
  <a href="https://img.shields.io/github/repo-size/kapraran/FreqCountESP">
    <img src="https://img.shields.io/github/repo-size/kapraran/FreqCountESP"
          alt="Gitter">
  </a>
</p>

### Table of Contents

- [About](#about)
- [Installation](#installation)
- [Usage](#usage)
- [Credits](#credits)
- [License](#license)

## About <a name="about"></a>

A frequency counter library for esp32. It counts the numbers of pulses on a specified pin during a fixed time frame using native interrupts and timers. It only supports one instance per sketch for now.

## Installation <a name="installation"></a>

To use this library you have to import it into Arduino IDE as follows:
1. Download or clone this repository as a zip file.
2. Go to "Sketch" > "Include Library" > "Add .ZIP library..." and select the zip file to import it.

## Usage <a name="usage"></a>

#### Include the library in the sketch

```C++
#include "FreqCountESP.h"
```

#### Initialize the instance

Set which pin to use and the length of the time frame in milliseconds.

```C++
void setup()
{
  int inputPin = 14;
  int timerMs = 1000;
  FreqCountESP.begin(inputPin, timerMs);
}
```

#### Read the frequency

Wait for a new value to become available and read it by calling the `read()` method.

```C++
void loop()
{
  if (FreqCountESP.available())
  {
    uint32_t frequency = FreqCountESP.read();
    // Do something with the frequency...
  }
}
```

## Credits <a name="credits"></a>

* Big thanks to [@dpwe](https://github.com/dpwe) for his contributions https://github.com/kapraran/FreqCountESP/pull/5

* [FreqCount library for Arduino & Teensy boards](https://www.pjrc.com/teensy/td_libs_FreqCount.html)
* [Frequency Icon](https://www.flaticon.com/free-icon/pulse_597841?term=frequency&page=1&position=2)

## License <a name="license"></a>

[MIT License](https://github.com/kapraran/FreqCountESP/blob/master/LICENSE)

Copyright (c) 2020 [Nikos Kapraras](https://kapraran.dev)
