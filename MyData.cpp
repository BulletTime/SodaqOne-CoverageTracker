/*
  The MIT License (MIT)

  Copyright © 2018 Sven Agneessens <sven.agneessens@gmail.com>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include "MyData.h"
#include "FlashStorage.h"

#define DEFAULT_HEADER 0xBEEF

MyData myData;

FlashStorage(flash, MyData);
static bool needsCommit;

static uint16_t crc16ccitt(const uint8_t *buf, size_t len) {
    uint16_t crc = 0;
    while (len--) {
        crc ^= (*buf++ << 8);
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            }
            else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

void MyData::read() {
    flash.read(this);

    // check header and CRC
    uint16_t calcCRC16 = crc16ccitt((uint8_t*)this, (uint32_t)&myData._crc16 - (uint32_t)&myData._header);
    if (_header != DEFAULT_HEADER || _crc16 != calcCRC16) {
        reset();
    }
}

void MyData::reset() {
  _amount = 0;
  _power = 1;
  memset(_loc, 0, sizeof(_loc));
}

/*
 * Write the configuration parameters to NVM / Dataflash
 */
void MyData::commit(bool forced) {
    if (!forced && !needsCommit) {
        return;
    }

    _header = DEFAULT_HEADER;
    _crc16 = crc16ccitt((uint8_t*)this, (uint32_t)&myData._crc16 - (uint32_t)&myData._header);

    flash.write(*this);

    needsCommit = false;
}

void MyData::setPower(uint8_t power) {
  _power = power;
  needsCommit = true;
}

bool MyData::addLocation(int32_t lat, int32_t lon) {
  if (_amount < SIZE_LAT_LON) {
    _loc[_amount]._latitude = lat;
    _loc[_amount]._latitude = lat;
    _amount++;

    needsCommit = true;

    return true;
  }

  return false;
}
