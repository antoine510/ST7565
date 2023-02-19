/*
$Id:$

ST7565 LCD library!

Copyright (C) 2010 Limor Fried, Adafruit Industries

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

// some of this code was written by <cstone@pobox.com> originally; it is in the public domain.
*/

//#include <Wire.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#endif

#ifndef _delay_ms
  #define _delay_ms(t) delay(t)
#endif

#ifndef _BV
  #define _BV(bit) (1u<<(bit))
#endif


#include <stdlib.h>

#include "ST7565.h"

// Standard Arduino
//#define SID_PORT PORTB
//#define SCLK_PORT PORTB
//constexpr uint8_t sid = 9, sclk = 8, a0 = 7, rst = 6, cs = 5;
//constexpr uint8_t sid_bit = 1 << 1, sclk_bit = 1 << 0;

// MPPTino v2
#define SID_PORT PORTB
#define SCLK_PORT PORTD
constexpr uint8_t sid = 8, sclk = 7, a0 = 6, rst = 5;
constexpr uint8_t sid_bit = 1 << 0, sclk_bit = 1 << 7;

const extern uint8_t PROGMEM font[];

// the memory buffer for the LCD
uint8_t st7565_buffer[1024]{};

void setpixelon(uint8_t x, uint8_t y) {
  if ((x >= LCDWIDTH) || (y >= LCDHEIGHT)) return;
  st7565_buffer[x + (y & 0xf8u) * 16] |= 1u << ((y & 0x7u) ^ 0x7u);
}

void setpixeloff(uint8_t x, uint8_t y) {
  if ((x >= LCDWIDTH) || (y >= LCDHEIGHT)) return;
  st7565_buffer[x + (y & 0xf8u) * 16] &= ~(1u << ((y & 0x7u) ^ 0x7u));
}

static uint8_t xUpdateMin, xUpdateMax, yUpdateMin, yUpdateMax;


static void updateBoundingBox(uint8_t xstart, uint8_t ystart, uint8_t xend, uint8_t yend) {
  if (xstart < xUpdateMin) xUpdateMin = xstart;
  if (xend > xUpdateMax) xUpdateMax = xend;
  if (ystart < yUpdateMin) yUpdateMin = ystart;
  if (yend > yUpdateMax) yUpdateMax = yend;
}

void ST7565::drawbitmap(uint8_t x, uint8_t y, 
			const uint8_t *bitmap, uint8_t w, uint8_t h,
			uint8_t color) {
  auto pxf = color ? &setpixelon : &setpixeloff;
  for (uint8_t j=0; j<h; j++) {
    for (uint8_t i=0; i<w; i++ ) {
      if (pgm_read_byte(bitmap + i + (j / 8u)*w) & _BV(j & 7u)) {
        pxf(x+i, y+j);
      }
    }
  }

  updateBoundingBox(x, y, x+w, y+h);
}

void ST7565::drawstring(uint8_t x, uint8_t line, char *c) {
  while (*c) {
    drawchar(x, line, *c);
    c++;
    x += font_width + 1;
    if (x + font_width + 1 >= LCDWIDTH) {
      x = 0;    // ran out of this line
      line += font_heightLines;
    }
    if (line + font_heightLines >= (LCDHEIGHT/8))
      return;        // ran out of space :(
  }
}


void ST7565::drawstring_P(uint8_t x, uint8_t line, const char *str) {
  uint8_t c;
  while ((c = pgm_read_byte(str++))) {
    drawchar(x, line, c);
    x += font_width + 1;
    if (x + font_width + 1 >= LCDWIDTH) {
      x = 0;    // ran out of this line
      line += 2;
    }
    if (line + font_heightLines >= (LCDHEIGHT/8))
      return;        // ran out of space :(
  }
}

void  ST7565::drawchar(uint8_t x, uint8_t line, uint8_t c) {
  for (uint8_t p = 0; p < font_heightLines; ++p) {
    for (uint8_t i = 0; i < font_width; ++i) {
      st7565_buffer[x + ((line + p) * 128) + i] = pgm_read_byte(font + (c * font_width + i + 1) * font_heightLines - p - 1);
    }
  }
  updateBoundingBox(x, line * 8, x + font_width, (line + font_heightLines) * 8);
}


// bresenham's algorithm - thx wikpedia
void ST7565::drawline(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, 
		      uint8_t color) {
  auto pxf = color ? &setpixelon : &setpixeloff;
  uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  // much faster to put the test here, since we've already sorted the points
  updateBoundingBox(x0, y0, x1+1, y1+1);

  uint8_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int8_t err = dx / 2;
  int8_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;}

  for (; x0<=x1; x0++) {
    if (steep) {
      pxf(y0, x0);
    } else {
      pxf(x0, y0);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// filled rectangle
void ST7565::fillrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
		      uint8_t color) {
  auto pxf = color ? &setpixelon : &setpixeloff;
  // stupidest version - just pixels - but fast with internal buffer!
  for (uint8_t i=x; i<x+w; i++) {
    for (uint8_t j=y; j<y+h; j++) {
      pxf(i, j);
    }
  }

  updateBoundingBox(x, y, x+w, y+h);
}

// draw a rectangle
void ST7565::drawrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
		      uint8_t color) {
  auto pxf = color ? &setpixelon : &setpixeloff;
  // stupidest version - just pixels - but fast with internal buffer!
  for (uint8_t i=x; i<x+w; i++) {
    pxf(i, y);
    pxf(i, y+h-1);
  }
  for (uint8_t i=y; i<y+h; i++) {
    pxf(x, i);
    pxf(x+w-1, i);
  } 

  updateBoundingBox(x, y, x+w, y+h);
}

// draw a circle outline
void ST7565::drawcircle(uint8_t x0, uint8_t y0, uint8_t r, 
			uint8_t color) {
  auto pxf = color ? &setpixelon : &setpixeloff;
  updateBoundingBox(x0-r, y0-r, x0+r+1, y0+r+1);

  int8_t f = 1 - r;
  int8_t ddF_x = 1;
  int8_t ddF_y = -2 * r;
  int8_t x = 0;
  int8_t y = r;

  pxf(x0, y0+r);
  pxf(x0, y0-r);
  pxf(x0+r, y0);
  pxf(x0-r, y0);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    pxf(x0 + x, y0 + y);
    pxf(x0 - x, y0 + y);
    pxf(x0 + x, y0 - y);
    pxf(x0 - x, y0 - y);
    
    pxf(x0 + y, y0 + x);
    pxf(x0 - y, y0 + x);
    pxf(x0 + y, y0 - x);
    pxf(x0 - y, y0 - x);
    
  }



}

void ST7565::fillcircle(uint8_t x0, uint8_t y0, uint8_t r, 
			uint8_t color) {
  updateBoundingBox(x0-r, y0-r, x0+r+1, y0+r+1);
  auto pxf = color ? &setpixelon : &setpixeloff;

  int8_t f = 1 - r;
  int8_t ddF_x = 1;
  int8_t ddF_y = -2 * r;
  int8_t x = 0;
  int8_t y = r;

  for (uint8_t i=y0-r; i<=y0+r; i++) {
    pxf(x0, i);
  }

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    for (uint8_t i=y0-y; i<=y0+y; i++) {
      pxf(x0+x, i);
      pxf(x0-x, i);
    } 
    for (uint8_t i=y0-x; i<=y0+x; i++) {
      pxf(x0+y, i);
      pxf(x0-y, i);
    }    
  }
}


// the most basic function, set a single pixel
void ST7565::setpixel(uint8_t x, uint8_t y, uint8_t color) {
  if(color) setpixelon(x, y); else setpixeloff(x, y);
  if ((x >= LCDWIDTH) || (y >= LCDHEIGHT))
    return;

  updateBoundingBox(x,y,x+1,y+1);
}


// the most basic function, get a single pixel
uint8_t ST7565::getpixel(uint8_t x, uint8_t y) {
  if ((x >= LCDWIDTH) || (y >= LCDHEIGHT))
    return 0;

  return (st7565_buffer[x+ (y/8)*128] >> (7-(y%8))) & 0x1;  
}

void ST7565::begin(uint8_t contrast) {
  st7565_init();
  st7565_command(CMD_DISPLAY_ON);
  st7565_command(CMD_SET_ALLPTS_NORMAL);
  st7565_set_brightness(contrast);
}

void ST7565::st7565_init(void) {
  // set pin directions
  pinMode(sid, OUTPUT);
  pinMode(sclk, OUTPUT);
  pinMode(a0, OUTPUT);
  pinMode(rst, OUTPUT);

  digitalWrite(rst, LOW);
  _delay_ms(500);
  digitalWrite(rst, HIGH);

  // LCD bias select
  st7565_command(CMD_SET_BIAS_7);
  // ADC select
  st7565_command(CMD_SET_ADC_NORMAL);
  // SHL select
  st7565_command(CMD_SET_COM_NORMAL);
  // Initial display line
  st7565_command(CMD_SET_DISP_START_LINE);

  // turn on voltage converter (VC=1, VR=0, VF=0)
  st7565_command(CMD_SET_POWER_CONTROL | 0x4);
  // wait for 50% rising
  _delay_ms(50);

  // turn on voltage regulator (VC=1, VR=1, VF=0)
  st7565_command(CMD_SET_POWER_CONTROL | 0x6);
  // wait >=50ms
  _delay_ms(50);

  // turn on voltage follower (VC=1, VR=1, VF=1)
  st7565_command(CMD_SET_POWER_CONTROL | 0x7);
  // wait
  _delay_ms(10);

  // set lcd operating voltage (regulator resistor, ref voltage resistor)
  st7565_command(CMD_SET_RESISTOR_RATIO | 0x6);

  // initial display line
  // set page address
  // set column address
  // write display data

  // set up a bounding box for screen updates

  updateBoundingBox(0, 0, LCDWIDTH, LCDHEIGHT);
}

inline void ST7565::spiwrite(uint8_t c) {
  SCLK_PORT &= ~sclk_bit;
  if (c & _BV(7))
    SID_PORT |= sid_bit;
  else
    SID_PORT &= ~sid_bit;
  SCLK_PORT |= sclk_bit;

  SCLK_PORT &= ~sclk_bit;
  if (c & _BV(6))
    SID_PORT |= sid_bit;
  else
    SID_PORT &= ~sid_bit;
  SCLK_PORT |= sclk_bit;
 
  SCLK_PORT &= ~sclk_bit;
  if (c & _BV(5))
    SID_PORT |= sid_bit;
  else
    SID_PORT &= ~sid_bit;
  SCLK_PORT |= sclk_bit;

  SCLK_PORT &= ~sclk_bit;
  if (c & _BV(4))
    SID_PORT |= sid_bit;
  else
    SID_PORT &= ~sid_bit;
  SCLK_PORT |= sclk_bit;

  SCLK_PORT &= ~sclk_bit;
  if (c & _BV(3))
    SID_PORT |= sid_bit;
  else
    SID_PORT &= ~sid_bit;
  SCLK_PORT |= sclk_bit;

  SCLK_PORT &= ~sclk_bit;
  if (c & _BV(2))
    SID_PORT |= sid_bit;
  else
    SID_PORT &= ~sid_bit;
  SCLK_PORT |= sclk_bit;

  SCLK_PORT &= ~sclk_bit;
  if (c & _BV(1))
    SID_PORT |= sid_bit;
  else
    SID_PORT &= ~sid_bit;
  SCLK_PORT |= sclk_bit;

  SCLK_PORT &= ~sclk_bit;
  if (c & _BV(0))
    SID_PORT |= sid_bit;
  else
    SID_PORT &= ~sid_bit;
  SCLK_PORT |= sclk_bit;
}

void ST7565::st7565_command(uint8_t c) {
  digitalWrite(a0, LOW);

  spiwrite(c);
}

void ST7565::st7565_data(uint8_t c) {
  digitalWrite(a0, HIGH);

  spiwrite(c);
}
void ST7565::st7565_set_brightness(uint8_t val) {
    st7565_command(CMD_SET_VOLUME_FIRST);
    st7565_command(CMD_SET_VOLUME_SECOND | (val & 0x3f));
}


void ST7565::display(void) {
  uint8_t page = yUpdateMin / 8;

  uint8_t* buf = st7565_buffer + page * 128;
  for(; page * 8 < yUpdateMax; page++) {
    st7565_command(CMD_SET_PAGE | page ^ 0x7); // Reverse order

    st7565_command(CMD_SET_COLUMN_LOWER | (xUpdateMin & 0xf));
    st7565_command(CMD_SET_COLUMN_UPPER | (xUpdateMin >> 4));
    st7565_command(CMD_RMW);
    
    for(uint8_t col = xUpdateMin; col < xUpdateMax; col++) {
      st7565_data(buf[col]);
    }
    buf += 128;
  }


  xUpdateMin = LCDWIDTH;
  xUpdateMax = 0;
  yUpdateMin = LCDHEIGHT;
  yUpdateMax = 0;
}

// clear everything
void ST7565::clear(void) {
  memset(st7565_buffer, 0, 1024);
  updateBoundingBox(0, 0, LCDWIDTH, LCDHEIGHT);
}


// this doesnt touch the buffer, just clears the display RAM - might be handy
void ST7565::clear_display(void) {
  uint8_t p, c;
  
  for(p = 0; p < 8; p++) {
    st7565_command(CMD_SET_PAGE | p);
    for(c = 0; c < 128; c++) {
      st7565_command(CMD_SET_COLUMN_LOWER | (c & 0xf));
      st7565_command(CMD_SET_COLUMN_UPPER | (c >> 4));
      st7565_data(0x0);
    }     
  }
}
