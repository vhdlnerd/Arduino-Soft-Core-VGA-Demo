
////////////////////////////////////////
#define BASE_ADDR         (0x0FE0)
#define VGA_WR_CURS_CNTL  _SFR_IO8(BASE_ADDR+0x00)
#define VGA_WR_FG_COLOR   _SFR_IO8(BASE_ADDR+0x01)
#define VGA_WR_BG_COLOR   _SFR_IO8(BASE_ADDR+0x02)
#define VGA_WR_INVERSE    _SFR_IO8(BASE_ADDR+0x03)
#define VGA_WR_POS_HI     _SFR_IO8(BASE_ADDR+0x04)
#define VGA_WR_POS_LO     _SFR_IO8(BASE_ADDR+0x05)
#define VGA_WR_EMIT       _SFR_IO8(BASE_ADDR+0x06)
#define VGA_WR_EMIT_INCR  _SFR_IO8(BASE_ADDR+0x07)
#define VGA_WR_FILL       _SFR_IO8(BASE_ADDR+0x08)

#define VGA_RD_CURS_CNTL  _SFR_IO8(BASE_ADDR+0x00)
#define VGA_RD_FG_COLOR   _SFR_IO8(BASE_ADDR+0x01)
#define VGA_RD_BG_COLOR   _SFR_IO8(BASE_ADDR+0x02)
#define VGA_RD_INVERSE    _SFR_IO8(BASE_ADDR+0x03)
#define VGA_RD_POS_HI     _SFR_IO8(BASE_ADDR+0x04)
#define VGA_RD_POS_LO     _SFR_IO8(BASE_ADDR+0x05)
#define VGA_RD_CHAR       _SFR_IO8(BASE_ADDR+0x06)
#define VGA_RD_CHAR_INCR  _SFR_IO8(BASE_ADDR+0x07)
#define VGA_RD_CHAR_ATTR  _SFR_IO8(BASE_ADDR+0x08)
#define VGA_RD_BUSY       _SFR_IO8(BASE_ADDR+0x09)
#define VGA_RD_DIS_COLS   _SFR_IO8(BASE_ADDR+0x10)
#define VGA_RD_DIS_ROWS   _SFR_IO8(BASE_ADDR+0x11)
#define VGA_RD_DIS_COLOR_BITS   _SFR_IO8(BASE_ADDR+0x12)
#define VGA_RD_DIS_COLOR_MODE   _SFR_IO8(BASE_ADDR+0x13)
#define VGA_RD_DIS_FONT_ID _SFR_IO8(BASE_ADDR+0x14)

#define CURSOR_ON             (0x01)
#define CURSOR_OFF            (0x00)
#define CURSOR_BLOCK          (0x02)
#define CURSOR_BLINK          (0x04)

enum LineParts {hBar=0,vBar,lTee,rTee,uTee,bTee,cross,ulC,urC,blC,brC};
const uint8_t fontLines1[11] = {0xCA,0xC5,0xCD,0xC7,0xCB,0xCE,0xCF,0xC6,0xCC,0xC3,0xC9};
const uint8_t fontLinesX[11] = {'-','|','+','+','+','+','+','+','+','+','+'};

class VnVGA {
private:
  uint8_t  slaveSelectPin;
  boolean  rawMode;
  const uint8_t *fontLines;

public:
  VnVGA(void);

  void Init(void);
  void SetFG(uint8_t color);
  void SetBG(uint8_t color);
  void SetInv(uint8_t flag);
  void FillScreen(byte fillChar);
  void Cls(void) {FillScreen(' ');};
  void SetLoc(uint16_t offset);
  void SetLoc(uint16_t row, uint16_t col);
  void CursorStyle(uint8_t style);
  void EmitChar(char c) {VGA_WR_EMIT = c;};
  void OutChar(char c) {VGA_WR_EMIT_INCR = c;};
  void OutStr(char *s);
  void Box(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, boolean dl=false); 
  void hLine(uint8_t x1, uint8_t x2, uint8_t y, boolean dl=false); 
  void vLine(uint8_t x, uint8_t y1, uint8_t y2, boolean dl=false); 
  uint8_t toLineChar(LineParts n) {return fontLines[n];};
  
  uint8_t GetBusy(void) {return (VGA_RD_BUSY);};
  uint8_t GetCursorStyle(void) {return (VGA_RD_CURS_CNTL);};
  uint8_t GetFGColor(void) {return (VGA_RD_FG_COLOR);};
  uint8_t GetBGColor(void) {return (VGA_RD_BG_COLOR);};
  uint8_t GetInv(void) {return (VGA_RD_INVERSE);};
  uint8_t GetDisCols(void) {return (VGA_RD_DIS_COLS);};
  uint8_t GetDisRows(void) {return (VGA_RD_DIS_ROWS);};
  uint8_t GetDisColorBits(void) {return (VGA_RD_DIS_COLOR_BITS);};
  uint8_t GetDisColorMode(void) {return (VGA_RD_DIS_COLOR_MODE);};
  uint8_t GetDisFontId(void) {return (VGA_RD_DIS_FONT_ID);};
  uint8_t GetChar(void) {return (VGA_RD_CHAR);};
  uint8_t GetCharIncr(void) {return (VGA_RD_CHAR_INCR);};
  uint16_t GetLoc(void) {return (((uint16_t)VGA_RD_POS_HI)<<8)+VGA_RD_POS_LO;};

private:
//  void wrReg(uint8_t addr, uint8_t data);
//  uint8_t rd(uint8_t addr);
  
};

VnVGA::VnVGA(void) {
  rawMode        = true;
  if (GetDisFontId()==1)
    fontLines  = fontLines1;
  else
    fontLines  = fontLinesX;
}

void VnVGA::FillScreen(byte fillChar) {
  VGA_WR_FILL = fillChar;
  while(GetBusy()==1) ;
}

void VnVGA::SetLoc(uint16_t offset) {
  VGA_WR_POS_HI = (offset>>8)&0xFF;
  VGA_WR_POS_LO = offset&0xFF;
}

void VnVGA::SetLoc(uint16_t row, uint16_t col) {
  SetLoc(row*GetDisCols()+col);
//  SetLoc(row*128 +col);
}

void VnVGA::CursorStyle(uint8_t style) {
  VGA_WR_CURS_CNTL = style;
}

void VnVGA::OutStr(char *s) {
  while (*s != 0) {
    OutChar(*s++);
  }
}

void VnVGA::SetFG(uint8_t color) {
  VGA_WR_FG_COLOR = color;
}

void VnVGA::SetBG(uint8_t color) {
  VGA_WR_BG_COLOR = color;
}

void VnVGA::SetInv(uint8_t flag) {
  VGA_WR_INVERSE = flag;
}

void VnVGA::hLine(uint8_t x1, uint8_t x2, uint8_t y, boolean dl) {
  uint8_t  i, n;

  if (x2<=x1) return;
  
  n = x2-x1+1;
  SetLoc(y,x1);
  for (i=0; i<n; i++)
    OutChar(fontLines[hBar]);
}

void VnVGA::vLine(uint8_t x, uint8_t y1, uint8_t y2, boolean dl) {
  uint8_t  i, n;

  if (y2<=y1) return;

  n = y2-y1+1;
  for (i=0; i<n; i++) {
    SetLoc(y1+i,x);
    OutChar(fontLines[vBar]);
  }
}

void VnVGA::Box(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, boolean dl) {
  uint8_t  i, n;
  
  if (x2<=x1 || y2<=y1) return;
  
  SetLoc(y1,x1);
  OutChar(fontLines[ulC]);
  n = x2-x1-1;
  for (i=0; i<n; i++)
    OutChar(fontLines[hBar]);
  OutChar(fontLines[urC]);

  n = y2-y1-1;
  for (i=0; i<n; i++) {
    SetLoc(y1+i+1,x1);
    OutChar(fontLines[vBar]);
    SetLoc(y1+i+1,x2);
    OutChar(fontLines[vBar]);
  }

  SetLoc(y2,x1);
  OutChar(fontLines[blC]);
  n = x2-x1-1;
  for (i=0; i<n; i++)
    OutChar(fontLines[hBar]);
  OutChar(fontLines[brC]);
}

////////////////////////////////////////

VnVGA vga;

const char hexLut[] = "0123456789ABCDEF";

void OutHex2 (uint8_t n) {
  vga.OutChar(hexLut[(n>>4)&0xF]);
  vga.OutChar(hexLut[n&0xF]);
}

void OutHex4 (uint16_t n) {
  OutHex2((n>>8)&0xFF);
  OutHex2( n    &0xFF);
}

void repeatStr(char *s, uint8_t n) {
  for (uint8_t i=0; i<n; i++)
    vga.OutStr(s);
}

#define LIFE_MAX_COLS (256)
#define LIFE_LIVE_CHAR (0x0C)
#define LIFE_NEW_CHAR (0x04)
#define LIFE_LIVE_COLOR (0x2)
#define LIFE_NEW_COLOR (0x3)
#define LIFE_DEAD_CHAR (' ')

uint8_t lifeNewColor, lifeLiveColor;
uint8_t lifeNewChar,  lifeLiveChar;

uint8_t bufs[3][LIFE_MAX_COLS];

void _LifeUpdateBuffer(uint8_t *buf, uint8_t row)
{
  uint16_t  i, cols;
  uint8_t   n;
  
  cols = vga.GetDisCols();
  vga.SetLoc(row,0);
  for(i=0; i<cols; i++) {
    n = (vga.GetCharIncr()==LIFE_DEAD_CHAR?0:1);
    if (n==1) {
      if (buf[i]==2 || buf[i]==3) {
        // the cell lives on
        buf[i] = lifeLiveChar;
      } else {
        // the cell died :-(
        buf[i] = LIFE_DEAD_CHAR;
      }
    } else {
      if (buf[i]==3) {
        // a new cell has been born :-)
        buf[i] = lifeNewChar;
      } else {
        buf[i] = LIFE_DEAD_CHAR;
      }
    }
  }
}

void LifeComputeLastLine(uint8_t *buf0, uint8_t *buf, uint8_t row)
{
  uint16_t  i, cols, rows, oa, o;
  uint8_t   n, na, nb, *p;

  cols = vga.GetDisCols();

  oa = cols * (row-1);
  o  = cols *  row;
  for(i=0,p=buf; i<cols; i++)
    *p++ = 0;

  for(i=0; i<cols; i++) {
    // add in the values from the line above
    vga.SetLoc(oa);
    vga.GetChar();    // extra access to give time for the location register to update
    na = (vga.GetChar()==LIFE_DEAD_CHAR?0:1);
    vga.SetLoc(o);
    vga.GetChar();    // extra access to give time for the location register to update
    n = (vga.GetChar()==LIFE_DEAD_CHAR?0:1);
    nb = (buf0[i]==LIFE_DEAD_CHAR?0:1);
    oa++;
    o++;

    if (i==0) {
      buf[cols-1] += (na+n+nb);
    } else {
      buf[i-1] += (na+n+nb);
    }
    buf[i] += (na+nb);
    if (i==cols-1) {
      buf[0] += (na+n+nb);
    } else {
      buf[i+1] +=(na+n+nb);
    }
  }

  _LifeUpdateBuffer(buf, row);
}

void LifeComputeLine(uint8_t *buf, uint8_t row)
{
  uint16_t  i, cols, rows, oa, o, ob;
  uint8_t   n, na, nb, *p;

  cols = vga.GetDisCols();
  rows = vga.GetDisRows();

  if (row==0) {
    oa = cols * (rows-1);
  } else {
    oa = cols * (row-1);
  }
  o  = cols * row;
  ob = cols * (row+1);
  for(i=0,p=buf; i<cols; i++)
    *p++ = 0;

  for(i=0; i<cols; i++) {
    // value from the line above
    vga.SetLoc(oa);
    vga.GetChar();    // extra access to give time for the location register to update
    na = (vga.GetChar()==LIFE_DEAD_CHAR?0:1);
    // value from the current line
    vga.SetLoc(o);
    vga.GetChar();    // extra access to give time for the location register to update
    n = (vga.GetChar()==LIFE_DEAD_CHAR?0:1);
    // value from the line below
    vga.SetLoc(ob);
    vga.GetChar();    // extra access to give time for the location register to update
    nb = (vga.GetChar()==LIFE_DEAD_CHAR?0:1);
    oa++;
    o++;
    ob++;

    if (i==0) {
      buf[cols-1] += (na+n+nb);
    } else {
      buf[i-1] += (na+n+nb);
    }
    buf[i] += (na+nb);
    if (i==cols-1) {
      buf[0] += (na+n+nb);
    } else {
      buf[i+1] +=(na+n+nb);
    }
  }
  _LifeUpdateBuffer(buf, row);
}

void _Glider(uint8_t row, uint8_t col)
{
  vga.SetLoc(row, col);
  vga.EmitChar(LIFE_NEW_CHAR);
  vga.SetLoc(row+1, col+1);
  vga.EmitChar(LIFE_NEW_CHAR);
  vga.SetLoc(row+1, col+2);
  vga.EmitChar(LIFE_NEW_CHAR);
  vga.SetLoc(row+2, col);
  vga.EmitChar(LIFE_NEW_CHAR);
  vga.SetLoc(row+2, col+1);
  vga.EmitChar(LIFE_NEW_CHAR);
}

void Life(uint16_t iterations=400)
{
  uint16_t scr, n, o, i, j, c, cnt;
  uint8_t  rows, cols, *p;
  
//uint8_t lifeNewColor, lifeLiveColor;
//uint8_t lifeNewChar,  lifeLiveChar;
  n = vga.GetDisFontId();
  if (n==1 || n==2) {
    lifeNewChar = LIFE_NEW_CHAR;
    lifeLiveChar = LIFE_LIVE_CHAR;
  } else {
    lifeNewChar = '+';
    lifeNewChar = '*';
  }
  if (vga.GetDisColorMode() ==1) {
    // each display char can be a differnt color
    lifeNewColor = LIFE_NEW_COLOR;
    lifeLiveColor = LIFE_LIVE_COLOR;
  } else {
    lifeNewColor = LIFE_LIVE_COLOR;
    lifeLiveColor = LIFE_LIVE_COLOR;
  }
  
  rows = vga.GetDisRows();
  cols = vga.GetDisCols();
  vga.SetBG(0x0);
  vga.SetFG(lifeNewColor);
  vga.CursorStyle(CURSOR_OFF);
  vga.Cls();
  
  // Gliders
//  _Glider(10,10);
//  _Glider(40,110);

  // random starting cells
  scr = vga.GetDisCols()*vga.GetDisRows();
  n = scr/8;
  for (i=0; i<n; i++) {
    o = rand()%scr;
    vga.SetLoc(o);
    vga.EmitChar(lifeNewChar);
  }

  for (cnt=0; cnt<iterations; cnt++) {
    // save the first row
    vga.SetLoc(0);
    for(i=0,p=bufs[2]; i<cols; i++) {
      *p++ = vga.GetCharIncr();
    }
  
    LifeComputeLine(bufs[0], 0);
    for(j=1; j<rows; j++) {
      if (j==rows-1)
        LifeComputeLastLine(bufs[2], bufs[j%2], j);
      else
        LifeComputeLine(bufs[j%2], j);
      // write previous line buffer to the screen
      vga.SetLoc(j-1,0);
      for(i=0,p=bufs[(j-1)%2]; i<cols; i++) {
        c = *p++;
        if (c == lifeNewChar) {
          vga.SetFG(lifeNewColor);
        } else if (c == lifeLiveChar) {
          vga.SetFG(lifeLiveColor);
          c = lifeNewChar;
        }
        vga.OutChar(c);
      }
    }
    // write out the last line
    vga.SetLoc(rows-1,0);
    for(i=0,p=bufs[(rows-1)%2]; i<cols; i++) {
      c = *p++;
      if (c == lifeNewChar) {
        vga.SetFG(lifeNewColor);
      } else if (c == lifeLiveChar) {
        vga.SetFG(lifeLiveColor);
        c = lifeNewChar;
      }
      vga.OutChar(c);
    }
    delay(50);
  }
/*
  vga.SetLoc(scr/2);
  for(i=0; i<vga.GetDisCols(); i++) {
    OutHex2(bufs[0][i]);
    vga.OutStr("  ");
  }
  */
}

void setup()
{
//  Serial.begin(9600);
  srand(42);
}

void loop() 
{
  uint8_t  cs, n, m, i, j, c, off;
  
  while(vga.GetBusy()==1) ;
  
  vga.SetBG(0x00);
  vga.SetFG(0x04);
  vga.Cls();
  vga.SetLoc(0x01, 0x01);
  vga.OutStr("   Display Columns : 0x");
  OutHex2(vga.GetDisCols());
  vga.SetLoc(0x02, 0x01);
  vga.OutStr("      Display Rows : 0x");
  OutHex2(vga.GetDisRows());
  vga.SetLoc(0x03, 0x01);
  vga.OutStr("Display Color Bits : 0x");
  OutHex2(vga.GetDisColorBits());
  vga.SetLoc(0x04, 0x01);
  vga.OutStr("Display Color Mode : 0x");
  OutHex2(vga.GetDisColorMode());
  vga.SetLoc(0x05, 0x01);
  vga.OutStr("   Display Font ID : 0x");
  OutHex2(vga.GetDisFontId());
  vga.SetFG(0x07);
  vga.Box(0,0,26,6);
  delay(4000);

  Life(300);

  if (vga.GetDisFontId()==1) {
    // Draw a lot of boxes
    vga.Cls();
    vga.CursorStyle(CURSOR_OFF);
    // set BG color to black
    vga.SetBG(0x00);
    // set FG color to green
    vga.SetFG(0x02);
    
    n = vga.GetDisCols()-1;
    m = vga.GetDisRows()-1;
    c=1;
    for (i=0; i<vga.GetDisRows()/2; i++,n--, m--) {
      if (vga.GetDisColorMode() == 1) {
        vga.SetFG(c);
      }
      vga.Box(i,i,n,m);
      c++;
      if (c==1<<vga.GetDisColorBits()) {
          c =1;
      }
    }
    delay(5000);
  }

  // demo the HW cursor
  vga.SetFG(0x03);
  vga.Cls();
  vga.CursorStyle(CURSOR_ON|CURSOR_BLINK);
  vga.SetLoc(5,0);
  vga.OutStr("This is the blinking underline HW cursor");
  delay(1000);
  for (i=0; i<16; i++) {
    vga.SetLoc(5,i);
    delay(200);
  }
  delay(1000);

  vga.Cls();
  vga.CursorStyle(CURSOR_ON|CURSOR_BLINK|CURSOR_BLOCK);
  vga.SetLoc(6,0);
  vga.OutStr("This is the blinking block HW cursor");
  delay(1000);
  for (i=0; i<16; i++) {
    vga.SetLoc(6,i);
    delay(200);
  }
  delay(1000);

  vga.Cls();
  vga.CursorStyle(CURSOR_ON);
  vga.SetLoc(7,0);
  vga.OutStr("This is the non-blinking underline HW cursor");
  delay(1000);
  for (i=0; i<16; i++) {
    vga.SetLoc(7,i);
    delay(200);
  }
  delay(1000);

  vga.Cls();
  vga.CursorStyle(CURSOR_ON|CURSOR_BLOCK);
  vga.SetLoc(8,0);
  vga.OutStr("This is the non-blinking block HW cursor");
  delay(1000);
  for (i=0; i<16; i++) {
    vga.SetLoc(8,i);
    delay(200);
  }
  delay(1000);

  // Display a nice looking ASCII Table
  off = 3;
  vga.Cls();
  vga.CursorStyle(CURSOR_OFF);
  if (vga.GetDisColorMode() == 1) {
    vga.SetBG(0x06);
    vga.SetFG(0x00);
  } else {
    vga.SetBG(0x00);
    vga.SetFG(0x06);
    vga.SetInv(1);
  }
  n = (vga.GetDisCols()-(8*8+7))/2+1;
  m = (vga.GetDisFontId()==1?32:16);
  vga.SetLoc(off+1, n);
  repeatStr(" ", 24);
  vga.OutStr("--== ASCII Table ==--");
  repeatStr(" ", 24);
  vga.SetBG(0x00);
  vga.SetFG(0x06);
  vga.SetInv(0x00);
  
  for (i=0; i<m; i++) {
    vga.SetLoc(i+off+3, n);
    for (j=0; j<8; j++) {
      OutHex2(i+j*m);
      vga.OutStr(" : ");
      vga.OutChar(i+j*m);
      vga.OutStr("   ");
    }
  }
  vga.Box(n-2, off, n+70, off+2+m+1);
  vga.hLine(n-1, n+69, off+2);
  vga.SetLoc(off+2, n-2);
  vga.OutChar(vga.toLineChar(rTee)); 
  vga.SetLoc(off+2, n+70);
  vga.OutChar(vga.toLineChar(lTee)); 
  for (i=0; i<7; i++) {
    j = n+(i*9)+7;
    vga.vLine(j, off+3, off+2+m);
    vga.SetLoc(off+2, j);
    vga.OutChar(vga.toLineChar(bTee)); 
    vga.SetLoc(off+3+m, j);
    vga.OutChar(vga.toLineChar(uTee)); 
  }
  delay(5000);

}

