#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"


#define CC 0x20
#define STARTUPVECTOR 0
#define TICKVECTOR 8

uint8_t memory[0x1800];
uint8_t *code = &memory[0];
uint32_t *stack = (uint32_t*) &memory[0x1400];
int32_t *globals = (int32_t*) &memory[0x13a0];

uint8_t *ip;
uint32_t *sp;
uint32_t *fp;

int accx,accy,accz;

uint32_t timert0 = 0;
uint32_t tickperiod = 0;
uint32_t lasttick = 0;

char buf[20];

void setup() {
  delay(2000);
  Serial.begin(9600);
  pinMode(13, OUTPUT); 
  digitalWrite(13, LOW);
}


/////////////////////////
// monitor
/////////////////////////

void loop() {
  if(Serial.available()) dispatch(Serial.read());
  if(tickperiod&&(millis()-lasttick>=tickperiod)){
    lasttick=millis();
    runVM(TICKVECTOR);
  }
}

void dispatch(uint8_t c){
  if(c==0xff) uputc(0x31);
  if(c==0xfe) readMemory();
  if(c==0xfd) writeMemory();
  if(c==0xfc) runVM(CC);
}

uint16_t read16(){
  uint8_t low = ugetc();
  uint8_t high = ugetc();
  return (high<<8)+low;
}

void readMemory(){
  uputc(memory[read16()]);
}

void writeMemory(){
  uint16_t addr = read16();
  uint16_t count = read16();
  for(int i=0;i<count;i++) memory[addr++]=ugetc();
}

uint8_t ugetc(){
  while(!Serial.available());
  return Serial.read();
}
void uputc(uint8_t c){
  Serial.write(c);
}

/////////////////////////
// uLogo Virtual Machine
/////////////////////////

void(*prims[])() = {
    eval_done, eval_byte, eval_num, eval_list, eval_eol,
    eval_lthing, eval_lmake,
    eval_gthing, eval_gmake, eval_ufun,
    prim_stop, prim_repeat, prim_if, prim_ifelse,
    prim_sum, prim_diff, prim_mul, prim_div, prim_mod,
    prim_equal, prim_ne, prim_greater, prim_less,
    prim_and, prim_or, prim_xor, prim_not, prim_lsl,
    prim_readb, prim_writeb, prim_read, prim_write,
    prim_wait, prim_resett, prim_timer, prim_startticker, prim_stopticker,
    prim_print, prim_prh, prim_prhb, prim_prs, prim_prf,
    prim_nop, prim_nop, prim_nop,
    prim_readimu, prim_accx, prim_accy, prim_accz,
    prim_ledon, prim_ledoff,
    prim_led0on, prim_led0off,
    prim_led1on, prim_led1off,
    prim_led2on, prim_led2off,
    prim_led3on, prim_led3off,
    prim_button
};

void runVM(uint16_t start){
  ip = &code[start];
  sp = stack;
  fp = 0;
  while(1){
    uint8_t token = *ip++;;
    if(token==0) break;
    (prims[token])();
  } 
}

void eval_done(){}

void eval_byte(){
    *sp++ = *ip++;
}

void eval_num(){
    int32_t t0 = *ip++;
    t0 += (*ip++<<8);
    t0 += (*ip++<<16);
    t0 += (*ip++<<24);
    *sp++ = t0;
}

void eval_list(){
    uint8_t offset = *ip++;
    offset += *ip++<<8;
    *sp++ = (int32_t)ip;
    ip+=offset;
}

void eval_eol(){
    switch(*--sp){
    case 0: ip=(uint8_t*)*--sp; break;
    case 1: eol_repeat(); break;
//    case 2: prim_loop(); break;
//    case 3: eol_runmacro(); break;
    }
}

uint32_t *get_local_address(){
    int8_t t0 = (int8_t) *ip++;
    if(t0<0) return fp-t0-1;
    else return fp-t0-4;
}

void eval_lthing(){
    *sp++ = *get_local_address();
}

void eval_lmake(){
    *get_local_address() = *--sp;
}

void eval_ufun(){
    uint16_t newip = *ip++;
    newip += *ip++<<8;
    *sp++ = (uint32_t)ip;
    ip = (uint8_t*)&memory[newip];
    *sp++ = *ip++;
    *sp++ = (int32_t)fp;
    fp = sp;
    sp+=*ip++;
}

void eval_gthing(){*sp++=globals[*ip++];}
void eval_gmake(){int32_t t0=*--sp; globals[*ip++]=t0;}

void prim_stop(){
    int32_t t0;
    if(fp==0) return;
    sp = fp;
    fp = (uint32_t*) *--sp;
    t0 = *--sp;
    ip = (uint8_t*)*--sp;
    sp-= t0;
}

void prim_repeat(){
    *sp++ = (uint32_t)ip;
    eol_repeat();
}

void eol_repeat(){
    if(*(sp-3)==0) {ip = (uint8_t*)(*--sp); sp-=2;}
    else {(*(sp-3))--; ip=(uint8_t*)(*(sp-2)); *sp++=1;}
}

void prim_if(){
    int32_t addr = *--sp;
    if(!*--sp) return;
    *sp++ = (int32_t)ip;
    *sp++ = 0;
    ip = (uint8_t*)addr;
}

void prim_ifelse(){
    int32_t addr;
    int32_t faddr = *--sp;
    int32_t taddr = *--sp;
    if(*--sp) addr = taddr;
    else addr = faddr;
    *sp++ = (int32_t)ip;
    *sp++ = 0;
    ip = (uint8_t*)addr;
}

void prim_sum(){int32_t  t0=*--sp; int32_t t1=*--sp; *sp++=t1+t0;}
void prim_diff(){int32_t  t0=*--sp; int32_t t1=*--sp; *sp++=t1-t0;}
void prim_mul(){int32_t  t0=*--sp; int32_t t1=*--sp; *sp++=t1*t0;}
void prim_div(){int32_t  t0=*--sp; int32_t t1=*--sp; *sp++=t1/t0;}

void prim_mod(){
    int32_t t0 = *--sp;
    int32_t res = (*--sp)%t0;
    *sp++=res;
}

void prim_equal(){int32_t  t0=*--sp; int32_t t1=*--sp; *sp++ = (int32_t)(t1==t0);}
void prim_ne(){int32_t  t0=*--sp; int32_t t1=*--sp; *sp++ = (int32_t)(t1!=t0);}
void prim_greater(){int32_t  t0=*--sp; int32_t t1=*--sp; *sp++ = (int32_t)(t1>t0);}
void prim_less(){int32_t  t0=*--sp; int32_t t1=*--sp; *sp++ = (int32_t)(t1<t0);}

void prim_lsl(){
    int32_t cnt = *--sp;
    int32_t n = *--sp;
    if(cnt<0) *sp++ = n>>-cnt;
    else *sp++ = n<<cnt;
}

void prim_and(){int32_t  t0=*--sp; int32_t t1=*--sp; *sp++=t1&t0;}
void prim_or(){int32_t  t0=*--sp; int32_t t1=*--sp; *sp++=t1|t0;}
void prim_xor(){int32_t  t0=*--sp; int32_t t1=*--sp; *sp++=t1^t0;}
void prim_not(){int32_t  t0=*--sp; if(t0) *sp++=0; else *sp++=1;}

void prim_readb(){
  uint8_t *t0 = &(memory[*--sp]);
  *sp++ = *t0;
}

void prim_read(){
  uint8_t *t0 = &(memory[*--sp]);
  *sp++ = (t0[3]<<24)+(t0[2]<<16)+(t0[1]<<8)+t0[0];
}

void prim_writeb(){
  int32_t t0 = *--sp;
  memory[*--sp] = (uint8_t)t0;
}

void prim_write(){
  int32_t t0 = *--sp;
  uint8_t *t1 = &(memory[*--sp]);
  t1[0] = (uint8_t) t0;
  t1[1] = (uint8_t)(t0>>8);
  t1[2] = (uint8_t)(t0>>16);
  t1[3] = (uint8_t)(t0>>24);
}

void prim_wait(){delay(*--sp);}
void prim_resett(){timert0=millis();}
void prim_timer(){*sp++=(millis()-timert0)&0x7fffffff;}

void prim_startticker(){tickperiod=*--sp;}
void prim_stopticker(){tickperiod=0;}

void prim_print(){sprintf(buf, "%d", (int)(*--sp)); Serial.println(buf);}
void prim_prh(){sprintf(buf, "%08x", (unsigned int)(*--sp)); Serial.println(buf);}
void prim_prhb(){sprintf(buf, "%02x", (unsigned int)((*--sp)&0xff)); Serial.println(buf);}
void prim_prs(){ Serial.println((char*)(*--sp));}

void prim_prf() {
  int n = (int)*--sp;
  char *s = (char*)*--sp;
  for (; *s; s++) {
    if (*s=='%'){
      s++;
      switch (*s){
          case 'b': sprintf(buf, "%02x", (n&0xff)); Serial.print(buf); break;
          case 'h': sprintf(buf, "%04x", (n&0xffff)); Serial.print(buf); break;
          case 'w': sprintf(buf, "%08x", n); Serial.print(buf); break;
          case 'd': sprintf(buf, "%d", n); Serial.print(buf); break;
          case 0: return;
          default: Serial.write((int)*s); break;
      }
    } else Serial.write((int)*s);
  }
}



/////////////////////////
// IMU
/////////////////////////

void prim_readimu(){}
void prim_accx(){*sp++=accx;}
void prim_accy(){*sp++=accy;}
void prim_accz(){*sp++=accz;}

/////////////////////////
// etc
/////////////////////////

void prim_ledon(){digitalWrite(13, HIGH);}
void prim_ledoff(){digitalWrite(13, LOW);}

void prim_led0on(){digitalWrite(5, HIGH);}
void prim_led0off(){digitalWrite(5, LOW);}
void prim_led1on(){digitalWrite(4, HIGH);}
void prim_led1off(){digitalWrite(4, LOW);}
void prim_led2on(){digitalWrite(3, HIGH);}
void prim_led2off(){digitalWrite(3, LOW);}
void prim_led3on(){digitalWrite(2, HIGH);}
void prim_led3off(){digitalWrite(2, LOW);}

void prim_button(){*sp++ = digitalRead(6)==0;}

void prim_nop(){}
