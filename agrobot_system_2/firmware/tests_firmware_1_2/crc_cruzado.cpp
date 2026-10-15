// Lee tramas hex (una por linea), imprime el CRC8 que calcula control_model.h.
#include <cstdio>
#include <cstring>
#include "control_model.h"
int main(){
 char linea[64];
 while(fgets(linea,sizeof linea,stdin)){
  uint8_t b[16];int n=0;unsigned x;char*p=linea;int k;
  while(sscanf(p,"%2x%n",&x,&k)==1&&n<16){b[n++]=(uint8_t)x;p+=k;}
  printf("%02x\n",agro::crc8(b,n));
 }
}
