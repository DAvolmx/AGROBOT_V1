#include "ControlPasivo.h"
#include <cassert>
#include <cstdio>
int main(){
 using namespace pasivo;
 assert(newer(0,65535));assert(!newer(10,10));assert(!newer(9,10));assert(!newer(32768,0));
 assert(mix(0,1).left==0);assert(mix(NAN,0).v==0);
 for(int a=-100;a<=100;a++)for(int b=-100;b<=100;b++){
  auto t=mix(a*.0055f,b*.015f);
  assert(std::fabs(t.w)<=std::fabs(t.v)*MAX_CURVATURE+1e-6);
  assert(t.left*t.right>=0);assert(t.v<=MAX_V && t.v>=-MAX_REVERSE);
  assert(std::fabs((t.right-t.left)-t.w*TRACK_CENTERS_M)<1e-6);
 }
 assert(std::fabs(slew(0,1,.01f)-.01f)<1e-6);
 puts("PASS: 40401 command pairs, units, curvature, no counterrotation, sequence wrap, NaN, ramp");
}
