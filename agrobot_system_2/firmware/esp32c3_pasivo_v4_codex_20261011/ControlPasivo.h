#pragma once
#include <cmath>
#include <cstdint>
namespace pasivo {
// Unlock only after mapping, driver PWM truth table and geometry verification.
constexpr bool PROFILE_VERIFIED=false;
constexpr float WHEELBASE_M=.37f;
constexpr float REAR_OUTSIDE_M=.37f, FRONT_OUTSIDE_M=.19f;
constexpr float TRACK_CENTERS_M=.30f; // Placeholder, NOT measured!
constexpr float MAX_V=.12f, MAX_REVERSE=.06f;
// Operational margin below mechanical +/-20 deg. Does NOT measure passive yaw.
constexpr float MAX_CURVATURE=.826299f; // tan(17 deg)/0.37, approx.
inline bool newer(uint16_t n,uint16_t old){uint16_t d=n-old;return d!=0 && d<32768;}
inline float clamp(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
inline float slew(float v,float goal,float step){return v+clamp(goal-v,-step,step);}
struct Target{float v,w,left,right;};
inline Target mix(float v,float w){
 if(!std::isfinite(v)||!std::isfinite(w))return {0,0,0,0};
 v=clamp(v,-MAX_REVERSE,MAX_V);
 if(std::fabs(v)<.002f)return {0,0,0,0};
 w=clamp(w,-std::fabs(v)*MAX_CURVATURE,std::fabs(v)*MAX_CURVATURE);
 return {v,w,v-w*TRACK_CENTERS_M*.5f,v+w*TRACK_CENTERS_M*.5f};
}
}
