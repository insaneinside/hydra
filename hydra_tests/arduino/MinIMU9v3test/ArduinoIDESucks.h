#ifndef ArduinoIDESucks_h
#define ArduinoIDESucks_h 1

/* The Arduino IDE does some "magic" preprocessing to do God knows what with a
   sketch before actually compiling it.

   Unfortunately, it completely fails for templates.  The bug was first
   reported February of 2011, but as of 1 May 2014 hasn't been fixed...
   <http://code.google.com/p/arduino/issues/detail?id=472>
 */

template <typename _Tp>
void
store(uint16_t** dest, const _Tp& src, unsigned char _n)
{
  dest[_n][0] = src.x;
  dest[_n][1] = src.y;
  dest[_n][2] = src.z;
}




#endif  /* ArduinoIDESucks_h */
