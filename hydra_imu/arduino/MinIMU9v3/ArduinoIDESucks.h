/* -*- c++ -*- */
#ifndef ArduinoIDESucks_h
#define ArduinoIDESucks_h 1

/* The Arduino IDE does some "magic" preprocessing to do God knows what with a
   sketch before actually compiling it.

   Unfortunately, it completely fails for templates.  The bug was first
   reported February of 2011, but as of 1 May 2014 hasn't been fixed...
   <http://code.google.com/p/arduino/issues/detail?id=472>
 */

#include <stdint.h>
#include <limits.h>

/** Power function suitable for use with integers.
 *
 * @param x base value
 * @param y exponent
 * @param z scale factor
 */
template < typename _Tp, typename _TpB >
inline _TpB
power(_Tp x, _Tp y, _TpB z = 1)
{
  _TpB o = z;
  if ( y > 0 )
    for ( ; y > 0; --y )
      o *= x;
  else
    for ( ; y < 0; ++y )
      o /= x;
    
  return o;
}
  
#undef abs
template < typename _Tp >
struct make_unsigned;

template < > struct make_unsigned<int8_t> { typedef uint8_t type; };
template < > struct make_unsigned<uint8_t> { typedef uint8_t type; };
template < > struct make_unsigned<int16_t> { typedef uint16_t type; };
template < > struct make_unsigned<uint16_t> { typedef uint16_t type; };
template < > struct make_unsigned<int32_t> { typedef uint32_t type; };
template < > struct make_unsigned<uint32_t> { typedef uint32_t type; };

template < typename _Tp >
inline typename make_unsigned<_Tp>::type
abs(_Tp v)
{
  return v < 0
    ? -v
    : v;
}

template <>
inline uint16_t
abs(int16_t x)
{
  return static_cast<uint16_t>(x < 0 ? -x : x);
}

#define SCALE_UPGRADE_MARGIN (SHRT_MAX/16)
#define SCALE_DOWNGRADE_MARGIN (SHRT_MAX/4)

/* When the scale is changed for a given sensor, it takes this many samples
   before the change is reflected in the sensor's values.  */
#define SCALE_DOWN_DELAY 0
#define SCALE_UP_DELAY 0

/** Information on how to fetch or set a given sensor parameter. */
template < typename _Register = byte >
struct SensorParameter
{
  typedef _Register Register;
  Register reg;                 /**< register in which the parameter
                                        lives */
  byte
    mask,                       /**< Bitmask for the value*/
    pitch;                      /**< Bit-offset of the value within the
                                   register */

  SensorParameter(Register _reg, byte _mask, byte _pitch)
    : reg ( _reg ),
      mask ( _mask ),
      pitch ( _pitch )
  {
  }

  template < typename _Chip >
  inline void
  set(_Chip& chip, byte new_value)
  {
    chip.writeReg(reg, (chip.readReg(reg) & ~mask) | (new_value << pitch));
  }

  template < typename _Chip >
  inline byte
  get(_Chip& chip)
  {
    return ((chip.readReg(reg) & mask) >> pitch);
  }
};

/** Sensor manager. Handles dynamic scale selection. */
template < typename _Chip, typename _Reg = byte,
           typename _Vector = typename _Chip::vector >
struct Sensor
{
  typedef _Chip Chip;

  typedef _Vector Vector;
  typedef const Vector Chip::*VectorPtr;
  typedef void (Chip::*ReadFunction)(void);

  Chip& chip;
  SensorParameter<_Reg> rate;
  SensorParameter<_Reg> scale;
  ReadFunction readfn;
  VectorPtr vector;             /**< pointer to the vector-member to examine
                                   for dynamic scale management. */
  const uint16_t* downgrade_thresholds;

  byte
    selected_scale,           /**< scale index we've sent to the sensor chip */
    current_scale,            /**< scale index to use when reporting values */
    max_scale,                /**< maximum scale index available */
    scale_delay;              /**< number of remaining samples before we start
                                 using the selected scale as current */

  Sensor(Chip& _chip,
         SensorParameter<_Reg> _rate,
         SensorParameter<_Reg> _scale,
         byte _max_scale, const uint16_t *_downgrade_thresholds,
         ReadFunction _readfn, VectorPtr _vector)
    : chip ( _chip ),
      rate ( _rate ),
      scale ( _scale ),
      readfn ( _readfn ),
      vector ( _vector ),
      downgrade_thresholds ( _downgrade_thresholds ),
      selected_scale ( 0 ),
      current_scale ( 0 ),
      max_scale ( _max_scale ),
      scale_delay ( 0 )
  {}

  /** Fetch the next value from the hardware */
  template < typename _Tp >
  void
  next(_Tp& vdest, uint8_t& sdest)
  {
    Vector prev = chip.*vector;

    (chip.*readfn)();
    const Vector& v ( chip.*vector);

    vdest.x = v.x;
    vdest.y = v.y;
    vdest.z = v.z;
    sdest = next_scale();

    check_upgrade(v, prev);
  }

  /** Check if the scale should be increased, and do so if necessary.  This
   * function currently uses a first-order linear-extrapolation prediction
   * heuristic.
   */
  void
  check_upgrade(const Vector& v, const Vector& vp)
  {
    if ( current_scale == selected_scale && current_scale < max_scale )
      {
        /* Vector delta ( v - vp ); */
        long int threshold ( SHRT_MAX - SCALE_UPGRADE_MARGIN );
        if ( abs(v.x) >= threshold ||
             abs(v.y) >= threshold ||
             abs(v.z) >= threshold )
          set_scale(current_scale + 1);
      }
  }

  /** Check if the scale can be safely decreased, and do so if it can be. */
  bool
  check_downgrade(bool allow_scale_change = true)
  {
    if ( (current_scale == selected_scale && current_scale > 0) || ! allow_scale_change )
      {
        int16_t d = 3 * downgrade_thresholds[current_scale] / 4;
        const Vector& v ( chip.*vector);
        if ( abs(v.x) < d &&
             abs(v.y) < d &&
             abs(v.z) < d )
          {
            if ( allow_scale_change )
              set_scale(current_scale - 1);
            return true;
          }
      }

    return false;
  }


  /** Fetch the current scale index directly from hardware. */
  inline byte
  fetch_scale()
  { return (selected_scale = scale.get(chip)); }


  /** Set a new scale index. */
  inline void
  set_scale(byte new_scale)
  {
    scale.set(chip, new_scale);
    scale_delay = (new_scale > selected_scale
                   ? SCALE_UP_DELAY
                   : SCALE_DOWN_DELAY );
    selected_scale = new_scale;

  }

  /** Get the scale index to report for the next sample. */
  byte
  next_scale()
  {
    byte o = current_scale;

    if ( ! scale_delay )
      current_scale = selected_scale;
    else
      --scale_delay;

    return o;
  }
};

template < typename _Vector >
inline _Vector
operator -(const _Vector& a, const _Vector& b)
{
  _Vector out;
  out.x = a.x - b.x;
  out.y = a.y - b.y;
  out.z = a.z - b.z;
  return out;
}

#endif  /* ArduinoIDESucks_h */
