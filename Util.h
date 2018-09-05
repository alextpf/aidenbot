//========================================================================================================================================
// Some util functions...
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
//========================================================================================================================================
// Arduino abs function sometimes fail!
int16_t myAbs(int16_t param)
{
  if (param < 0)
    return -param;
  else
    return param;
}
//========================================================================================================================================
// Extract sign of a variable
int sign(int val)
{
  if (val < 0)
    return (-1);
  else
    return (1);
}