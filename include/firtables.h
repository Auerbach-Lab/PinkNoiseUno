/*
  Arduino pink noise generator -- Mark Tillotson, 2018-12-09

  Works for Uno/Pro Mini/Mega, using pin 11 or 10 depending on board (the one driven by timer2 output A)

  Uses the highly efficient algorithm from Stenzel, see:
     http://stenzel.waldorfmusic.de/post/pink/
     https://github.com/Stenzel/newshadeofpink

  Note his request that if used in a commercial product, he is sent one of the units...

 */


int highfirtab [0x40] =
  {
    -2840,    2037,    -2174,    2703,    -2831,    2046,    -2165,    2712,
    -2735,    2141,    -2070,    2807,    -2726,    2150,    -2060,    2816,
    -2846,    2031,    -2180,    2696,    -2837,    2040,    -2171,    2705,
    -2742,    2135,    -2076,    2801,    -2733,    2144,    -2067,    2810,
    -2810,    2067,    -2144,    2733,    -2801,    2076,    -2135,    2742,
    -2705,    2171,    -2040,    2837,    -2696,    2180,    -2031,    2846,
    -2816,    2060,    -2150,    2726,    -2807,    2070,    -2141,    2735,
    -2712,    2165,    -2046,    2831,    -2703,    2174,    -2037,    2840
  };

int lowfirtab [0x40] =
  {
    -14,    -7,    4,    11,    -21,    -14,    -2,    5,
    -11,    -4,    7,    15,    -18,    -10,    1,    8,
    -16,    -9,    2,    9,     -23,    -16,    -4,    3,
    -13,    -6,    5,    13,    -20,    -12,    -1,    6,
    -6,     1,    12,    20,    -13,    -5,    6,    13,
    -3,     4,    16,    23,    -9,     -2,    9,    16,
    -8,    -1,    10,    18,    -15,    -7,    4,    11,
    -5,     2,    14,    21,    -11,    -4,    7,    14
  };
  
