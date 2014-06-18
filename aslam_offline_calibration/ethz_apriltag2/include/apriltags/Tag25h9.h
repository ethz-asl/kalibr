/** Tag family with 35 distinct codes.
    bits: 25,  minimum hamming: 9,  minimum complexity: 8

    Max bits corrected       False positive rate
            0                    0.000104 %
            1                    0.002712 %
            2                    0.034004 %
            3                    0.273913 %
            4                    1.593411 %

    Generation time: 31.283000 s

    Hamming distance between pairs of codes (accounting for rotation):

       0  0
       1  0
       2  0
       3  0
       4  0
       5  0
       6  0
       7  0
       8  0
       9  156
      10  214
      11  120
      12  64
      13  29
      14  11
      15  1
      16  0
      17  0
      18  0
      19  0
      20  0
      21  0
      22  0
      23  0
      24  0
      25  0
**/

#pragma once

namespace AprilTags {

const unsigned long long t25h9[] =
  { 0x155cbf1LL, 0x1e4d1b6LL, 0x17b0b68LL, 0x1eac9cdLL, 0x12e14ceLL, 0x3548bbLL, 0x7757e6LL, 0x1065dabLL, 0x1baa2e7LL, 0xdea688LL, 0x81d927LL, 0x51b241LL, 0xdbc8aeLL, 0x1e50e19LL, 0x15819d2LL, 0x16d8282LL, 0x163e035LL, 0x9d9b81LL, 0x173eec4LL, 0xae3a09LL, 0x5f7c51LL, 0x1a137fcLL, 0xdc9562LL, 0x1802e45LL, 0x1c3542cLL, 0x870fa4LL, 0x914709LL, 0x16684f0LL, 0xc8f2a5LL, 0x833ebbLL, 0x59717fLL, 0x13cd050LL, 0xfa0ad1LL, 0x1b763b0LL, 0xb991ceLL };

static const TagCodes tagCodes25h9 = TagCodes(25, 9, t25h9, sizeof(t25h9)/sizeof(t25h9[0]));

}
