#pragma once

namespace AprilTags {

const unsigned long long t25h4[] =
{
   0x2cd4ddl, 0x2cd6d6l, 0x2cd5f4l, 0x2cd5cel, 0x2cd6ecl, 0x2cd4e7l, 0x2cd7c5l, 0x2cd4bel, 0x2cd79cl, 0x2cd597l, 0x2cd6b5l, 
};

static const TagCodes tagCodes25h4 = TagCodes(25,4,t25h4, sizeof(t25h4)/sizeof(t25h4[0]));

}
