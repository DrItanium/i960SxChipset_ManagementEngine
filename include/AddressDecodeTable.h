//
// Created by jwscoggins on 5/29/21.
//

#ifndef I960SXCHIPSET_ADDRESSDECODETABLE_H
#define I960SXCHIPSET_ADDRESSDECODETABLE_H
#include <Arduino.h>
constexpr uint32_t DecodeTable[4][256] = {
{
0x0,
0x1,
0x10,
0x11,
0x100,
0x101,
0x110,
0x111,
0x1000,
0x1001,
0x1010,
0x1011,
0x1100,
0x1101,
0x1110,
0x1111,
0x10000,
0x10001,
0x10010,
0x10011,
0x10100,
0x10101,
0x10110,
0x10111,
0x11000,
0x11001,
0x11010,
0x11011,
0x11100,
0x11101,
0x11110,
0x11111,
0x100000,
0x100001,
0x100010,
0x100011,
0x100100,
0x100101,
0x100110,
0x100111,
0x101000,
0x101001,
0x101010,
0x101011,
0x101100,
0x101101,
0x101110,
0x101111,
0x110000,
0x110001,
0x110010,
0x110011,
0x110100,
0x110101,
0x110110,
0x110111,
0x111000,
0x111001,
0x111010,
0x111011,
0x111100,
0x111101,
0x111110,
0x111111,
0x1000000,
0x1000001,
0x1000010,
0x1000011,
0x1000100,
0x1000101,
0x1000110,
0x1000111,
0x1001000,
0x1001001,
0x1001010,
0x1001011,
0x1001100,
0x1001101,
0x1001110,
0x1001111,
0x1010000,
0x1010001,
0x1010010,
0x1010011,
0x1010100,
0x1010101,
0x1010110,
0x1010111,
0x1011000,
0x1011001,
0x1011010,
0x1011011,
0x1011100,
0x1011101,
0x1011110,
0x1011111,
0x1100000,
0x1100001,
0x1100010,
0x1100011,
0x1100100,
0x1100101,
0x1100110,
0x1100111,
0x1101000,
0x1101001,
0x1101010,
0x1101011,
0x1101100,
0x1101101,
0x1101110,
0x1101111,
0x1110000,
0x1110001,
0x1110010,
0x1110011,
0x1110100,
0x1110101,
0x1110110,
0x1110111,
0x1111000,
0x1111001,
0x1111010,
0x1111011,
0x1111100,
0x1111101,
0x1111110,
0x1111111,
0x10000000,
0x10000001,
0x10000010,
0x10000011,
0x10000100,
0x10000101,
0x10000110,
0x10000111,
0x10001000,
0x10001001,
0x10001010,
0x10001011,
0x10001100,
0x10001101,
0x10001110,
0x10001111,
0x10010000,
0x10010001,
0x10010010,
0x10010011,
0x10010100,
0x10010101,
0x10010110,
0x10010111,
0x10011000,
0x10011001,
0x10011010,
0x10011011,
0x10011100,
0x10011101,
0x10011110,
0x10011111,
0x10100000,
0x10100001,
0x10100010,
0x10100011,
0x10100100,
0x10100101,
0x10100110,
0x10100111,
0x10101000,
0x10101001,
0x10101010,
0x10101011,
0x10101100,
0x10101101,
0x10101110,
0x10101111,
0x10110000,
0x10110001,
0x10110010,
0x10110011,
0x10110100,
0x10110101,
0x10110110,
0x10110111,
0x10111000,
0x10111001,
0x10111010,
0x10111011,
0x10111100,
0x10111101,
0x10111110,
0x10111111,
0x11000000,
0x11000001,
0x11000010,
0x11000011,
0x11000100,
0x11000101,
0x11000110,
0x11000111,
0x11001000,
0x11001001,
0x11001010,
0x11001011,
0x11001100,
0x11001101,
0x11001110,
0x11001111,
0x11010000,
0x11010001,
0x11010010,
0x11010011,
0x11010100,
0x11010101,
0x11010110,
0x11010111,
0x11011000,
0x11011001,
0x11011010,
0x11011011,
0x11011100,
0x11011101,
0x11011110,
0x11011111,
0x11100000,
0x11100001,
0x11100010,
0x11100011,
0x11100100,
0x11100101,
0x11100110,
0x11100111,
0x11101000,
0x11101001,
0x11101010,
0x11101011,
0x11101100,
0x11101101,
0x11101110,
0x11101111,
0x11110000,
0x11110001,
0x11110010,
0x11110011,
0x11110100,
0x11110101,
0x11110110,
0x11110111,
0x11111000,
0x11111001,
0x11111010,
0x11111011,
0x11111100,
0x11111101,
0x11111110,
0x11111111,
},
{
0x0,
0x2,
0x20,
0x22,
0x200,
0x202,
0x220,
0x222,
0x2000,
0x2002,
0x2020,
0x2022,
0x2200,
0x2202,
0x2220,
0x2222,
0x20000,
0x20002,
0x20020,
0x20022,
0x20200,
0x20202,
0x20220,
0x20222,
0x22000,
0x22002,
0x22020,
0x22022,
0x22200,
0x22202,
0x22220,
0x22222,
0x200000,
0x200002,
0x200020,
0x200022,
0x200200,
0x200202,
0x200220,
0x200222,
0x202000,
0x202002,
0x202020,
0x202022,
0x202200,
0x202202,
0x202220,
0x202222,
0x220000,
0x220002,
0x220020,
0x220022,
0x220200,
0x220202,
0x220220,
0x220222,
0x222000,
0x222002,
0x222020,
0x222022,
0x222200,
0x222202,
0x222220,
0x222222,
0x2000000,
0x2000002,
0x2000020,
0x2000022,
0x2000200,
0x2000202,
0x2000220,
0x2000222,
0x2002000,
0x2002002,
0x2002020,
0x2002022,
0x2002200,
0x2002202,
0x2002220,
0x2002222,
0x2020000,
0x2020002,
0x2020020,
0x2020022,
0x2020200,
0x2020202,
0x2020220,
0x2020222,
0x2022000,
0x2022002,
0x2022020,
0x2022022,
0x2022200,
0x2022202,
0x2022220,
0x2022222,
0x2200000,
0x2200002,
0x2200020,
0x2200022,
0x2200200,
0x2200202,
0x2200220,
0x2200222,
0x2202000,
0x2202002,
0x2202020,
0x2202022,
0x2202200,
0x2202202,
0x2202220,
0x2202222,
0x2220000,
0x2220002,
0x2220020,
0x2220022,
0x2220200,
0x2220202,
0x2220220,
0x2220222,
0x2222000,
0x2222002,
0x2222020,
0x2222022,
0x2222200,
0x2222202,
0x2222220,
0x2222222,
0x20000000,
0x20000002,
0x20000020,
0x20000022,
0x20000200,
0x20000202,
0x20000220,
0x20000222,
0x20002000,
0x20002002,
0x20002020,
0x20002022,
0x20002200,
0x20002202,
0x20002220,
0x20002222,
0x20020000,
0x20020002,
0x20020020,
0x20020022,
0x20020200,
0x20020202,
0x20020220,
0x20020222,
0x20022000,
0x20022002,
0x20022020,
0x20022022,
0x20022200,
0x20022202,
0x20022220,
0x20022222,
0x20200000,
0x20200002,
0x20200020,
0x20200022,
0x20200200,
0x20200202,
0x20200220,
0x20200222,
0x20202000,
0x20202002,
0x20202020,
0x20202022,
0x20202200,
0x20202202,
0x20202220,
0x20202222,
0x20220000,
0x20220002,
0x20220020,
0x20220022,
0x20220200,
0x20220202,
0x20220220,
0x20220222,
0x20222000,
0x20222002,
0x20222020,
0x20222022,
0x20222200,
0x20222202,
0x20222220,
0x20222222,
0x22000000,
0x22000002,
0x22000020,
0x22000022,
0x22000200,
0x22000202,
0x22000220,
0x22000222,
0x22002000,
0x22002002,
0x22002020,
0x22002022,
0x22002200,
0x22002202,
0x22002220,
0x22002222,
0x22020000,
0x22020002,
0x22020020,
0x22020022,
0x22020200,
0x22020202,
0x22020220,
0x22020222,
0x22022000,
0x22022002,
0x22022020,
0x22022022,
0x22022200,
0x22022202,
0x22022220,
0x22022222,
0x22200000,
0x22200002,
0x22200020,
0x22200022,
0x22200200,
0x22200202,
0x22200220,
0x22200222,
0x22202000,
0x22202002,
0x22202020,
0x22202022,
0x22202200,
0x22202202,
0x22202220,
0x22202222,
0x22220000,
0x22220002,
0x22220020,
0x22220022,
0x22220200,
0x22220202,
0x22220220,
0x22220222,
0x22222000,
0x22222002,
0x22222020,
0x22222022,
0x22222200,
0x22222202,
0x22222220,
0x22222222,
},
{
0x0,
0x4,
0x40,
0x44,
0x400,
0x404,
0x440,
0x444,
0x4000,
0x4004,
0x4040,
0x4044,
0x4400,
0x4404,
0x4440,
0x4444,
0x40000,
0x40004,
0x40040,
0x40044,
0x40400,
0x40404,
0x40440,
0x40444,
0x44000,
0x44004,
0x44040,
0x44044,
0x44400,
0x44404,
0x44440,
0x44444,
0x400000,
0x400004,
0x400040,
0x400044,
0x400400,
0x400404,
0x400440,
0x400444,
0x404000,
0x404004,
0x404040,
0x404044,
0x404400,
0x404404,
0x404440,
0x404444,
0x440000,
0x440004,
0x440040,
0x440044,
0x440400,
0x440404,
0x440440,
0x440444,
0x444000,
0x444004,
0x444040,
0x444044,
0x444400,
0x444404,
0x444440,
0x444444,
0x4000000,
0x4000004,
0x4000040,
0x4000044,
0x4000400,
0x4000404,
0x4000440,
0x4000444,
0x4004000,
0x4004004,
0x4004040,
0x4004044,
0x4004400,
0x4004404,
0x4004440,
0x4004444,
0x4040000,
0x4040004,
0x4040040,
0x4040044,
0x4040400,
0x4040404,
0x4040440,
0x4040444,
0x4044000,
0x4044004,
0x4044040,
0x4044044,
0x4044400,
0x4044404,
0x4044440,
0x4044444,
0x4400000,
0x4400004,
0x4400040,
0x4400044,
0x4400400,
0x4400404,
0x4400440,
0x4400444,
0x4404000,
0x4404004,
0x4404040,
0x4404044,
0x4404400,
0x4404404,
0x4404440,
0x4404444,
0x4440000,
0x4440004,
0x4440040,
0x4440044,
0x4440400,
0x4440404,
0x4440440,
0x4440444,
0x4444000,
0x4444004,
0x4444040,
0x4444044,
0x4444400,
0x4444404,
0x4444440,
0x4444444,
0x40000000,
0x40000004,
0x40000040,
0x40000044,
0x40000400,
0x40000404,
0x40000440,
0x40000444,
0x40004000,
0x40004004,
0x40004040,
0x40004044,
0x40004400,
0x40004404,
0x40004440,
0x40004444,
0x40040000,
0x40040004,
0x40040040,
0x40040044,
0x40040400,
0x40040404,
0x40040440,
0x40040444,
0x40044000,
0x40044004,
0x40044040,
0x40044044,
0x40044400,
0x40044404,
0x40044440,
0x40044444,
0x40400000,
0x40400004,
0x40400040,
0x40400044,
0x40400400,
0x40400404,
0x40400440,
0x40400444,
0x40404000,
0x40404004,
0x40404040,
0x40404044,
0x40404400,
0x40404404,
0x40404440,
0x40404444,
0x40440000,
0x40440004,
0x40440040,
0x40440044,
0x40440400,
0x40440404,
0x40440440,
0x40440444,
0x40444000,
0x40444004,
0x40444040,
0x40444044,
0x40444400,
0x40444404,
0x40444440,
0x40444444,
0x44000000,
0x44000004,
0x44000040,
0x44000044,
0x44000400,
0x44000404,
0x44000440,
0x44000444,
0x44004000,
0x44004004,
0x44004040,
0x44004044,
0x44004400,
0x44004404,
0x44004440,
0x44004444,
0x44040000,
0x44040004,
0x44040040,
0x44040044,
0x44040400,
0x44040404,
0x44040440,
0x44040444,
0x44044000,
0x44044004,
0x44044040,
0x44044044,
0x44044400,
0x44044404,
0x44044440,
0x44044444,
0x44400000,
0x44400004,
0x44400040,
0x44400044,
0x44400400,
0x44400404,
0x44400440,
0x44400444,
0x44404000,
0x44404004,
0x44404040,
0x44404044,
0x44404400,
0x44404404,
0x44404440,
0x44404444,
0x44440000,
0x44440004,
0x44440040,
0x44440044,
0x44440400,
0x44440404,
0x44440440,
0x44440444,
0x44444000,
0x44444004,
0x44444040,
0x44444044,
0x44444400,
0x44444404,
0x44444440,
0x44444444,
},
{
0x0,
0x8,
0x80,
0x88,
0x800,
0x808,
0x880,
0x888,
0x8000,
0x8008,
0x8080,
0x8088,
0x8800,
0x8808,
0x8880,
0x8888,
0x80000,
0x80008,
0x80080,
0x80088,
0x80800,
0x80808,
0x80880,
0x80888,
0x88000,
0x88008,
0x88080,
0x88088,
0x88800,
0x88808,
0x88880,
0x88888,
0x800000,
0x800008,
0x800080,
0x800088,
0x800800,
0x800808,
0x800880,
0x800888,
0x808000,
0x808008,
0x808080,
0x808088,
0x808800,
0x808808,
0x808880,
0x808888,
0x880000,
0x880008,
0x880080,
0x880088,
0x880800,
0x880808,
0x880880,
0x880888,
0x888000,
0x888008,
0x888080,
0x888088,
0x888800,
0x888808,
0x888880,
0x888888,
0x8000000,
0x8000008,
0x8000080,
0x8000088,
0x8000800,
0x8000808,
0x8000880,
0x8000888,
0x8008000,
0x8008008,
0x8008080,
0x8008088,
0x8008800,
0x8008808,
0x8008880,
0x8008888,
0x8080000,
0x8080008,
0x8080080,
0x8080088,
0x8080800,
0x8080808,
0x8080880,
0x8080888,
0x8088000,
0x8088008,
0x8088080,
0x8088088,
0x8088800,
0x8088808,
0x8088880,
0x8088888,
0x8800000,
0x8800008,
0x8800080,
0x8800088,
0x8800800,
0x8800808,
0x8800880,
0x8800888,
0x8808000,
0x8808008,
0x8808080,
0x8808088,
0x8808800,
0x8808808,
0x8808880,
0x8808888,
0x8880000,
0x8880008,
0x8880080,
0x8880088,
0x8880800,
0x8880808,
0x8880880,
0x8880888,
0x8888000,
0x8888008,
0x8888080,
0x8888088,
0x8888800,
0x8888808,
0x8888880,
0x8888888,
0x80000000,
0x80000008,
0x80000080,
0x80000088,
0x80000800,
0x80000808,
0x80000880,
0x80000888,
0x80008000,
0x80008008,
0x80008080,
0x80008088,
0x80008800,
0x80008808,
0x80008880,
0x80008888,
0x80080000,
0x80080008,
0x80080080,
0x80080088,
0x80080800,
0x80080808,
0x80080880,
0x80080888,
0x80088000,
0x80088008,
0x80088080,
0x80088088,
0x80088800,
0x80088808,
0x80088880,
0x80088888,
0x80800000,
0x80800008,
0x80800080,
0x80800088,
0x80800800,
0x80800808,
0x80800880,
0x80800888,
0x80808000,
0x80808008,
0x80808080,
0x80808088,
0x80808800,
0x80808808,
0x80808880,
0x80808888,
0x80880000,
0x80880008,
0x80880080,
0x80880088,
0x80880800,
0x80880808,
0x80880880,
0x80880888,
0x80888000,
0x80888008,
0x80888080,
0x80888088,
0x80888800,
0x80888808,
0x80888880,
0x80888888,
0x88000000,
0x88000008,
0x88000080,
0x88000088,
0x88000800,
0x88000808,
0x88000880,
0x88000888,
0x88008000,
0x88008008,
0x88008080,
0x88008088,
0x88008800,
0x88008808,
0x88008880,
0x88008888,
0x88080000,
0x88080008,
0x88080080,
0x88080088,
0x88080800,
0x88080808,
0x88080880,
0x88080888,
0x88088000,
0x88088008,
0x88088080,
0x88088088,
0x88088800,
0x88088808,
0x88088880,
0x88088888,
0x88800000,
0x88800008,
0x88800080,
0x88800088,
0x88800800,
0x88800808,
0x88800880,
0x88800888,
0x88808000,
0x88808008,
0x88808080,
0x88808088,
0x88808800,
0x88808808,
0x88808880,
0x88808888,
0x88880000,
0x88880008,
0x88880080,
0x88880088,
0x88880800,
0x88880808,
0x88880880,
0x88880888,
0x88888000,
0x88888008,
0x88888080,
0x88888088,
0x88888800,
0x88888808,
0x88888880,
0x88888888,
},
};
#endif //I960SXCHIPSET_ADDRESSDECODETABLE_H
