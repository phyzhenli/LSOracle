//# 14 inputs
//# 14 outputs
//# 18 D-type flipflops
//# 141 inverters
//# 388 gates (118 ANDs + 119 NANDs + 101 ORs + 50 NORs)

module s1196(clk,G0,G1,G10,G11,G12,G13,G2,G3,G4,G45,G5,G530,G532,G535,
  G537,G539,
  G542,G546,G547,G548,G549,G550,G551,G552,G6,G7,G8,G9);
input clk,G0,G1,G2,G3,G4,G5,G6,G7,G8,G9,G10,G11,G12,G13;
output G546,G539,G550,G551,G552,G547,G548,G549,G530,G45,G542,G532,G535,G537;

  wire G29,G502,G30,G503,G31,G504,G32,G505,G33,G506,G34,G507,G35,G508,G36,G509,
    G37,G510,G38,G511,G39,G512,G40,G513,G41,G514,G42,G515,G43,G516,G44,G517,
    G518,G46,G519,G520,G521,G522,G524,II156,G334,G527,G528,G529,G531,G533,G536,
    G538,G540,G541,G543,G476,G484,G125,G140,G132,G70,G67,G99,G475,G57,G59,G58,
    G228,G272,G271,G98,G97,G135,G134,II218,G333,G55,G54,G165,G72,G71,G236,G274,
    G275,II249,G370,G75,G74,G490,G190,G482,G241,G153,G192,G193,G123,G122,II272,
    G209,G458,II276,G238,G332,II280,G309,II287,G347,G498,G195,G78,G77,II295,
    G198,G459,G199,G200,G90,G89,G221,G222,G223,G224,II316,G239,G369,G234,G235,
    II327,G435,II330,G441,G50,G49,G130,G501,G156,G477,G276,G485,II352,G299,
    G497,G205,II371,G335,II374,G456,G87,G86,II386,G414,G486,G68,G231,G232,G160,
    G161,G265,G64,G63,G180,G181,G107,G207,G208,G167,G168,G124,G206,G203,G204,
    G489,G273,G495,G177,G357,G212,G213,II493,G218,G404,II502,G468,G173,G487,
    G534,II529,G149,II536,G79,G446,G494,G500,G214,G215,G492,G62,G483,G182,G282,
    G281,II573,G176,G403,II576,G175,G447,G479,G194,G491,G554,G553,G170,G171,
    G172,G525,G526,G493,G544,G545,G488,G499,G280,II624,G120,G303,G480,G179,
    II631,G188,G336,G496,G174,II662,G405,G478,G279,II692,G145,G432,G359,G469,
    G163,G461,G431,G362,G129,G81,G288,G240,G348,G352,G164,G379,G211,G385,G376,
    G387,G462,G436,G363,G410,G399,G437,G66,G229,G307,G104,G306,G283,G219,G339,
    G472,G136,G351,G169,G440,G382,G100,G386,G85,G321,G378,G471,G191,G103,G112,
    G377,G56,G358,G83,G400,G277,G308,G151,G411,G48,G413,G197,G201,G434,G373,
    G444,G361,G202,G346,G82,G457,G364,G109,G445,G53,G225,G412,G371,G267,G353,
    G92,G388,G114,G473,G143,G331,G257,G429,G51,G380,G93,G360,G106,G338,G337,
    G270,G340,G322,G105,G196,G330,G248,G249,G430,G344,G111,G189,G428,G227,G349,
    G108,G460,G115,G463,G148,G393,G127,G470,G341,G118,G342,G73,G324,G183,G323,
    G144,G354,G312,G315,G250,G251,G474,G242,G343,G147,G304,G52,G158,G398,G94,
    G365,G137,G417,G290,G117,G157,G327,G367,G126,G397,G101,G451,G187,G406,G418,
    G60,G453,G186,G289,G119,G311,G178,G402,G154,G433,G91,G449,G88,G452,G184,
    G329,G150,G291,G138,G155,G328,G102,G366,G372,G116,G383,G131,G392,G396,G76,
    G401,G110,G422,G80,G415,G146,G142,G425,G438,G133,G424,G439,G317,G159,G245,
    G426,G162,G443,G47,G416,G61,G427,G95,G442,G121,G423,G128,G448,G139,G419,
    G394,G407,G314,G395,G302,G355,G316,G350,G368,G381,G384,G389,G374,G286,G293,
    G375,G356,G313,G420,G421,G320,G310,G408,G305,G409,G296,G325,G464,G391,G220,
    G292,G345,G226,G465,G210,G454,G269,G287,G318,G326,G390,G298,G300,G261,G301,
    G297,G455,G152,G319,G284,G294,G141,G285,G295,G450,G244,G166,G252,G216,G263,
    G233,G243,G237,G96,G278,G255,G69,G264,G84,G258,G259,G217,G230,G260,G266,
    G262,G256,G113,G268,G253,G254,G523,G247,G246,G185;

  dff DFF_0(clk,G29,G502);
  dff DFF_1(clk,G30,G503);
  dff DFF_2(clk,G31,G504);
  dff DFF_3(clk,G32,G505);
  dff DFF_4(clk,G33,G506);
  dff DFF_5(clk,G34,G507);
  dff DFF_6(clk,G35,G508);
  dff DFF_7(clk,G36,G509);
  dff DFF_8(clk,G37,G510);
  dff DFF_9(clk,G38,G511);
  dff DFF_10(clk,G39,G512);
  dff DFF_11(clk,G40,G513);
  dff DFF_12(clk,G41,G514);
  dff DFF_13(clk,G42,G515);
  dff DFF_14(clk,G43,G516);
  dff DFF_15(clk,G44,G517);
  dff DFF_16(clk,G45,G518);
  dff DFF_17(clk,G46,G519);
  not NOT_0(G520,G0);
  not NOT_1(G521,G1);
  not NOT_2(G522,G2);
  not NOT_3(G524,G3);
  not NOT_4(II156,G4);
  not NOT_5(G334,II156);
  not NOT_6(G527,G4);
  not NOT_7(G528,G5);
  not NOT_8(G529,G6);
  not NOT_9(G531,G7);
  not NOT_10(G533,G8);
  not NOT_11(G536,G9);
  not NOT_12(G538,G10);
  not NOT_13(G540,G11);
  not NOT_14(G541,G12);
  not NOT_15(G543,G13);
  not NOT_16(G476,G30);
  not NOT_17(G484,G30);
  not NOT_18(G125,G40);
  not NOT_19(G140,G33);
  not NOT_20(G546,G41);
  not NOT_21(G132,G42);
  not NOT_22(G70,G43);
  not NOT_23(G67,G44);
  not NOT_24(G99,G29);
  not NOT_25(G475,G57);
  not NOT_26(G59,G58);
  not NOT_27(G228,G524);
  not NOT_28(G272,G271);
  not NOT_29(G98,G97);
  not NOT_30(G135,G134);
  not NOT_31(II218,G528);
  not NOT_32(G333,II218);
  not NOT_33(G55,G54);
  not NOT_34(G165,G529);
  not NOT_35(G72,G71);
  not NOT_36(G236,G274);
  not NOT_37(G275,G274);
  not NOT_38(II249,G538);
  not NOT_39(G370,II249);
  not NOT_40(G75,G74);
  not NOT_41(G490,G190);
  not NOT_42(G482,G241);
  not NOT_43(G153,G522);
  not NOT_44(G192,G193);
  not NOT_45(G123,G122);
  not NOT_46(II272,G209);
  not NOT_47(G458,II272);
  not NOT_48(II276,G238);
  not NOT_49(G332,II276);
  not NOT_50(II280,G272);
  not NOT_51(G309,II280);
  not NOT_52(II287,G135);
  not NOT_53(G347,II287);
  not NOT_54(G498,G195);
  not NOT_55(G78,G77);
  not NOT_56(II295,G198);
  not NOT_57(G459,II295);
  not NOT_58(G199,G200);
  not NOT_59(G90,G89);
  not NOT_60(G221,G222);
  not NOT_61(G223,G224);
  not NOT_62(II316,G239);
  not NOT_63(G369,II316);
  not NOT_64(G234,G235);
  not NOT_65(II327,G135);
  not NOT_66(G435,II327);
  not NOT_67(II330,G236);
  not NOT_68(G441,II330);
  not NOT_69(G50,G49);
  not NOT_70(G130,G9);
  not NOT_71(G501,G156);
  not NOT_72(G477,G276);
  not NOT_73(G485,G276);
  not NOT_74(II352,G77);
  not NOT_75(G299,II352);
  not NOT_76(G497,G205);
  not NOT_77(II371,G1);
  not NOT_78(G335,II371);
  not NOT_79(II374,G520);
  not NOT_80(G456,II374);
  not NOT_81(G87,G86);
  not NOT_82(II386,G199);
  not NOT_83(G414,II386);
  not NOT_84(G486,G68);
  not NOT_85(G231,G232);
  not NOT_86(G160,G161);
  not NOT_87(G265,G50);
  not NOT_88(G64,G63);
  not NOT_89(G180,G181);
  not NOT_90(G107,G456);
  not NOT_91(G207,G208);
  not NOT_92(G167,G168);
  not NOT_93(G124,G206);
  not NOT_94(G203,G204);
  not NOT_95(G489,G273);
  not NOT_96(G495,G273);
  not NOT_97(G177,G357);
  not NOT_98(G212,G213);
  not NOT_99(II493,G218);
  not NOT_100(G404,II493);
  not NOT_101(II502,G124);
  not NOT_102(G468,II502);
  not NOT_103(G173,G495);
  not NOT_104(G487,G534);
  not NOT_105(II529,G468);
  not NOT_106(G149,II529);
  not NOT_107(II536,G79);
  not NOT_108(G446,II536);
  not NOT_109(G494,G173);
  not NOT_110(G500,G173);
  not NOT_111(G214,G215);
  not NOT_112(G492,G62);
  not NOT_113(G483,G182);
  not NOT_114(G282,G281);
  not NOT_115(II573,G176);
  not NOT_116(G403,II573);
  not NOT_117(II576,G175);
  not NOT_118(G447,II576);
  not NOT_119(G479,G194);
  not NOT_120(G491,G194);
  not NOT_121(G554,G553);
  not NOT_122(G170,G171);
  not NOT_123(G172,G171);
  not NOT_124(G525,G526);
  not NOT_125(G493,G544);
  not NOT_126(G545,G544);
  not NOT_127(G488,G172);
  not NOT_128(G499,G280);
  not NOT_129(II624,G120);
  not NOT_130(G303,II624);
  not NOT_131(G480,G179);
  not NOT_132(II631,G188);
  not NOT_133(G336,II631);
  not NOT_134(G496,G188);
  not NOT_135(G174,G496);
  not NOT_136(II662,G174);
  not NOT_137(G405,II662);
  not NOT_138(G478,G279);
  not NOT_139(II692,G145);
  not NOT_140(G432,II692);
  and AND2_0(G359,G6,G31);
  and AND2_1(G469,G163,G3);
  and AND2_2(G461,G529,G531);
  and AND2_3(G431,G524,G67);
  and AND2_4(G362,G129,G77);
  and AND2_5(G81,G288,G240);
  and AND2_6(G348,G97,G55);
  and AND4_0(G352,G8,G135,G37,G164);
  and AND2_7(G511,G163,G164);
  and AND2_8(G379,G9,G211);
  and AND3_0(G385,G529,G7,G49);
  and AND2_9(G376,G533,G75);
  and AND3_1(G387,G6,G274,G75);
  and AND2_10(G462,G192,G538);
  and AND2_11(G436,G123,G77);
  and AND2_12(G363,G77,G205);
  and AND2_13(G410,G1,G205);
  and AND2_14(G399,G520,G1);
  and AND2_15(G437,G66,G229);
  and AND2_16(G307,G6,G104);
  and AND2_17(G306,G524,G78);
  and AND2_18(G283,G122,G219);
  and AND3_2(G339,G533,G199,G209);
  and AND3_3(G472,G136,G9,G190);
  and AND4_1(G351,G524,G169,G221,G234);
  and AND2_19(G440,G38,G234);
  and AND3_4(G382,G9,G100,G34);
  and AND2_20(G386,G536,G85);
  and AND2_21(G321,G90,G50);
  and AND2_22(G378,G89,G50);
  and AND3_5(G471,G191,G103,G112);
  and AND2_23(G377,G90,G56);
  and AND2_24(G358,G7,G83);
  and AND2_25(G400,G0,G277);
  and AND2_26(G308,G5,G151);
  and AND2_27(G411,G48,G59);
  and AND2_28(G413,G197,G201);
  and AND2_29(G434,G165,G231);
  and AND2_30(G373,G34,G160);
  and AND2_31(G357,G265,G232);
  and AND3_6(G444,G64,G78,G211);
  and AND2_32(G361,G6,G202);
  and AND2_33(G346,G2,G82);
  and AND2_34(G457,G4,G107);
  and AND2_35(G364,G2,G109);
  and AND2_36(G445,G53,G225);
  and AND2_37(G412,G3,G207);
  and AND3_7(G371,G161,G168,G267);
  and AND3_8(G353,G11,G92,G163);
  and AND2_38(G388,G11,G114);
  and AND2_39(G473,G11,G143);
  and AND2_40(G331,G213,G257);
  and AND2_41(G429,G51,G225);
  and AND2_42(G380,G6,G93);
  and AND2_43(G360,G8,G106);
  and AND2_44(G338,G202,G203);
  and AND2_45(G337,G270,G167);
  and AND2_46(G340,G8,G270);
  and AND3_9(G322,G522,G105,G196);
  and AND2_47(G330,G248,G249);
  and AND2_48(G430,G177,G196);
  and AND3_10(G344,G111,G189,G195);
  and AND2_49(G428,G212,G227);
  and AND2_50(G349,G6,G108);
  and AND3_11(G460,G2,G81,G115);
  and AND2_51(G463,G521,G148);
  and AND2_52(G393,G127,G34);
  and AND2_53(G470,G528,G149);
  and AND2_54(G341,G531,G118);
  and AND2_55(G342,G73,G197);
  and AND2_56(G324,G522,G183);
  and AND2_57(G323,G2,G144);
  and AND2_58(G354,G0,G214);
  and AND2_59(G312,G180,G182);
  and AND2_60(G315,G250,G251);
  and AND2_61(G474,G242,G77);
  and AND3_12(G343,G2,G528,G147);
  and AND2_62(G304,G52,G158);
  and AND3_13(G398,G94,G156,G158);
  and AND3_14(G365,G282,G137,G156);
  and AND3_15(G417,G13,G282,G70);
  and AND3_16(G290,G117,G135,G157);
  and AND3_17(G327,G4,G39,G157);
  and AND2_63(G367,G126,G157);
  and AND3_18(G397,G101,G98,G157);
  and AND3_19(G451,G541,G554,G187);
  and AND2_64(G406,G87,G172);
  and AND3_20(G418,G524,G60,G172);
  and AND2_65(G453,G545,G186);
  and AND3_21(G289,G2,G119,G156);
  and AND3_22(G311,G0,G178,G179);
  and AND2_66(G402,G154,G183);
  and AND2_67(G433,G91,G154);
  and AND2_68(G449,G88,G154);
  and AND2_69(G452,G526,G184);
  and AND2_70(G329,G150,G156);
  and AND2_71(G291,G138,G155);
  and AND3_23(G328,G5,G102,G155);
  and AND2_72(G366,G125,G155);
  and AND3_24(G372,G116,G275,G155);
  and AND2_73(G383,G131,G155);
  and AND2_74(G392,G132,G155);
  and AND3_25(G396,G76,G272,G155);
  and AND3_26(G401,G2,G110,G155);
  and AND3_27(G422,G0,G80,G155);
  and AND3_28(G415,G146,G142,G165);
  and AND2_75(G425,G146,G176);
  and AND3_29(G438,G8,G146,G133);
  and AND3_30(G424,G78,G174,G177);
  and AND2_76(G439,G174,G175);
  and AND2_77(G317,G159,G245);
  and AND3_31(G426,G37,G162,G38);
  and AND2_78(G443,G47,G162);
  and AND2_79(G416,G61,G167);
  and AND3_32(G427,G541,G95,G165);
  and AND2_80(G442,G541,G121);
  and AND2_81(G423,G541,G128);
  and AND2_82(G448,G139,G153);
  or OR2_0(G419,G3,G5);
  or OR2_1(G193,G6,G30);
  or OR2_2(G394,G5,G58);
  or OR2_3(G407,G6,G117);
  or OR2_4(G314,G527,G57);
  or OR2_5(G395,G4,G134);
  or OR2_6(G288,G1,G528);
  or OR2_7(G302,G4,G529);
  or OR2_8(G224,G533,G31);
  or OR2_9(G355,G11,G116);
  or OR2_10(G316,G531,G536);
  or OR2_11(G350,G6,G536);
  or OR2_12(G368,G533,G536);
  or OR2_13(G381,G7,G71);
  or OR2_14(G384,G529,G71);
  or OR2_15(G389,G9,G274);
  or OR2_16(G374,G536,G538);
  or OR2_17(G286,G9,G540);
  or OR2_18(G293,G7,G540);
  or OR2_19(G375,G10,G540);
  or OR2_20(G356,G6,G476);
  or OR2_21(G313,G521,G475);
  or OR2_22(G420,G522,G59);
  or OR3_0(G421,G521,G2,G228);
  or OR2_23(G320,G76,G272);
  or OR2_24(G310,G522,G135);
  or OR2_25(G408,G529,G77);
  or OR2_26(G305,G524,G55);
  or OR2_27(G409,G528,G55);
  or OR2_28(G296,G89,G484);
  or OR3_1(G325,G7,G536,G222);
  or OR2_29(G464,G72,G536);
  or OR2_30(G391,G74,G220);
  or OR2_31(G292,G538,G75);
  or OR2_32(G345,G529,G226);
  or OR2_33(G465,G524,G210);
  or OR2_34(G454,G122,G77);
  or OR2_35(G269,G362,G529);
  or OR2_36(G287,G522,G81);
  or OR3_2(G318,G6,G8,G232);
  or OR2_37(G326,G533,G232);
  or OR2_38(G390,G89,G50);
  or OR2_39(G298,G5,G497);
  or OR2_40(G300,G87,G97);
  or OR2_41(G261,G283,G528);
  or OR2_42(G301,G122,G486);
  or OR2_43(G92,G351,G352);
  or OR2_44(G47,G440,G441);
  or OR2_45(G114,G385,G386);
  or OR2_46(G297,G64,G274);
  or OR3_3(G93,G376,G377,G378);
  or OR2_47(G106,G358,G359);
  or OR2_48(G110,G399,G400);
  or OR2_49(G455,G78,G206);
  or OR3_4(G152,G306,G307,G308);
  or OR2_50(G60,G413,G414);
  or OR2_51(G133,G434,G435);
  or OR2_52(G105,G321,G273);
  or OR2_53(G108,G346,G347);
  or OR3_5(G115,G457,G458,G459);
  or OR2_54(G126,G363,G364);
  or OR2_55(G79,G444,G445);
  or OR2_56(G319,G529,G489);
  or OR2_57(G131,G379,G380);
  or OR2_58(G118,G337,G338);
  or OR2_59(G73,G339,G340);
  or OR2_60(G91,G430,G431);
  or OR2_61(G137,G348,G349);
  or OR2_62(G242,G469,G470);
  or OR2_63(G147,G341,G342);
  or OR3_6(G284,G528,G272,G281);
  or OR3_7(G294,G1,G117,G281);
  or OR3_8(G553,G322,G323,G324);
  or OR2_64(G141,G353,G354);
  or OR2_65(G142,G403,G404);
  or OR2_66(G88,G446,G447);
  or OR2_67(G544,G343,G344);
  or OR2_68(G285,G5,G479);
  or OR2_69(G295,G122,G491);
  or OR2_70(G450,G12,G171);
  or OR2_71(G150,G303,G304);
  or OR2_72(G146,G336,G170);
  or OR3_9(G539,G451,G452,G453);
  or OR2_73(G244,G371,G159);
  or OR4_0(G550,G289,G290,G291,G485);
  or OR3_10(G551,G327,G328,G329);
  or OR3_11(G552,G365,G366,G367);
  or OR2_74(G547,G382,G383);
  or OR2_75(G548,G392,G393);
  or OR4_1(G549,G396,G397,G398,G477);
  or OR2_76(G530,G401,G402);
  or OR2_77(G61,G405,G406);
  or OR2_78(G95,G424,G425);
  or OR2_79(G121,G438,G439);
  or OR2_80(G279,G317,G166);
  or OR4_2(G128,G415,G416,G417,G418);
  or OR2_81(G145,G426,G427);
  or OR2_82(G139,G442,G443);
  or OR2_83(G532,G422,G423);
  or OR2_84(G535,G432,G433);
  or OR2_85(G537,G448,G449);
  nand NAND2_0(G57,G0,G2);
  nand NAND2_1(G58,G1,G3);
  nand NAND2_2(G76,G0,G3);
  nand NAND2_3(G101,G3,G4);
  nand NAND2_4(G117,G2,G4);
  nand NAND2_5(G271,G1,G4);
  nand NAND2_6(G97,G2,G5);
  nand NAND2_7(G134,G3,G5);
  nand NAND2_8(G54,G4,G6);
  nand NAND2_9(G116,G6,G9);
  nand NAND2_10(G71,G8,G10);
  nand NAND2_11(G274,G7,G10);
  nand NAND2_12(G74,G9,G11);
  nand NAND2_13(G112,G8,G31);
  nand NAND2_14(G245,G8,G34);
  nand NAND2_15(G122,G522,G3);
  nand NAND2_16(G238,G2,G524);
  nand NAND2_17(G129,G527,G5);
  nand NAND2_18(G240,G4,G134);
  nand NAND4_0(G252,G3,G11,G35,G216);
  nand NAND2_19(G77,G4,G528);
  nand NAND3_0(G103,G529,G7,G30);
  nand NAND2_20(G200,G527,G529);
  nand NAND2_21(G248,G529,G36);
  nand NAND2_22(G89,G531,G8);
  nand NAND2_23(G222,G533,G10);
  nand NAND2_24(G239,G7,G533);
  nand NAND2_25(G235,G6,G536);
  nand NAND2_26(G220,G7,G71);
  nand NAND2_27(G49,G9,G538);
  nand NAND2_28(G251,G543,G32);
  nand NAND3_1(G276,G3,G543,G140);
  nand NAND2_29(G263,G0,G99);
  nand NAND2_30(G226,G527,G59);
  nand NAND2_31(G210,G520,G272);
  nand NAND2_32(G66,G129,G101);
  nand NAND2_33(G233,G522,G135);
  nand NAND3_2(G104,G122,G238,G240);
  nand NAND2_34(G86,G55,G3);
  nand NAND2_35(G219,G524,G55);
  nand NAND2_36(G68,G302,G528);
  nand NAND2_37(G232,G536,G164);
  nand NAND2_38(G136,G222,G224);
  nand NAND2_39(G510,G350,G235);
  nand NAND2_40(G161,G316,G72);
  nand NAND2_41(G100,G381,G220);
  nand NAND2_42(G85,G384,G239);
  nand NAND3_3(G243,G368,G275,G34);
  nand NAND2_43(G63,G75,G8);
  nand NAND3_4(G237,G10,G75,G201);
  nand NAND2_44(G503,G286,G538);
  nand NAND2_45(G56,G374,G375);
  nand NAND2_46(G83,G355,G356);
  nand NAND2_47(G96,G313,G314);
  nand NAND2_48(G278,G332,G333);
  nand NAND3_5(G255,G309,G2,G529);
  nand NAND3_6(G69,G419,G420,G233);
  nand NAND2_49(G512,G310,G233);
  nand NAND2_50(G181,G2,G78);
  nand NAND3_7(G277,G394,G395,G81);
  nand NAND2_51(G151,G305,G200);
  nand NAND3_8(G48,G407,G408,G409);
  nand NAND2_52(G264,G227,G241);
  nand NAND2_53(G208,G68,G229);
  nand NAND2_54(G168,G75,G221);
  nand NAND2_55(G84,G369,G370);
  nand NAND3_9(G258,G464,G103,G223);
  nand NAND2_56(G166,G7,G50);
  nand NAND2_57(G259,G130,G225);
  nand NAND2_58(G504,G292,G293);
  nand NAND2_59(G217,G50,G230);
  nand NAND2_60(G257,G538,G230);
  nand NAND3_10(G260,G528,G529,G191);
  nand NAND2_61(G266,G524,G96);
  nand NAND2_62(G262,G527,G278);
  nand NAND2_63(G138,G465,G263);
  nand NAND2_64(G256,G4,G69);
  nand NAND2_65(G82,G334,G335);
  nand NAND2_66(G109,G269,G219);
  nand NAND2_67(G206,G287,G524);
  nand NAND2_68(G204,G521,G87);
  nand NAND2_69(G53,G264,G237);
  nand NAND2_70(G273,G325,G326);
  nand NAND2_71(G267,G536,G84);
  nand NAND2_72(G113,G389,G390);
  nand NAND3_11(G143,G258,G193,G259);
  nand NAND2_73(G213,G64,G275);
  nand NAND2_74(G51,G260,G237);
  nand NAND3_12(G102,G320,G266,G210);
  nand NAND3_13(G52,G298,G299,G219);
  nand NAND3_14(G80,G421,G226,G256);
  nand NAND2_75(G270,G345,G204);
  nand NAND3_15(G94,G261,G181,G262);
  nand NAND3_16(G505,G300,G301,G181);
  nand NAND3_17(G249,G11,G273,G201);
  nand NAND2_76(G268,G11,G113);
  nand NAND2_77(G111,G213,G217);
  nand NAND3_18(G534,G296,G297,G166);
  nand NAND2_78(G253,G87,G218);
  nand NAND3_19(G148,G454,G455,G0);
  nand NAND2_79(G254,G1,G152);
  nand NAND2_80(G127,G391,G268);
  nand NAND3_20(G215,G135,G55,G212);
  nand NAND2_81(G62,G534,G32);
  nand NAND3_21(G523,G254,G255,G208);
  nand NAND2_82(G508,G318,G319);
  nand NAND3_22(G144,G215,G252,G253);
  nand NAND2_83(G250,G13,G523);
  nand NAND2_84(G281,G523,G534);
  nand NAND2_85(G171,G553,G187);
  nand NAND3_23(G526,G1,G2,G141);
  nand NAND2_86(G280,G46,G247);
  nand NAND2_87(G246,G544,G186);
  nand NAND2_88(G119,G284,G285);
  nand NAND2_89(G120,G294,G295);
  nand NAND2_90(G185,G525,G184);
  nand NAND2_91(G159,G6,G155);
  nand NAND3_24(G518,G450,G185,G246);
  nand NAND3_25(G542,G243,G244,G279);
  nor NOR2_0(G163,G0,G4);
  nor NOR2_1(G216,G4,G5);
  nor NOR2_2(G169,G5,G7);
  nor NOR2_3(G225,G7,G8);
  nor NOR2_4(G190,G7,G11);
  nor NOR2_5(G241,G10,G11);
  nor NOR2_6(G198,G520,G3);
  nor NOR2_7(G178,G521,G4);
  nor NOR2_8(G229,G1,G522);
  nor NOR2_9(G209,G1,G524);
  nor NOR2_10(G195,G521,G134);
  nor NOR2_11(G189,G522,G54);
  nor NOR2_12(G201,G528,G54);
  nor NOR2_13(G164,G531,G10);
  nor NOR2_14(G211,G6,G274);
  nor NOR2_15(G156,G12,G543);
  nor NOR2_16(G205,G529,G122);
  nor NOR2_17(G227,G5,G200);
  nor NOR2_18(G230,G8,G490);
  nor NOR2_19(G191,G9,G482);
  nor NOR3_0(G196,G5,G540,G86);
  nor NOR2_20(G197,G540,G232);
  nor NOR2_21(G202,G10,G63);
  nor NOR2_22(G502,G436,G437);
  nor NOR2_23(G218,G528,G217);
  nor NOR3_1(G516,G410,G411,G412);
  nor NOR2_24(G515,G387,G388);
  nor NOR2_25(G509,G331,G5);
  nor NOR2_26(G513,G360,G361);
  nor NOR2_27(G183,G330,G3);
  nor NOR2_28(G517,G428,G429);
  nor NOR2_29(G182,G12,G62);
  nor NOR4_0(G519,G460,G461,G462,G463);
  nor NOR2_30(G176,G4,G494);
  nor NOR2_31(G175,G86,G500);
  nor NOR2_32(G187,G13,G492);
  nor NOR2_33(G158,G521,G281);
  nor NOR2_34(G194,G281,G271);
  nor NOR2_35(G157,G13,G483);
  nor NOR3_2(G507,G315,G12,G487);
  nor NOR2_36(G186,G282,G501);
  nor NOR4_1(G247,G471,G472,G473,G474);
  nor NOR2_37(G179,G541,G280);
  nor NOR2_38(G188,G543,G493);
  nor NOR2_39(G154,G12,G488);
  nor NOR3_3(G184,G541,G13,G499);
  nor NOR2_40(G506,G311,G312);
  nor NOR2_41(G155,G13,G480);
  nor NOR2_42(G162,G185,G498);
  nor NOR3_4(G514,G372,G373,G478);

endmodule
module dff(input clk, input d, output reg q);
always@ (posedge clk)
q <= d;
endmodule
