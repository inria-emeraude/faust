ID_0 = float(fSamplingFreq);
ID_1 = max(1.0f, ID_0);
ID_2 = min(192000.0f, ID_1);
ID_3 = 6.2831853071795862f/ID_2;
ID_4 = hslider("freq[unit:Hz]",700.0f,1.0f,20000.0f,1.0f);
ID_5 = ID_3*ID_4;
ID_6 = (ID_5+1.0f);
ID_7 = 1.0f/ID_6;
ID_8 = W0;
ID_9 = proj0(ID_8);
ID_10 = ID_9';
ID_11 = W1;
ID_12 = proj0(ID_11);
ID_13 = ID_12';
ID_14 = W2;
ID_15 = proj0(ID_14);
ID_16 = ID_15';
ID_17 = W3;
ID_18 = proj0(ID_17);
ID_19 = ID_18';
ID_20 = hslider("level",0.5f,0.0f,1.0f,0.01f);
ID_21 = ID_20*ID_4;
ID_22 = ID_3*ID_21;
ID_23 = 0.25f*ID_2;
ID_24 = 1';
ID_25 = float(ID_24);
ID_26 = hslider("freq carre",440.0f,20.0f,8000.0f,1.0f);
ID_27 = max(ID_26, 23.448949682462139f);
ID_28 = abs(ID_27);
ID_29 = max(20.0f, ID_28);
ID_30 = 1.0f/ID_29;
ID_31 = 1.0f/ID_2;
ID_32 = ID_31*ID_29;
ID_33 = W4;
ID_34 = proj0(ID_33);
ID_35 = ID_34';
ID_36 = ID_32+ID_35;
ID_37 = floor(ID_36);
ID_38 = ID_35-ID_37;
ID_39 = ID_32+ID_38;
ID_40 = letrec(W4 = (ID_39));
ID_41 = proj0(ID_40);
ID_42 = (ID_41@0);
ID_43 = 2.0f*ID_42;
ID_44 = ID_43+-1.0f;
ID_45 = pow(ID_44, 2.0f);
ID_46 = ID_45';
ID_47 = (ID_45-ID_46);
ID_48 = ID_30*ID_47;
ID_49 = hslider("delta",2.0f,0.0f,6.0f,0.10000000000000001f);
ID_50 = ID_26+ID_49;
ID_51 = max(ID_50, 23.448949682462139f);
ID_52 = abs(ID_51);
ID_53 = max(20.0f, ID_52);
ID_54 = 1.0f/ID_53;
ID_55 = ID_31*ID_53;
ID_56 = W5;
ID_57 = proj0(ID_56);
ID_58 = ID_57';
ID_59 = ID_55+ID_58;
ID_60 = floor(ID_59);
ID_61 = ID_58-ID_60;
ID_62 = ID_55+ID_61;
ID_63 = letrec(W5 = (ID_62));
ID_64 = proj0(ID_63);
ID_65 = (ID_64@0);
ID_66 = 2.0f*ID_65;
ID_67 = ID_66+-1.0f;
ID_68 = pow(ID_67, 2.0f);
ID_69 = ID_68';
ID_70 = (ID_68-ID_69);
ID_71 = ID_54*ID_70;
ID_72 = (ID_48+ID_71);
ID_73 = ID_25*ID_72;
ID_74 = ID_23*ID_73;
ID_75 = (0.5f*ID_2);
ID_76 = ID_75/ID_51;
ID_77 = min(2047.0f, ID_76);
ID_78 = max(0.0f, ID_77);
ID_79 = int(ID_78);
ID_80 = float(ID_79);
ID_81 = (ID_78-ID_80);
ID_82 = ID_23/ID_53;
ID_83 = ID_25*ID_70;
ID_84 = ID_82*ID_83;
ID_85 = (ID_79+1);
ID_86 = (ID_84@ID_85);
ID_87 = ID_81*ID_86;
ID_88 = ID_75/ID_27;
ID_89 = min(2047.0f, ID_88);
ID_90 = max(0.0f, ID_89);
ID_91 = int(ID_90);
ID_92 = float(ID_91);
ID_93 = 1.0f-ID_90;
ID_94 = (ID_92+ID_93);
ID_95 = ID_23/ID_29;
ID_96 = ID_25*ID_47;
ID_97 = ID_95*ID_96;
ID_98 = (ID_97@ID_91);
ID_99 = ID_94*ID_98;
ID_100 = (ID_90-ID_92);
ID_101 = (ID_91+1);
ID_102 = (ID_97@ID_101);
ID_103 = ID_100*ID_102;
ID_104 = ID_99+ID_103;
ID_105 = 1.0f-ID_78;
ID_106 = (ID_80+ID_105);
ID_107 = (ID_84@ID_79);
ID_108 = ID_106*ID_107;
ID_109 = ID_104+ID_108;
ID_110 = (ID_87+ID_109);
ID_111 = (ID_74-ID_110);
ID_112 = ID_22*ID_111;
ID_113 = (ID_19+ID_112);
ID_114 = ID_7*ID_113;
ID_115 = letrec(W3 = (ID_114));
ID_116 = proj0(ID_115);
ID_117 = (ID_116@0);
ID_118 = ID_5*ID_117;
ID_119 = (ID_16+ID_118);
ID_120 = ID_7*ID_119;
ID_121 = letrec(W2 = (ID_120));
ID_122 = proj0(ID_121);
ID_123 = (ID_122@0);
ID_124 = ID_5*ID_123;
ID_125 = (ID_13+ID_124);
ID_126 = ID_7*ID_125;
ID_127 = letrec(W1 = (ID_126));
ID_128 = proj0(ID_127);
ID_129 = (ID_128@0);
ID_130 = ID_5*ID_129;
ID_131 = (ID_10+ID_130);
ID_132 = ID_7*ID_131;
ID_133 = letrec(W0 = (ID_132));
ID_134 = proj0(ID_133);
ID_135 = ID_134@0;
ID_136 = checkbox("NL");
ID_137 = 0.33333333333333331f*ID_136;
ID_138 = (0.0f-ID_137);
ID_139 = W6;
ID_140 = proj0(ID_139);
ID_141 = ID_140';
ID_142 = W7;
ID_143 = proj0(ID_142);
ID_144 = ID_143';
ID_145 = W8;
ID_146 = proj0(ID_145);
ID_147 = ID_146';
ID_148 = W9;
ID_149 = proj0(ID_148);
ID_150 = ID_149';
ID_151 = ID_20*ID_111;
ID_152 = pow(ID_151, 3.0f);
ID_153 = pow(ID_117, 3.0f);
ID_154 = (ID_152-ID_153);
ID_155 = ID_5*ID_154;
ID_156 = (ID_150+ID_155);
ID_157 = ID_7*ID_156;
ID_158 = letrec(W9 = (ID_157));
ID_159 = proj0(ID_158);
ID_160 = ID_159@0;
ID_161 = (ID_160+ID_153);
ID_162 = pow(ID_123, 3.0f);
ID_163 = (ID_161-ID_162);
ID_164 = ID_5*ID_163;
ID_165 = (ID_147+ID_164);
ID_166 = ID_7*ID_165;
ID_167 = letrec(W8 = (ID_166));
ID_168 = proj0(ID_167);
ID_169 = ID_168@0;
ID_170 = (ID_169+ID_162);
ID_171 = pow(ID_129, 3.0f);
ID_172 = (ID_170-ID_171);
ID_173 = ID_5*ID_172;
ID_174 = (ID_144+ID_173);
ID_175 = ID_7*ID_174;
ID_176 = letrec(W7 = (ID_175));
ID_177 = proj0(ID_176);
ID_178 = ID_177@0;
ID_179 = (ID_178+ID_171);
ID_180 = pow(ID_135, 3.0f);
ID_181 = (ID_179-ID_180);
ID_182 = ID_5*ID_181;
ID_183 = (ID_141+ID_182);
ID_184 = ID_7*ID_183;
ID_185 = letrec(W6 = (ID_184));
ID_186 = proj0(ID_185);
ID_187 = (ID_186@0);
ID_188 = ID_138*ID_187;
ID_189 = ID_135+ID_188;
SIG = (ID_189);