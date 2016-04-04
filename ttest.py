from collections import deque
import numpy as np

def time_val(value):	
	return ((ord(value[0])-48)<<18)|((ord(value[1])-48)<<12)|((ord(value[2])-48)<<6)|(ord(value[3])-48)

def dist_val(value):	
	try:	
		return ((ord(value[0])-48)<<12)|((ord(value[1])-48)<<6)|(ord(value[2])-48)
	except IndexError:
		print 'error ', value	
		return 0	

def listc(data):
	dist2 = [item[:-1] for item in data]
			
def forl(data):
	for idx, item in enumerate(data):
		data[idx] = item[:-1]

def collec(d):
	for i in xrange(1000):
		d.append(i)
	#print d

def lists(l):
	for i in xrange(1000):
		l.append(i)
	#print l

def array(n):
	for i in xrange(1000):	
		np.append(n, i)
	#print n

def concat(data):
	dist4 = ''.join(data)

def plus(data):
	item = ''	
	for i in data:
		item += i

#@profile
def update_di(answer):		
	#answer = answer.split('\n')		
	#time = answer.pop(1)[:-1]
	dist = answer[2:-2]
	dist2 = [item[:-1] for item in dist]
	dist4 = ''.join(dist2)			
	step, idxh = 0, 3 			
	polar_graph = deque()
	angle = deque()	
	lend = len(dist4)
	while idxh <= lend:
		point = dist_val(dist4[idxh-3:idxh])
		#strength = dist_val(dist4[idxh:idxh+3])
		if dist_val(dist4[idxh:idxh+3]) > 1600:	
			polar_graph.append(point)
			angle.append(step)
		#idxl = idxh + 3
		idxh += 6		
		step += 0.004363323129985824
	#print 'point ', polar_graph
	#print 'angle ', angle
	combined = zip(angle, polar_graph)
	print 'zip ', combined	
	print len(combined)	
#print type(combined)
#d = deque()
#n = np.empty([1, 1])
#l = []

data = ['0VM06=0VI06A0VW05o0VW05o0VW05o0VW06]0VD06`0V@06b0VA06n0V:07<0UF0f', 
 '740RS07m0MM08`0LL08f0LB08_0L?0920L?0920L=0950L10940Ko0980Ke0950KB', 
 '_09:0KX0980KL08e0KS09>0KG08d0K908a0Jm08`0Jl08j0Je08b0J^08Z0JJ08P`', 
 '0JO08b0JE08\\0JA08_0J808Z0J208^0I^08T0IY08U0IL08W0I308K0I808b0Hi0?', 
 '8h0HX08i0HK0900H708g0Ge08m0GT08m0GH08l0G80930G409W0Fc09N0F_09M0F6', 
 'S07h0FN06C?om000?om000?om000?om000?om000?om000?om000?om000?om000B',
 '?om000?om000?om000?om000?om000?om000?om000?om0000`00720]D0:b0]R0M',
 'C@0]Q0B80]N0@J0\\M07V0\\<06^0]206^0^706d?om000?om000?om000?om000?o^',
 'm000?om000?om000?om000?om000?om000?om0000hC08>?om000?om000?om0001',
 '?om000?om000?om000?om00004405f03Y06803U06N03[06b03Q07403Y07Z03V0@', 
 '7l03M08S0470<504;0@60490DU0480FM04:0H>0480HC0450I[0480Ig04>0J004H', 
 '90J>0440JG0410JU0410JW0410J^03h0J_03d0J^03d0Jb03d0Je03h0Jf03h0JfS', 
 '03e0Jk03g0Jk03j0Jj03j0Jl03j0Jk03k0Jk03k0J`03k0J`0400Jn03k0Jh03l0P', 
 'K303k0K003e0K203e0Jn03g0Ja03n0J^03j0J^03j0J^03j0JZ03k0JV0410JQ04g',
 '10JQ03k0J603g0I\\0430IX04@0IO0470HI0480H<04;0Ga04F0Fc04>0CD04F0>YM', 
 '04K0<U05K0<K05f0<N0650;106G0:]06L0:106[09G06o09G07N09A07n09=08?0k',
 '9:08e09A0:808a0;F0:N0<L0:@0>U0=C0?X0=[0H:0;J0HA0;U0HD0;e0H?0;?0HW', 
 '@0;k0G_09`0GT0:80GR0:Z0GN0;80GR0;L0GR0;L0GK0;A0GO0;Z0GO0;Z0GN0;SN',
 '0GN0;S0GO0;[0GN0;U0GF0;A0GI0;70GR0:e0GR0:i0GF09n0GJ09:0GT08h0H806', 
 '8S0I108F0I>08@0I408C0Hn08T0I<08i0IJ09>0IJ09>0IC0910IH0920IZ08m0JZ', 
 '008h0J908B0JV0800KO08A0Ka08E0Ka08E0LP09Q0Lo09\\0Nd09J0Q[09g0WP08;k', 
 '0W;07V0Uj0750U<0740T=07K0Sn07M0Sg07J0S_07O0T?07=0U206k0Ti06[0UM0@',
 '6l0Un07B0V007F0Uc07J0Uc07^0U\\07M0UR07A0Ua07\\0UY07W0UZ08E0U>07m0TW', 
 'o07f0Tm08:0Tj0840Tl08L0Tl08V0TQ0860TG07i0TP08D0TA0850T@08I0TG08`S',
 '0Sk07l0QS0880O008_0M\\0860LX07_0LE07_0KJ07l0I>08N0H309G0Gn0:80Gj0k',
 ':Z0Ga0:_0G`0:`0G[0:_0GV0:n0GN0:e0GM0:l0GN0:f0GJ0:e0G>0:Z0G?0:[0GD',
 '30:S0G50:U0G00:[0G10;40Fo0;E0Fd0;=0Fc0;C0Fe0;E0Ff0:m0F^0;20FQ0;1n', 
 '0FU0;F0FN0;D0FN0;U0FK0;C0FW0:Q0FP08l0F^08a0Fl08S0G508W0GI08Y0G^0_',
 '8[0Ha09n0Ha09n0HG09M0HM08a0HQ08W0HV08Q0I108[0I`09D0Jm09h0Mg09V0PC', 
 '90920P508M0P>08W0PD08a0P>08X0P008B0Of0890Oc07k0OX07o0OX08?0Og08@C',
 '0PN08;0Q?08L0Rm08o0U[0910Wo08U0X408U0X508Z0Wj08Y0Wb08S0Wa08T0Wi0Z',
 '8W0Wc08\\0WV08Y0W?08a0Jm0;C0GM0;K0FW09b0FF09:0E_07l0EQ07c0EH07[0EK',
 'H07[0ED07W0EG07a0EG07a0E=07T0EI07k0E>07`0EC07j0E707d0E107j0E107i]',
 '0E107h0E107[0E607j0E307n0E10800Di07j0Dl0840Do08:0Dj08;0DX08Y0DP05',
 '9@0D]0:P0D[0:e0DU0;00DU0;00DW0;00DW0;00D[0;B0DY0;L0DR0;X0DR0;X0Dl',
 'W0;e0DZ0;f0D\\0<M0DZ0<R0DZ0<a0DY0<j0D[0=>0D_0=g0Da0>B0DV0>[0DW0>hc',
 '0D[0?M0D^0?a0D^0?e0D^0>h0D^0>h0D]0>[0D]0=T0D\\0=B0D\\0=B0D_0<f0D^0d',
 '<\\0DY0<<0DV0<00DT0;j0DT0;j0D\\0;]0D\\0;L0D\\0;L0D[0;G0DY0;;0DZ0;G0Dk',
 'Z0;80D\\0;60D]0;10D^0:h0Dc0:i0D`0:_0Dd0:_0De0:_0Di0:^0Dd0:Y0De0:RN', 
 '0Dn0:D0E30:E0E20:D0Di09m0Dh0:40Dg09d0Dg09Y0Dl09a0Dj09M0Dm09F0Do0?',
 '9?0E209>0E60920EB08Z0EE0850EK07n0ES07i0EN07d0EJ07^0EY07d0E]07d0E6', 
 'Z07c0EY07_0E_07f0Ej07e0Eo07d0F207c0F707h0F707_0F>07b0FM0870FF07dj',
 '0FW0820F\\0850GU09o0H=0;20Hb0;>0Jg0;j0TS0:_0U@0:90Um09f0Wc09N0Z]06',
 '8i0Zm08Y0[408W0[808X0[808X0[<08T0[=08M0[C08U0[O08M0[J08M0[U08\\0[`',
 'V08Y0[X08Q0[d08]0[f08R0[f08R0[m08W0[m08P0\\208K0\\C08Z0\\I08Y0\\S08eX',
 '0\\N08X0\\\\08a0\\a08\\0\\g08V0\\l08`0\\c08[0\\H08a0Uj0:50T>09d0SI08m0Rm0I', 
 '8Q0R107W0Q]07Y0QN07a0QL07b0Pl07Z0Pb07j0Pb07o0P\\07>0QD06S?om000?oe',
 'm000?om000?om000?om0000SB06W0TB06c0TF06U0TI06G?om000?om000?om000?', 
 '0TL05o0T105k0TY07>0S107b0PK08h0OM0970Nl09=0N[09;0NX09<0NN09N0NP0h', 
 '9^0NA09=0NA0950ND0900NT08e0O=08m0P408e0R808J0Rm08>0S20880Rb07n0Rf',
 'U07l0RQ0860RH07j0RK07`0RT07\\0RU07^0R[07o0R_0850SH08Q0S]08L0UE088`',
 '0WD07]0Wg0830Wo07f0X?0830XK07[0XR07:0XW0750Xb07=?om000?om000?om0g',
 '00?om000?om00007P05d07H06P07M07208a0=G08L0=Z07n0>M07\\0CS07_0HM07[', 
 'X0JN07Q0Ki07N0LJ07M0LM07@0LU07>0La07;0M807>0MN07>0MT07>0MT07>0MYL', 
 '07F0Mi07A0M]07A0M]07>0MW07E0ML07E0ML07H0M<07?0Lf07Q0L907E0Ko07P0D', 
 'L107P0K<07V0KI07W0IJ07X0FA07a0BC0850>G08Q0>P07:07A07306L07O05j?oa',
 'm000?om000?om000?om000?om000?om000?om0000[N08j0[=09B0Ze09N0ZP09D:',
 '0ZN09C0ZO09?0ZQ08R0ZZ0860Z`07Q0Z;06V0Y`06:0Y^06;0Z406n0Z\\07b0^H0G', 
 '7`0`]0750`]0750MK0;B0IZ0<C0Hm0;T0H609V0H309S0Gg09i0G`0:70GW0:;0GZ', 
 ']0:<0G]0:30GR09k0GG0:00GF0:20GA0:60G@0:>0GD0:D0GQ0:G0GS0:C0GE0:9a',
 '0GE0:90GC0:=0G?0:D0GA0:B0GA0:C0GC0:C0GC0:C0G50:60G@0:10GF0:20GF0O',
 ':20GI0:20GE09n0GD09j0GE09n0GE09n0GA09[0GA09[0GH09R0GD0940GN0940Gb',
 '\\0960H<0:K0H<0:K0H=0:W0H80:?0H809`0HC09g0HC09_0H<0900HD0970HQ08lY', 
 '0H_08d0IU0970Ii09F0J609H0JU09X0L=09N?om0000[Q0820Y708^0U609_0Q10?',
 ':60O_08K0OA07H?om000?om000?om0000Q>06K0Q>06K0R>06c0T^06=?om0000V@', 
 'i08C0VY07M0XL07Y0Z00640Yi0660Yc0680Yk06A0Yo06@0Y[0670YU05n0Y[060<',
 '0Yb0620Yb0620YZ05j0YZ05j?om000?om000?om000?om000?om000?om000?om0g',
 '000`T06R0`=06V0`J06]?om000?om000?om0000c706F0cH06D0dA06;0d;05k?oQ',
 'm000?om000?om000?om000?om000?om0000bM06n0cB0;h0cM0EY0bX0<L0bI0:ma', 
 '0b60910bL08n0c;0870e207R0gj06]0ho06C0iJ05d?om000?om000?om000?om0_',
 '00?om000?om000?om000?om000?om000?om00007k06T07U06f07U06f07T06I?oF',
 'm000?om000?om0000:]09P0;?0=40;F0=_0;<0=h0;70=m0;30>00;80><0;60>E`', 
 '0:e0>70:c0>60:`0>20:H0>10:60=h09c0=M09]0=C09l0=V09l0=V09f0=T09d0>', 
 '>709V0>P09A0>I08o0=V08d0=b08H0;108>0;70860:\\08<0;d0870<207n0<X07`', 
 'f0<M07b0<L07P0<_07:0<>06Q0;Z05U0;m0550>Y05:0A305L0GG05I0HI05B0IGT', 
 '0580Id0510JH0550Ja0550JX0520Jj0500K404m0K804j0K=04j0KC04j0KB04k0B', 
 'KT04n0KT04k0KU04l0K]04l0K^04l0Ka04l0Kb04l0K^0520K_0510KX0510KR05D', 
 '10KR0510KT0510KP0510KK04k0KD04j0K804k0K904o0Jk0550J`0550J80530J0\\',
 '0510HN05@0HQ05@0F=05?0@g05B0>O04m09]04m08V05507l05O07i05c08706N0S',
 '8T07E08a07L09107S08j07b08^07i08c07o08[08A08]08C08Y08G08Y08N08V083',
 'O08Y08J08J08O08G08T08L08V08I08]08K08\\08D08[08A08]08?08_08608^0895',
 '08\\08708\\08708h08=08f08908h08;08`07k08^07m08`08208g08808i08008k0>', 
 '7n09107m09107j09408009807o09607k09807c09807c09@08209808009:08409E', 
 '@08P09C08^09C09B09U0:l09Z0<_09Z0<_09G09J09E08@09T071?om000?om000E',
 '?om000?om000?om000?om0000:S06S0:507I0:807j0:008_09l08a0:108l09b0=',
 '8[09V08X09P08R09S08Z09Y08c09S08^09R08o09K08d09D08U09J08O09@08A09c', 
 'A08B09C08M09D08W09@08n09809309308l09=09009809:08l08c08e08Z094096A', 
 '09208d08o08W08o08X08o08T08m08O08k08J08j08N08c08R08c08R08c08\\08h0I', 
 '8j08m09808i08m08e08a08d08Z08^08W08X08N08P08F08B08;08D08>08E08>08M',
 'J08W08N08[08J08Y08B08Q08:08I08<08J08808D08908C08;08G08>08M08;08GH', 
 '08508908808D07n08307m07j07h07a07g07l07a07`07h07l07j08607i08207b0J', 
 '7d07b07d07b08207d08107_07o07V07^07W07f07X07h07U07o07V08207W07n07@', 
 'd07m07d07f07U07c07V07m07V07m07R07j07P07m07N07i07L07o07O08?07M08AE', 
 '07P08M07N08P07V08m07b08b07a08X07J08107J08207B07X07F08I07909107?0a',
 '9`07C09M07?07m07<07Y07Q08@07M08807D07o07C08707H08O07H08O07E08B07M',
 'F08?07A07\\07F07=07J06c07>06507;05k07H06`06f07506k07=07V06k07T06NT',
 '07d06Z?om000?om000?om000?om00009n05l0:>06U09j08H0:B0@>0:<0An0:?0m', 
 'Cl0:80>I09k09M09d08;09i07j09d07U09f07J09h06X?om0000eT06a0eU06b0eO',
 'H06b0;Q05k0<\\07Q0?:0>I0>n0=o0=m0:l0=N0:>0=@09K0=J09o0=@09K0<M087`',
 '0;S0720;G0780:o06O?om000?om000?om000?om000?om000?om000?om000?om0^',
 '00?om000?om000?om000?om000?om000?om000?om000?om000?om000?om000?ol',
 'm000?om000?om000?om000n']

test_data = ['GE0000108000\r00P', '9BI\\P',
 '0VM06=0VI06A0VW05o0VW05o0VW05o0VW06]0VD06`0V@06b0VA06n0V:07<0UF0f',
 '740RS07m0MM08`0LL08f0LB08_0L?0920L?0920L=0950L10940Ko0980Ke0950KB', 
 '_09:0KX0980KL08e0KS09>0KG08d0K908a0Jm08`0Jl08j0Je08b0J^08Z0JJ08P`',
 '0JO08b0JE08\\0JA08_0J808Z0J208^0I^08T0IY08U0IL08W0I308K0I808b0Hi0?', 
 '8h0HX08i0HK0900H708g0Ge08m0GT08m0GH08l0G80930G409W0Fc09N0F_09M0F6',
 'S07h0FN06C?om000?om000?om000?om000?om000?om000?om000?om000?om000B',
 '?om000?om000?om000?om000?om000?om000?om000?om0000`00720]D0:b0]R0M',
 'C@0]Q0B80]N0@J0\\M07V0\\<06^0]206^0^706d?om000?om000?om000?om000?o^',
 'm000?om000?om000?om000?om000?om000?om0000hC08>?om000?om000?om0001',
 '?om000?om000?om000?om00004405f03Y06803U06N03[06b03Q07403Y07Z03V0@', 
 '7l03M08S0470<504;0@60490DU0480FM04:0H>0480HC0450I[0480Ig04>0J004H', 
 '90J>0440JG0410JU0410JW0410J^03h0J_03d0J^03d0Jb03d0Je03h0Jf03h0JfS', 
 '03e0Jk03g0Jk03j0Jj03j0Jl03j0Jk03k0Jk03k0J`03k0J`0400Jn03k0Jh03l0P', 
 'K303k0K003e0K203e0Jn03g0Ja03n0J^03j0J^03j0J^03j0JZ03k0JV0410JQ04g',
 '10JQ03k0J603g0I\\0430IX04@0IO0470HI0480H<04;0Ga04F0Fc04>0CD04F0>YM', 
 '04K0<U05K0<K05f0<N0650;106G0:]06L0:106[09G06o09G07N09A07n09=08?0k',
 '9:08e09A0:808a0;F0:N0<L0:@0>U0=C0?X0=[0H:0;J0HA0;U0HD0;e0H?0;?0HW', 
 '@0;k0G_09`0GT0:80GR0:Z0GN0;80GR0;L0GR0;L0GK0;A0GO0;Z0GO0;Z0GN0;SN',
 '0GN0;S0GO0;[0GN0;U0GF0;A0GI0;70GR0:e0GR0:i0GF09n0GJ09:0GT08h0H806', 
 '8S0I108F0I>08@0I408C0Hn08T0I<08i0IJ09>0IJ09>0IC0910IH0920IZ08m0JZ', 
 '008h0J908B0JV0800KO08A0Ka08E0Ka08E0LP09Q0Lo09\\0Nd09J0Q[09g0WP08;k', 
 '0W;07V0Uj0750U<0740T=07K0Sn07M0Sg07J0S_07O0T?07=0U206k0Ti06[0UM0@',
 '6l0Un07B0V007F0Uc07J0Uc07^0U\\07M0UR07A0Ua07\\0UY07W0UZ08E0U>07m0TW', 
 'o07f0Tm08:0Tj0840Tl08L0Tl08V0TQ0860TG07i0TP08D0TA0850T@08I0TG08`S',
 '0Sk07l0QS0880O008_0M\\0860LX07_0LE07_0KJ07l0I>08N0H309G0Gn0:80Gj0k',
 ':Z0Ga0:_0G`0:`0G[0:_0GV0:n0GN0:e0GM0:l0GN0:f0GJ0:e0G>0:Z0G?0:[0GD',
 '30:S0G50:U0G00:[0G10;40Fo0;E0Fd0;=0Fc0;C0Fe0;E0Ff0:m0F^0;20FQ0;1n', 
 '0FU0;F0FN0;D0FN0;U0FK0;C0FW0:Q0FP08l0F^08a0Fl08S0G508W0GI08Y0G^0_',
 '8[0Ha09n0Ha09n0HG09M0HM08a0HQ08W0HV08Q0I108[0I`09D0Jm09h0Mg09V0PC', 
 '90920P508M0P>08W0PD08a0P>08X0P008B0Of0890Oc07k0OX07o0OX08?0Og08@C',
 '0PN08;0Q?08L0Rm08o0U[0910Wo08U0X408U0X508Z0Wj08Y0Wb08S0Wa08T0Wi0Z',
 '8W0Wc08\\0WV08Y0W?08a0Jm0;C0GM0;K0FW09b0FF09:0E_07l0EQ07c0EH07[0EK',
 'H07[0ED07W0EG07a0EG07a0E=07T0EI07k0E>07`0EC07j0E707d0E107j0E107i]',
 '0E107h0E107[0E607j0E307n0E10800Di07j0Dl0840Do08:0Dj08;0DX08Y0DP05',
 '9@0D]0:P0D[0:e0DU0;00DU0;00DW0;00DW0;00D[0;B0DY0;L0DR0;X0DR0;X0Dl',
 'W0;e0DZ0;f0D\\0<M0DZ0<R0DZ0<a0DY0<j0D[0=>0D_0=g0Da0>B0DV0>[0DW0>hc',
 '0D[0?M0D^0?a0D^0?e0D^0>h0D^0>h0D]0>[0D]0=T0D\\0=B0D\\0=B0D_0<f0D^0d',
 '<\\0DY0<<0DV0<00DT0;j0DT0;j0D\\0;]0D\\0;L0D\\0;L0D[0;G0DY0;;0DZ0;G0Dk',
 'Z0;80D\\0;60D]0;10D^0:h0Dc0:i0D`0:_0Dd0:_0De0:_0Di0:^0Dd0:Y0De0:RN', 
 '0Dn0:D0E30:E0E20:D0Di09m0Dh0:40Dg09d0Dg09Y0Dl09a0Dj09M0Dm09F0Do0?',
 '9?0E209>0E60920EB08Z0EE0850EK07n0ES07i0EN07d0EJ07^0EY07d0E]07d0E6', 
 'Z07c0EY07_0E_07f0Ej07e0Eo07d0F207c0F707h0F707_0F>07b0FM0870FF07dj',
 '0FW0820F\\0850GU09o0H=0;20Hb0;>0Jg0;j0TS0:_0U@0:90Um09f0Wc09N0Z]06',
 '8i0Zm08Y0[408W0[808X0[808X0[<08T0[=08M0[C08U0[O08M0[J08M0[U08\\0[`',
 'V08Y0[X08Q0[d08]0[f08R0[f08R0[m08W0[m08P0\\208K0\\C08Z0\\I08Y0\\S08eX',
 '0\\N08X0\\\\08a0\\a08\\0\\g08V0\\l08`0\\c08[0\\H08a0Uj0:50T>09d0SI08m0Rm0I', 
 '8Q0R107W0Q]07Y0QN07a0QL07b0Pl07Z0Pb07j0Pb07o0P\\07>0QD06S?om000?oe',
 'm000?om000?om000?om0000SB06W0TB06c0TF06U0TI06G?om000?om000?om000?', 
 '0TL05o0T105k0TY07>0S107b0PK08h0OM0970Nl09=0N[09;0NX09<0NN09N0NP0h', 
 '9^0NA09=0NA0950ND0900NT08e0O=08m0P408e0R808J0Rm08>0S20880Rb07n0Rf',
 'U07l0RQ0860RH07j0RK07`0RT07\\0RU07^0R[07o0R_0850SH08Q0S]08L0UE088`',
 '0WD07]0Wg0830Wo07f0X?0830XK07[0XR07:0XW0750Xb07=?om000?om000?om0g',
 '00?om000?om00007P05d07H06P07M07208a0=G08L0=Z07n0>M07\\0CS07_0HM07[', 
 'X0JN07Q0Ki07N0LJ07M0LM07@0LU07>0La07;0M807>0MN07>0MT07>0MT07>0MYL', 
 '07F0Mi07A0M]07A0M]07>0MW07E0ML07E0ML07H0M<07?0Lf07Q0L907E0Ko07P0D', 
 'L107P0K<07V0KI07W0IJ07X0FA07a0BC0850>G08Q0>P07:07A07306L07O05j?oa',
 'm000?om000?om000?om000?om000?om000?om0000[N08j0[=09B0Ze09N0ZP09D:',
 '0ZN09C0ZO09?0ZQ08R0ZZ0860Z`07Q0Z;06V0Y`06:0Y^06;0Z406n0Z\\07b0^H0G', 
 '7`0`]0750`]0750MK0;B0IZ0<C0Hm0;T0H609V0H309S0Gg09i0G`0:70GW0:;0GZ', 
 ']0:<0G]0:30GR09k0GG0:00GF0:20GA0:60G@0:>0GD0:D0GQ0:G0GS0:C0GE0:9a',
 '0GE0:90GC0:=0G?0:D0GA0:B0GA0:C0GC0:C0GC0:C0G50:60G@0:10GF0:20GF0O', 
 ':20GI0:20GE09n0GD09j0GE09n0GE09n0GA09[0GA09[0GH09R0GD0940GN0940Gb',
 '\\0960H<0:K0H<0:K0H=0:W0H80:?0H809`0HC09g0HC09_0H<0900HD0970HQ08lY', 
 '0H_08d0IU0970Ii09F0J609H0JU09X0L=09N?om0000[Q0820Y708^0U609_0Q10?',
 ':60O_08K0OA07H?om000?om000?om0000Q>06K0Q>06K0R>06c0T^06=?om0000V@', 
 'i08C0VY07M0XL07Y0Z00640Yi0660Yc0680Yk06A0Yo06@0Y[0670YU05n0Y[060<',
 '0Yb0620Yb0620YZ05j0YZ05j?om000?om000?om000?om000?om000?om000?om0g',
 '000`T06R0`=06V0`J06]?om000?om000?om0000c706F0cH06D0dA06;0d;05k?oQ',
 'm000?om000?om000?om000?om000?om0000bM06n0cB0;h0cM0EY0bX0<L0bI0:ma', 
 '0b60910bL08n0c;0870e207R0gj06]0ho06C0iJ05d?om000?om000?om000?om0_',
 '00?om000?om000?om000?om000?om000?om00007k06T07U06f07U06f07T06I?oF',
 'm000?om000?om0000:]09P0;?0=40;F0=_0;<0=h0;70=m0;30>00;80><0;60>E`', 
 '0:e0>70:c0>60:`0>20:H0>10:60=h09c0=M09]0=C09l0=V09l0=V09f0=T09d0>', 
 '>709V0>P09A0>I08o0=V08d0=b08H0;108>0;70860:\\08<0;d0870<207n0<X07`', 
 'f0<M07b0<L07P0<_07:0<>06Q0;Z05U0;m0550>Y05:0A305L0GG05I0HI05B0IGT', 
 '0580Id0510JH0550Ja0550JX0520Jj0500K404m0K804j0K=04j0KC04j0KB04k0B', 
 'KT04n0KT04k0KU04l0K]04l0K^04l0Ka04l0Kb04l0K^0520K_0510KX0510KR05D', 
 '10KR0510KT0510KP0510KK04k0KD04j0K804k0K904o0Jk0550J`0550J80530J0\\',
 '0510HN05@0HQ05@0F=05?0@g05B0>O04m09]04m08V05507l05O07i05c08706N0S', 
 '8T07E08a07L09107S08j07b08^07i08c07o08[08A08]08C08Y08G08Y08N08V083',
 'O08Y08J08J08O08G08T08L08V08I08]08K08\\08D08[08A08]08?08_08608^0895',
 '08\\08708\\08708h08=08f08908h08;08`07k08^07m08`08208g08808i08008k0>', 
 '7n09107m09107j09408009807o09607k09807c09807c09@08209808009:08409E', 
 '@08P09C08^09C09B09U0:l09Z0<_09Z0<_09G09J09E08@09T071?om000?om000E',
 '?om000?om000?om000?om0000:S06S0:507I0:807j0:008_09l08a0:108l09b0=',
 '8[09V08X09P08R09S08Z09Y08c09S08^09R08o09K08d09D08U09J08O09@08A09c', 
 'A08B09C08M09D08W09@08n09809309308l09=09009809:08l08c08e08Z094096A', 
 '09208d08o08W08o08X08o08T08m08O08k08J08j08N08c08R08c08R08c08\\08h0I', 
 '8j08m09808i08m08e08a08d08Z08^08W08X08N08P08F08B08;08D08>08E08>08M',
 'J08W08N08[08J08Y08B08Q08:08I08<08J08808D08908C08;08G08>08M08;08GH', 
 '08508908808D07n08307m07j07h07a07g07l07a07`07h07l07j08607i08207b0J', 
 '7d07b07d07b08207d08107_07o07V07^07W07f07X07h07U07o07V08207W07n07@', 
 'd07m07d07f07U07c07V07m07V07m07R07j07P07m07N07i07L07o07O08?07M08AE', 
 '07P08M07N08P07V08m07b08b07a08X07J08107J08207B07X07F08I07909107?0a',
 '9`07C09M07?07m07<07Y07Q08@07M08807D07o07C08707H08O07H08O07E08B07M',
 'F08?07A07\\07F07=07J06c07>06507;05k07H06`06f07506k07=07V06k07T06NT',
 '07d06Z?om000?om000?om000?om00009n05l0:>06U09j08H0:B0@>0:<0An0:?0m', 
 'Cl0:80>I09k09M09d08;09i07j09d07U09f07J09h06X?om0000eT06a0eU06b0eO',
 'H06b0;Q05k0<\\07Q0?:0>I0>n0=o0=m0:l0=N0:>0=@09K0=J09o0=@09K0<M087`',
 '0;S0720;G0780:o06O?om000?om000?om000?om000?om000?om000?om000?om0^',
 '00?om000?om000?om000?om000?om000?om000?om000?om000?om000?om000?ol',
 'm000?om000?om000?om000n', '', '']

data_3 = ['GE0000108000\r00P', '2NDPD', '0K>0<o0KT0=10Ka0<i0Kf0<o0Kb0=00K\\0<o0KW0=40KU0=10KM0<o0KM0=<0KL06', '<C0K009G0JL0770JA06b0J406G0J406G0J506D0J106E0Ie06C0Ic06I0I_06H0Ig', 'Z06I0IP06F0IM06H0IF06D0I606L0H\\06^0HB07`0HI0?H0HA0Hc0H50Jj0H60LBN', '0Gm0Ki0H40L30H20KW0H30K]0H40K50H80FF0HE0:l0HJ06n0HM06W0HP06V0HJ0;', '6S0HS06X0HS06X0HR06W0HM06W0HH06Q0HH06Q0HH06Q0HE06P0HE06T0HD06Q0HN', 'F06R0HD06R0HD06W0H=06U0H506Y0H606T0H606T0H@06S0H@06S0H<06U0Gk06Q1', '0Gk06T0Go06U0H006V0H006X0Gg06T0Ge06T0G`06W0G^06W0Gh06_0Gh06_0Gj0d', '6[0Gf06V0Gk06[0Gg06V0G_06X0G_06X0GZ06[0GR06Y0GV06f0GV06f0GJ06Y0GB', 'J06Y0GM06Z0GP06S0GS06Y0GD06X0GD06X0GD06V0GH06T0GD06V0G?06T0G906\\A', '0G>06W0G=06Z0GC06_0GG06]0GH06Y0GC06_0GD06a0GE06T0GC06_0GA06]0G=0i', '6^0G706[0G706[0G?06^0GB06e0G106`0G306_0G706e0G706W0G306X0G906^0G4', '=06`0G?06Y0GL06f0GL06f0G:06Z0G<06Z0GE06b0GB06]0G;06Y0G?06`0G=06]N', '0GD06[0GF06^0GH06`0GI06]0GN06a0GC06c0GI06a0GK06X0GK06U0GK06S0GB0l', '6]0GL06W0GG06M0GL06R0GQ06W0GP06S0GL06P0GG06L0GJ06U0GK06U0GL06R0GH', '\\06Y0GR06T0GM06M0GU06L0GY06O0GY06W0GY06W0GU06O0GZ06O0G_06M0G^06P=', '0GZ06M0GZ06M0GZ06L0Gd06O0G`06L0Gl06O0Gj06M0H006P0Gd06G0Gc06E0Gb04', '6I0Gl06H0H206F0Gl06C0Gn06K0Go06H0H406D0H:06F0H906C0H806C0H706B0HB', '906E0H=06D0H@06G0HE06G0HG06E0HV06I0HV06I0HU06G0HT06F0HV06C0H]06FP', '0Hb06E0H[06D0HV06F0HZ06F0H^06D0H_06D0Hc06D0Hf06A0Hg06?0Hm06B0I10`', '6A0I506C0I006?0I006?0I906C0IE06@0IF06A0IH06C0ID06C0ID06A0IF06=0IN', 'O06=0IT06>0IW06;0IP06C0IV0680I[06:0Ib06=0Ig06:0In0670In0670J606=0', '0J20680J:06;0J:06;0J:06;0J;0680JC0640JD0620JM06>0JP0680JR0640J^0:', '6;0Jg0600Jg0600Jh05l0Jo05o0K305k0K905o0KA0620K?05m0KC05o0KK0600KN', 'E0600KN05o0KT05j0KY05o0K^05m0Ko05j0Kk05d0L305h0L?05k0LG05h0LI05eg', '?om000?om0000LL05e0LS05f0L\\05f0La05f0Ln05j0M205i0M405f0MA05g0M?0e', '5e0MF05d0M[05h0M_05h?om0000N40640N@06P0NG08F0NU0:\\0NU0:\\0Nd0:d0Nb', 'l0:b0Nm0:`0O00:_0O90:]0O@0:[0OH0:Y0OO0:V0O_0:Y0Oc0:Y0Oh0:W0Oh0:WF', '0P70:Q0PG0:O0PS0:V0PV0:W0PW0:]0P\\0;[0PC0<I0Ob0D[0OV0I^0OP0IG0OU03', 'J`0OO0Jk0OU0IA0O`0I60O]0BA0OQ0:Z0OQ0:Z0OD0:T0OA0:[0O90:Q0Ni0:`0NH', 'i0:`0Nd0:\\0NS0:a0NO0:f0NI0:Z0ND0:f0N?0:n0N30:h0Mi0:b0Mf0:e0M\\0:fF', '0MV0:j0MI0:n0MI0:n0MN0:`0MP0:I0M^0:70Mg0:50Mk0:50N>09k0NF09j0NN0Q', '9j0NT09c0N\\09d0Nb09V0O=09X0OH09W0OR09V0O\\09S0Oa09I0P209K0PD09E0PL', 'K09J0PO09B0PS09@0Pi0920Q008m0Q:08n0QP0910QW08i0R008o0RC08o0RL08kY', '0RV08h0S309@0S609I0SZ0:70TE0:P0U<0;V0Vk0;Y0W=0;V0U30;k0S^0:h0SS0f', ':^0SC09`0Sh09i0Sc0:00S@09e0Rf0980Rj09V0Re09X0R_09b0RQ09Z0RR09d0Rf', 'I09k0R;09j0R30:20R30:20R70:40R30:40Ql0:70Qb0:80Q]0:80QW0:;0QT0:B:', '0QM0:>0QB0:@0PP0:S0M;0;:0L@0;70I60;50HL0;:0Gg0;U0G\\0;U0GW0<90GZ08', '<S0GF0<O0GF0<Q0GC0<[0GF0=20GE0=:0GJ0=@0GI0=D0GC0=@0GC0=<0GB0=@0G7', 'C0=@0G<0=80G=0=F0G60<a0G50<e0Fk0<l0Fl0<d0Fk0<f0Fl0<_0Fe0<o0Fb0=0L', '0Fb0=20F`0=:0F`0=:0Fh0=Q0Fm0=Q0Fe0=N0Fa0=N0F^0=Y0F]0=Y0F_0=c0Fa0e', '=f0Fa0=l0Fc0>60FY0>10FY0>?0FR0>90FJ0>=0FP0>A0FS0>H0FQ0>L0FO0>T0F6', 'M0>]0FM0>]0FP0>Z0FJ0>b0FF0>j0FJ0?:0FJ0?:0FB0?B0F<0?A0FA0?N0FC0?EF', '0FD0?W0FE0?Y0FI0?f0FH0?l0F@0?i0F@0?n0FE0@20FK0@?0FH0@E0FC0@20F>0@', '?N0F@0?<0FB0>N0FD0>>0FB0=M0F:0<]0F<0=X0F=0=k0FE0>`0FD0?40F>0?U0F9', '?0?_0F50?]0F80@50F<0@G0FB0@S0FA0@\\0FC0@k0F;0@l0F70@o0F60A40F70A6@', '0F90A80F;0A80F90A80F<0A90F:0A90F>0A;0F;0A50F>0A50F>0@m0F;0@f0F;0:', '@f0F=0@^0F@0@Z0F>0@X0F=0@V0F<0@Z0FA0@O0FC0@O0F?0@M0F60@H0F90@B0Fc', ';0@50F@0@?0FB0@J0FG0@F0FF0@S0FE0@_0FD0@_0FF0@[0FH0@[0FH0@[0FJ0@QF', '0FH0@P0FG0@F0FA0@F0F=0@G0FK0@F0FL0@H0FK0@G0FG0@H0FG0@B0FG0@B0FG0a', '@80FF0@30FF0@20FH0?i0FN0?c0FQ0?a0F\\0?`0F_0?^0F^0?Z0F^0?Z0F[0?Z0F5', 'a0?N0F\\0?M0Fc0?C0Fe0?:0Fh0?80Fe0>o0Ff0>d0Fi0>i0Fi0?<0Fg0?40Ff0?>_', '0Fd0?C0Fd0?:0Fi0?80Fm0?20G60>g0G90>[0G80>W0G90>\\0G90>W0G=0>R0G<0>', '>E0G;0=>0G;0=>0GD0<k0GY0;B0H\\0;E0K:0;b0K^0<G0L80<K0LH0<o0LE0<f0Lg', '?0<@0LA0<O0LL0;g0LO0;l0LL0;R0LY0;M0L]0:b0M50;40M]0<P0O00<V0QI0<6J', '0d\\07m0dh0810df07g0dn07o0e507g0e:07c0eP0840ee0870fP0930fR0910fh0f', '9D0fk08`0g008h0gL09R0gb09]0hF0:=0hZ0:B0hh09U0i=08k0kA09D0nG09517h', '=07O16d07\\13h08\\12E08h11l08d11b08\\11B08m0mV09D0kf0920k808\\0jK087k', '0j@08>0j=08K0j=08L0jJ08M0jj09E0l20;c0lj0<C0o;0;k1BX07h1Bm07o1C90>', '7l1C@07m1CW07c1Cn07m1D707k1DS07o1DT07d1Dh07c1E707i1EP07b1ER07Z1EH', 'l07f1F@07f1FM07a1F`07e1G;07_1GP07_1G\\07d1Gd07^1HA07_1HS07_1HT07V?', '1I=07\\1IL07Z1Ic07U1J307Z1JO07\\1K107T1K?07U1KP07R1Kl07O1L:07H1LM0h', '7F1Lh07G1MJ07P1MW07H1N107I1N70701N306Y?om000?om000?om000?om000?o;', 'm000?om000?om0001IZ05m1I706N1I706N1H]0651HJ06M1H;06N1H806G?om000K', '?om0000R505d0QT07<0Q]0860Q`08M0Q]08E0Qn08L0RH08M0RW08I0Rf08L0S=0h', '8T0SP08V0S^08\\0T308U0TA08G0TS08J0TU08H0Td08D0To08A0U<08V0UH08V0Uk', 'J08C0Ud08U0Uk08Y0V;08I0VC08K0VM08U0VZ08I0Vb08J0Vg08A0W907I0WL06Ei', '?om000?om0001Hc0681Hj0661I2065?om000?om000?om000?om000?om000?om0h', '00?om000?om000?om0001F=05f1En0631EV06L1EZ06B1E]067?om000?om000?o7', 'm000?om000?om000?om000?om000?om000?om000?om000?om000?om000?om000[', '?om000?om000?om000?om000?om000?om00018g06R18g06R18I07Z18F07a18:0?', '7i18708218807k17m08017^08317J08117P08@17A08617608217008?16h08816J', 'b08416S08;16M08=16M08>16J08;16E08;16=08>15o08?15k08?15k08?16008AW', '16407B17606a18L06o19=07<19@06j19G06=?om000?om00019=06419@07e?om0S', '000Kc08S0J=0<H0Hi0DA0Hh0IQ0H_0Je0H^0KZ0H]0K^0H_0K_0H_0K20Hf0JJ0H2', 'o0G\\0IB0>20JF0:H0L50:20Mg09M0QE08^0Qb08a0Qj08Q0RA08d0RG08a0RM08d>', '0RD08R0R:08A0RA08M0RI08Y0RF08M0RF08M0RF08[0R=08T0RN08V0RU08_0RT0K', '8T0S109g0Rn0:10S>0;@0SO0<H0SR0<[0SO0<10SS0<R0SX0=V0SY0=T0SP0;V0Sn', 'Q0;<0Re08l0R[08R0R`08Z0S00850S006F17<06[17@06[17@06a17>06`17A06ej', '17A06e17C06k17@06f17@06f16o06i16^06k16`07816Q07a16D0;416V0Mj16=0k', 'OK16R0NL16L0<F16G09416U07T17107617;07318G09i17N09e17008i17708L16P', 'f06:?om000?om000?om000?om000?om000?om000?om000?om000?om000?om000d', '?om000?om000?om000?om000?om000?om000?om000?om0000_M08O0`?0;B0_\\02', '9M0`F0<20_m09n0_b09J0_U08c0_V08f0_J08I0_J08Y0J50<J0Hd09M0Hl09@0Id', '<08T0J?08M0JX08E0JH0860J<07Q0K?08=0KW08U0K[08\\0L308a0L=08m0LH09;Y', '0LV0920Lj0900K`06N0Ki06H0Kk06A0L=06?0L?06?0LA06:0LP0670Lf06;0M503', '610MH05o?om000?om000?om000?om000?om000?om000?om000?om000?om000?oa', 'm000?om000?om000?om000?om0000LZ0740L;08V0L<09H0L609I0LN0:60N20=H[', '0PE0>30gn09B0h409A0h609D0h309A0go09B0g]08T0gD07h0g:07`0g507L0g;0P', '7V0gB07f0gG0820gG0820gW08K0gT08D?om0000I70=_0HG0=C0H00=X0Gk0=f0G:', 'b0=_0G\\0=?0Fj0;20Ei09=0E\\09;0EQ09J0EK09H0EL09M0EG09\\0EF09[0ED0:3J', '0EA09j0E>0:U0E80:V0E@0;c0EG0=Q0ED0=K0EA0=<0E=0<l0EC0<k0EC0<k0EH0:', '<h0EH0<h0EN0<e0EK0<e0EH0<h0EC0<h0EH0<h0EE0<P0EE0<P0EJ0<b0EK0<@0ER', 'G0<70EU0<?0EU0<<0EV0<20EY0<:0EW0;l0EZ0;d0E^0;l0Ed0<20E_0;m0Ec0;hW', '0Ec0;h0Ea0;Y0E]0;D0Em09\\0G109R0IZ08k0I_09J0Ic09i0Ic09I0J909;0JJ08', '9:0K80;X0K20<B0K<0<f0K90<e0Jf0<Y0G^0<O0FM0<a0D\\0<h0AV0<<0A@0<C0@j', '[0<J0@U0<70@O0;d0?j0;J0?b0;S0?`0;O0@10;P0@M0;S0@W0;O0@`0;[0@W0;@=', '0@W0;C0@I0:<0?V08k0?308j0?00930>i08h0>Y08N0>c08Y0>n08>0?N07B0?X0`', '6g0?j07>0@707]0?L07G0?L07G0?V07e0?R07`0?J07f0?=08E0?[0;\\0?@0:O0?h', '=0:S0?:09g0?E0:80?K0:E0?T09?0@I09R0@Z09Q0A10930@k07S0A207L0@U071K', '0@>06]0?a07M0?L07h0?@07_0?307l0>_0860>d0810>f07_0?207f0??07Y0?P0T', '7i0?U07k0?N07`0?Q07R0@306P?om000?om000?om000?om000?om000?om00016e', 'C06>13g06Q13b06Z13[06W[', '', '']

update_di(test_data)

