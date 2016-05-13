import socket
import binascii as baci
from collections import deque
import matplotlib.pyplot as plt
import numpy as np

TCP_IP = '192.168.0.10'
TCP_PORT = 10940
BUFFER_SIZE = 8192 #4096
version = 'VV\r'
parameters = 'PP\r'
state = 'II\r'
scan = 'BM\r'
stop = 'QT\r'
values = 'GD0000108000\r'
state2 = '%ST\r'
cont = 'GE0000108000\r'

def time_val(value):	
	return ((ord(value[0])-48)<<18)|((ord(value[1])-48)<<12)|((ord(value[2])-48)<<6)|(ord(value[3])-48)

def dist_val(value):	
	try:	
		return ((ord(value[0])-48)<<12)|((ord(value[1])-48)<<6)|(ord(value[2])-48)
	except IndexError:
		print 'error ', value	
		return 0		
		#s.shutdown(2)
		#s.close()	
		#plt.close('all')	
		#print 'END'		
		#exit()

def update_d(answer):		
	answer = answer.split('\n')		
	time = answer.pop(1)[:-1]
	dist = answer[1:-2]
	dist2 = [item[:-1] for item in dist]
	dist4 = ''.join(dist2)			
	idxl = 0			
	idxh= 3	
	polar_graph = deque()	
	angle = np.arange(0,len(dist4)/12.0,0.25)
	angle = np.radians(angle)	
	while idxh <= len(dist4):
		point = dist_val(dist4[idxl:idxh])
		polar_graph.append(point)
		idxl = idxh
		idxh += 3				
	if len(angle) != len(polar_graph):
		print 'not same length'
		return None	
	update_polar_plot(angle, polar_graph)

def update_di(answer):		
	answer = answer.split('\n')		
	#time = answer.pop(1)[:-1]
	dist = answer[2:-2]
	dist2 = [item[:-1] for item in dist]
	dist4 = ''.join(dist2)			
	step, idxl, idxh = 0, 0, 3			
	polar_graph = deque()
	angle = deque()	
	while idxh <= len(dist4):
		point = dist_val(dist4[idxl:idxh])
		#strength = dist_val(dist4[idxh:idxh+3])
		if dist_val(dist4[idxh:idxh+3]) > 1100 and point < 4000:	
			polar_graph.append(point)
			angle.append(step)
		idxl = idxh + 3
		idxh += 6		
		step += 0.004363323129985824
	print polar_graph	
	try:
		update_polar_plot(angle, polar_graph)
	except:
		print 'not same length'

def read(answer):
	answer = answer.split('\n')		
	dist = answer[2:-2]
	dist2 = [item[:-1] for item in dist]
	dist4 = ''.join(dist2)			
	step, idxh = 0, 3 			
	polar_graph = deque()
	angle = deque()	
	lend = len(dist4)
	while idxh <= lend:
		point = dist_val(dist4[idxh-3:idxh])
		if dist_val(dist4[idxh:idxh+3]) > 1600 and point < 3200:	
			polar_graph.append(point)
			angle.append(step)
		idxh += 6		
		step += 0.004363323129985824	
	combined = zip(angle, polar_graph)
	y = 1
	prev = 0
	beacons = []
	for i in xrange(1,len(combined)):
		if ((combined[i][0] - combined[i-1][0]) > 0.2 or i == (len(combined)-1)) and y<4:
			beacons.append(beacon_num(i,y, prev, combined))
			y += 1
			prev = i
			print 'beacon in the loop: ', beacons
	try:
		update_polar_plot(angle, polar_graph)
	except:
		print 'not same length'
	return beacons

def beacon_num(index, number, prev, combined):
	if number == 1:
		return np.mean(combined[:index], axis = 0)
	if number == 2:
		return np.mean(combined[prev:index], axis = 0)
	if number == 3:
		return np.mean(combined[prev:], axis = 0)
	
def init_xy_plot():
	""" setup an XY plot canvas """

	plt.ion()
	figure = plt.figure(figsize=(6, 6),
						dpi=160,
						facecolor="w",
						edgecolor="k")
	ax = figure.add_subplot(111)
	lines, = ax.plot([],[],linestyle="none",
						marker=".",
						markersize=3,
						markerfacecolor="blue")
	ax.set_xlim(-5000, 5000)
	ax.set_ylim(-5000, 5000)
	ax.grid()
	
def update_xy_plot():
	""" re-draw the XY plot with new current_frame """

	lines.set_xdata(self.current_frame.x)
	lines.set_ydata(self.current_frame.y)
	figure.canvas.draw()

def init_polar_plot():
	""" setup a polar plot canvas """

	plt.ion()
	figure = plt.figure(figsize=(6, 6), 
						dpi=160, 
						facecolor="w", 
						edgecolor="k")
	ax = figure.add_subplot(111, polar=True)
	lines, = ax.plot([],[], 
					linestyle="none", 
					marker=".", 
					markersize=3, 
					markerfacecolor="blue")
	ax.set_rmax(4000)
	ax.set_theta_direction(1) #set to clockwise
	ax.set_theta_offset(-np.pi/4) #offset by 90 degree so that 0 degree is at 12 o'clock
	#ax.grid()
	return figure, lines

def update_polar_plot(angle, dist):
	""" re-draw the polar plot with new current_frame """

	lines.set_xdata(angle)
	lines.set_ydata(dist)
	figure.canvas.draw()

if __name__ == '__main__':
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))

	figure, lines = init_polar_plot()

	while True:
		try:
			msg = raw_input('Command: ')
			if msg == 'l':
				s.send(state2)
				data = s.recv(BUFFER_SIZE)	
				print data			
			if msg == 'v':
				s.send(version)
				data = s.recv(BUFFER_SIZE)	
				print data
			elif msg ==	'p':
				s.send(parameters)
				data = s.recv(BUFFER_SIZE)
				print data
			elif msg ==	's':
				s.send(state)
				data = s.recv(BUFFER_SIZE)
				print data	
			elif msg ==	'b':
				s.send(scan)
				data = s.recv(BUFFER_SIZE)
				print data
			elif msg ==	'q':
				s.send(stop)
				data = s.recv(BUFFER_SIZE)
				print data
			elif msg ==	'g':
				s.send(values)
				answer = s.recv(BUFFER_SIZE)
				print type(answer)
				print answer
				d = deque(answer)
				print type(d)
				print d			
				answer = answer.split('\n')
				print 'data ', answer	
				print 'len ', len(answer)		
				time = answer.pop(1)[:-1]
				print time			
				print time_val(time)
				print 'data ', answer
				dist = answer[1:-2]
				print dist
				dist2 = [item[:-1] for item in dist]
				print 'list comprehension ', dist2
				dist3 = deque(''.join(dist2))
				dist4 = ''.join(dist2)			
				#dist3 = [dist_val(dist
				idxl = 0			
				idxh= 3	
				polar_graph = deque()	
				while idxh <= len(dist4):
					point = dist_val(dist4[idxl:idxh])
					polar_graph.append(point)
					idxl = idxh
					idxh += 3	
				print 'polar ', len(polar_graph), '\n', polar_graph		
				angle = np.arange(0,len(polar_graph)/4.0,0.25)
				angle = np.radians(angle)			
				print angle	
				print 'len dist %i, len angle %i' %(len(polar_graph), len(angle))			
				update_polar_plot(angle, polar_graph)
			elif msg == 'c':
				while True:
					s.send(values)
					answer = s.recv(BUFFER_SIZE)
					update_d(answer)
			elif msg == 'n':
				while True:
					s.send(cont)
					answer = s.recv(BUFFER_SIZE)
					update_di(answer)

			elif msg == 'r':
				while True:
					try:
						s.send(cont)
						answer = s.recv(BUFFER_SIZE)
						beacons = read(answer)
						print 'beacons2 :', beacons
					except:
						s.shutdown(2)
						s.close()	

			elif msg == 'k':
				s.send(cont)
				answer = s.recv(BUFFER_SIZE)		
				answer = answer.split('\n')		
				print 'answer ', answer
				time = answer.pop(1)[:-1]
				dist = answer[1:-2]
				dist2 = [item[:-1] for item in dist]
				dist4 = ''.join(dist2)
				print 'dist4 ', dist4
				print 'len dist4 ', len(dist4)			
				idxl, idxh, maxs = 0, 3, 0			
				polar_graph = deque()
				#angle = np.arange(0,len(dist4)/24.0,0.25)
				#angle = np.radians(angle)
				angle = deque()
				stre = deque()
				step = 0
				print 'len angle ', len(angle)	
				while idxh <= len(dist4):
					point = dist_val(dist4[idxl:idxh])
					strength = dist_val(dist4[idxh:idxh+3])
					if strength > maxs:
						maxs = strength
					#print 'strenght ', strength
					if strength > 1200 and point < 4000:	
						polar_graph.append(point)
						angle.append(step)
						stre.append(strength)
					idxl = idxh + 3
					idxh += 6		
					step += np.radians(0.25)	
				print 'strenght ', maxs
				print 'polar graph: ', polar_graph
				if len(angle) != len(polar_graph):
					print 'not same length, angle len %i, polar_graph len %i' %(len(angle), len(polar_graph))
				try:
					update_polar_plot(angle, polar_graph)
					print 'max strength ', maxs
					print zip(angle, polar_graph, stre)
				except:
					print 'not same length'	
		except KeyboardInterrupt:
			s.shutdown(2)
			s.close()	
			plt.close('all')	
			print 'END'
			break

# test_data = ['GE0000108000\r00P', '9BI\\P',
#  '0VM06=0VI06A0VW05o0VW05o0VW05o0VW06]0VD06`0V@06b0VA06n0V:07<0UF0f',
#  '740RS07m0MM08`0LL08f0LB08_0L?0920L?0920L=0950L10940Ko0980Ke0950KB', 
#  '_09:0KX0980KL08e0KS09>0KG08d0K908a0Jm08`0Jl08j0Je08b0J^08Z0JJ08P`',
#  '0JO08b0JE08\\0JA08_0J808Z0J208^0I^08T0IY08U0IL08W0I308K0I808b0Hi0?', 
#  '8h0HX08i0HK0900H708g0Ge08m0GT08m0GH08l0G80930G409W0Fc09N0F_09M0F6',
#  'S07h0FN06C?om000?om000?om000?om000?om000?om000?om000?om000?om000B',
#  '?om000?om000?om000?om000?om000?om000?om000?om0000`00720]D0:b0]R0M',
#  'C@0]Q0B80]N0@J0\\M07V0\\<06^0]206^0^706d?om000?om000?om000?om000?o^',
#  'm000?om000?om000?om000?om000?om000?om0000hC08>?om000?om000?om0001',
#  '?om000?om000?om000?om00004405f03Y06803U06N03[06b03Q07403Y07Z03V0@', 
#  '7l03M08S0470<504;0@60490DU0480FM04:0H>0480HC0450I[0480Ig04>0J004H', 
#  '90J>0440JG0410JU0410JW0410J^03h0J_03d0J^03d0Jb03d0Je03h0Jf03h0JfS', 
#  '03e0Jk03g0Jk03j0Jj03j0Jl03j0Jk03k0Jk03k0J`03k0J`0400Jn03k0Jh03l0P', 
#  'K303k0K003e0K203e0Jn03g0Ja03n0J^03j0J^03j0J^03j0JZ03k0JV0410JQ04g',
#  '10JQ03k0J603g0I\\0430IX04@0IO0470HI0480H<04;0Ga04F0Fc04>0CD04F0>YM', 
#  '04K0<U05K0<K05f0<N0650;106G0:]06L0:106[09G06o09G07N09A07n09=08?0k',
#  '9:08e09A0:808a0;F0:N0<L0:@0>U0=C0?X0=[0H:0;J0HA0;U0HD0;e0H?0;?0HW', 
#  '@0;k0G_09`0GT0:80GR0:Z0GN0;80GR0;L0GR0;L0GK0;A0GO0;Z0GO0;Z0GN0;SN',
#  '0GN0;S0GO0;[0GN0;U0GF0;A0GI0;70GR0:e0GR0:i0GF09n0GJ09:0GT08h0H806', 
#  '8S0I108F0I>08@0I408C0Hn08T0I<08i0IJ09>0IJ09>0IC0910IH0920IZ08m0JZ', 
#  '008h0J908B0JV0800KO08A0Ka08E0Ka08E0LP09Q0Lo09\\0Nd09J0Q[09g0WP08;k', 
#  '0W;07V0Uj0750U<0740T=07K0Sn07M0Sg07J0S_07O0T?07=0U206k0Ti06[0UM0@',
#  '6l0Un07B0V007F0Uc07J0Uc07^0U\\07M0UR07A0Ua07\\0UY07W0UZ08E0U>07m0TW', 
#  'o07f0Tm08:0Tj0840Tl08L0Tl08V0TQ0860TG07i0TP08D0TA0850T@08I0TG08`S',
#  '0Sk07l0QS0880O008_0M\\0860LX07_0LE07_0KJ07l0I>08N0H309G0Gn0:80Gj0k',
#  ':Z0Ga0:_0G`0:`0G[0:_0GV0:n0GN0:e0GM0:l0GN0:f0GJ0:e0G>0:Z0G?0:[0GD',
#  '30:S0G50:U0G00:[0G10;40Fo0;E0Fd0;=0Fc0;C0Fe0;E0Ff0:m0F^0;20FQ0;1n', 
#  '0FU0;F0FN0;D0FN0;U0FK0;C0FW0:Q0FP08l0F^08a0Fl08S0G508W0GI08Y0G^0_',
#  '8[0Ha09n0Ha09n0HG09M0HM08a0HQ08W0HV08Q0I108[0I`09D0Jm09h0Mg09V0PC', 
#  '90920P508M0P>08W0PD08a0P>08X0P008B0Of0890Oc07k0OX07o0OX08?0Og08@C',
#  '0PN08;0Q?08L0Rm08o0U[0910Wo08U0X408U0X508Z0Wj08Y0Wb08S0Wa08T0Wi0Z',
#  '8W0Wc08\\0WV08Y0W?08a0Jm0;C0GM0;K0FW09b0FF09:0E_07l0EQ07c0EH07[0EK',
#  'H07[0ED07W0EG07a0EG07a0E=07T0EI07k0E>07`0EC07j0E707d0E107j0E107i]',
#  '0E107h0E107[0E607j0E307n0E10800Di07j0Dl0840Do08:0Dj08;0DX08Y0DP05',
#  '9@0D]0:P0D[0:e0DU0;00DU0;00DW0;00DW0;00D[0;B0DY0;L0DR0;X0DR0;X0Dl',
#  'W0;e0DZ0;f0D\\0<M0DZ0<R0DZ0<a0DY0<j0D[0=>0D_0=g0Da0>B0DV0>[0DW0>hc',
#  '0D[0?M0D^0?a0D^0?e0D^0>h0D^0>h0D]0>[0D]0=T0D\\0=B0D\\0=B0D_0<f0D^0d',
#  '<\\0DY0<<0DV0<00DT0;j0DT0;j0D\\0;]0D\\0;L0D\\0;L0D[0;G0DY0;;0DZ0;G0Dk',
#  'Z0;80D\\0;60D]0;10D^0:h0Dc0:i0D`0:_0Dd0:_0De0:_0Di0:^0Dd0:Y0De0:RN', 
#  '0Dn0:D0E30:E0E20:D0Di09m0Dh0:40Dg09d0Dg09Y0Dl09a0Dj09M0Dm09F0Do0?',
#  '9?0E209>0E60920EB08Z0EE0850EK07n0ES07i0EN07d0EJ07^0EY07d0E]07d0E6', 
#  'Z07c0EY07_0E_07f0Ej07e0Eo07d0F207c0F707h0F707_0F>07b0FM0870FF07dj',
#  '0FW0820F\\0850GU09o0H=0;20Hb0;>0Jg0;j0TS0:_0U@0:90Um09f0Wc09N0Z]06',
#  '8i0Zm08Y0[408W0[808X0[808X0[<08T0[=08M0[C08U0[O08M0[J08M0[U08\\0[`',
#  'V08Y0[X08Q0[d08]0[f08R0[f08R0[m08W0[m08P0\\208K0\\C08Z0\\I08Y0\\S08eX',
#  '0\\N08X0\\\\08a0\\a08\\0\\g08V0\\l08`0\\c08[0\\H08a0Uj0:50T>09d0SI08m0Rm0I', 
#  '8Q0R107W0Q]07Y0QN07a0QL07b0Pl07Z0Pb07j0Pb07o0P\\07>0QD06S?om000?oe',
#  'm000?om000?om000?om0000SB06W0TB06c0TF06U0TI06G?om000?om000?om000?', 
#  '0TL05o0T105k0TY07>0S107b0PK08h0OM0970Nl09=0N[09;0NX09<0NN09N0NP0h', 
#  '9^0NA09=0NA0950ND0900NT08e0O=08m0P408e0R808J0Rm08>0S20880Rb07n0Rf',
#  'U07l0RQ0860RH07j0RK07`0RT07\\0RU07^0R[07o0R_0850SH08Q0S]08L0UE088`',
#  '0WD07]0Wg0830Wo07f0X?0830XK07[0XR07:0XW0750Xb07=?om000?om000?om0g',
#  '00?om000?om00007P05d07H06P07M07208a0=G08L0=Z07n0>M07\\0CS07_0HM07[', 
#  'X0JN07Q0Ki07N0LJ07M0LM07@0LU07>0La07;0M807>0MN07>0MT07>0MT07>0MYL', 
#  '07F0Mi07A0M]07A0M]07>0MW07E0ML07E0ML07H0M<07?0Lf07Q0L907E0Ko07P0D', 
#  'L107P0K<07V0KI07W0IJ07X0FA07a0BC0850>G08Q0>P07:07A07306L07O05j?oa',
#  'm000?om000?om000?om000?om000?om000?om0000[N08j0[=09B0Ze09N0ZP09D:',
#  '0ZN09C0ZO09?0ZQ08R0ZZ0860Z`07Q0Z;06V0Y`06:0Y^06;0Z406n0Z\\07b0^H0G', 
#  '7`0`]0750`]0750MK0;B0IZ0<C0Hm0;T0H609V0H309S0Gg09i0G`0:70GW0:;0GZ', 
#  ']0:<0G]0:30GR09k0GG0:00GF0:20GA0:60G@0:>0GD0:D0GQ0:G0GS0:C0GE0:9a',
#  '0GE0:90GC0:=0G?0:D0GA0:B0GA0:C0GC0:C0GC0:C0G50:60G@0:10GF0:20GF0O', 
#  ':20GI0:20GE09n0GD09j0GE09n0GE09n0GA09[0GA09[0GH09R0GD0940GN0940Gb',
#  '\\0960H<0:K0H<0:K0H=0:W0H80:?0H809`0HC09g0HC09_0H<0900HD0970HQ08lY', 
#  '0H_08d0IU0970Ii09F0J609H0JU09X0L=09N?om0000[Q0820Y708^0U609_0Q10?',
#  ':60O_08K0OA07H?om000?om000?om0000Q>06K0Q>06K0R>06c0T^06=?om0000V@', 
#  'i08C0VY07M0XL07Y0Z00640Yi0660Yc0680Yk06A0Yo06@0Y[0670YU05n0Y[060<',
#  '0Yb0620Yb0620YZ05j0YZ05j?om000?om000?om000?om000?om000?om000?om0g',
#  '000`T06R0`=06V0`J06]?om000?om000?om0000c706F0cH06D0dA06;0d;05k?oQ',
#  'm000?om000?om000?om000?om000?om0000bM06n0cB0;h0cM0EY0bX0<L0bI0:ma', 
#  '0b60910bL08n0c;0870e207R0gj06]0ho06C0iJ05d?om000?om000?om000?om0_',
#  '00?om000?om000?om000?om000?om000?om00007k06T07U06f07U06f07T06I?oF',
#  'm000?om000?om0000:]09P0;?0=40;F0=_0;<0=h0;70=m0;30>00;80><0;60>E`', 
#  '0:e0>70:c0>60:`0>20:H0>10:60=h09c0=M09]0=C09l0=V09l0=V09f0=T09d0>', 
#  '>709V0>P09A0>I08o0=V08d0=b08H0;108>0;70860:\\08<0;d0870<207n0<X07`', 
#  'f0<M07b0<L07P0<_07:0<>06Q0;Z05U0;m0550>Y05:0A305L0GG05I0HI05B0IGT', 
#  '0580Id0510JH0550Ja0550JX0520Jj0500K404m0K804j0K=04j0KC04j0KB04k0B', 
#  'KT04n0KT04k0KU04l0K]04l0K^04l0Ka04l0Kb04l0K^0520K_0510KX0510KR05D', 
#  '10KR0510KT0510KP0510KK04k0KD04j0K804k0K904o0Jk0550J`0550J80530J0\\',
#  '0510HN05@0HQ05@0F=05?0@g05B0>O04m09]04m08V05507l05O07i05c08706N0S', 
#  '8T07E08a07L09107S08j07b08^07i08c07o08[08A08]08C08Y08G08Y08N08V083',
#  'O08Y08J08J08O08G08T08L08V08I08]08K08\\08D08[08A08]08?08_08608^0895',
#  '08\\08708\\08708h08=08f08908h08;08`07k08^07m08`08208g08808i08008k0>', 
#  '7n09107m09107j09408009807o09607k09807c09807c09@08209808009:08409E', 
#  '@08P09C08^09C09B09U0:l09Z0<_09Z0<_09G09J09E08@09T071?om000?om000E',
#  '?om000?om000?om000?om0000:S06S0:507I0:807j0:008_09l08a0:108l09b0=',
#  '8[09V08X09P08R09S08Z09Y08c09S08^09R08o09K08d09D08U09J08O09@08A09c', 
#  'A08B09C08M09D08W09@08n09809309308l09=09009809:08l08c08e08Z094096A', 
#  '09208d08o08W08o08X08o08T08m08O08k08J08j08N08c08R08c08R08c08\\08h0I', 
#  '8j08m09808i08m08e08a08d08Z08^08W08X08N08P08F08B08;08D08>08E08>08M',
#  'J08W08N08[08J08Y08B08Q08:08I08<08J08808D08908C08;08G08>08M08;08GH', 
#  '08508908808D07n08307m07j07h07a07g07l07a07`07h07l07j08607i08207b0J', 
#  '7d07b07d07b08207d08107_07o07V07^07W07f07X07h07U07o07V08207W07n07@', 
#  'd07m07d07f07U07c07V07m07V07m07R07j07P07m07N07i07L07o07O08?07M08AE', 
#  '07P08M07N08P07V08m07b08b07a08X07J08107J08207B07X07F08I07909107?0a',
#  '9`07C09M07?07m07<07Y07Q08@07M08807D07o07C08707H08O07H08O07E08B07M',
#  'F08?07A07\\07F07=07J06c07>06507;05k07H06`06f07506k07=07V06k07T06NT',
#  '07d06Z?om000?om000?om000?om00009n05l0:>06U09j08H0:B0@>0:<0An0:?0m', 
#  'Cl0:80>I09k09M09d08;09i07j09d07U09f07J09h06X?om0000eT06a0eU06b0eO',
#  'H06b0;Q05k0<\\07Q0?:0>I0>n0=o0=m0:l0=N0:>0=@09K0=J09o0=@09K0<M087`',
#  '0;S0720;G0780:o06O?om000?om000?om000?om000?om000?om000?om000?om0^',
#  '00?om000?om000?om000?om000?om000?om000?om000?om000?om000?om000?ol',
#  'm000?om000?om000?om000n', '', '']
