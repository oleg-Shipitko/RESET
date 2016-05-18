import multiprocessing
import server
import sys
import matplotlib.pyplot as plt

def init_xy_plot():
	""" setup an XY plot canvas """
	plt.ion()
	figure = plt.figure(figsize=(6, 4),
						dpi=200,
						facecolor="w",
						edgecolor="k")
	ax = figure.add_subplot(111)
	lines, = ax.plot([],[],linestyle="none",
						marker=".",
						markersize=1,
						markerfacecolor="blue")
	ax.set_xlim(-200, 3200)
	ax.set_ylim(-200, 2200)
	ax.grid()
	return figure, lines

def update_xy_plot(x, y):
	""" re-draw the XY plot with new current_frame """
	#global lines
	lines.set_xdata(x)
	lines.set_ydata(y)
	figure.canvas.draw()


if __name__ == '__main__':

	data_queue = multiprocessing.Queue()
	server_proc = multiprocessing.Process(target=server.main, args=(data_queue,))
	server_proc.daemon = True
	server_proc.start()
	figure, lines = init_xy_plot()
	try:
		while True:
			plt.pause(0.001) 
			#print data_queue.get()
			#print '==============================================='
			particles = data_queue.get()
			#print particles
			weights = data_queue.get()
			wgh = [i*10000 for i in weights]
			px = [p[0] for p in particles]
			py = [p[1] for p in particles]
			update_xy_plot(px + px + wgh, py + wgh +py)
			#update_xy_plot(px + px + wgh+x_rob, py + wgh +py+y_rob)
	except Exception as err:
		print err
		sys.exit()

