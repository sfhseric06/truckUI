import pygame
import math
import os, time
from gps import *
import matplotlib
matplotlib.use("Agg")
import matplotlib.backends.backend_agg as agg
import pylab
import matplotlib.pyplot as plt


## slow style ##
fig = pylab.figure(figsize=[6.4,4.8],dpi=50)
plt.hold(False)
ax = fig.gca()
ax.plot([1,2,4], 'ro')
canvas = agg.FigureCanvasAgg(fig)
canvas.draw()
background = fig.canvas.copy_from_bbox(ax.bbox)
renderer = canvas.get_renderer()
raw_data = renderer.tostring_rgb()


'''
#fig,ax = plt.subplots()
fig = plt.figure(1, (2,2), dpi=72)
ax = plt.axes()
#ax = fig.gca()
manager = plt.get_current_fig_manager()
#manager.window.wm_geometry("+100+100")
ax.plot([1,2,4], 'ro')
canvas = agg.FigureCanvasAgg(fig)
canvas.draw()
renderer = canvas.get_renderer()
background = canvas.copy_from_bbox(ax.bbox)
#fig.show()
#fig.canvas.draw()
'''

session = gps("localhost", "2947")
session.stream(WATCH_ENABLE | WATCH_NEWSTYLE)

pygame.init()
screen = pygame.display.set_mode((640,480))

running = 1

line_height = 22
myfont = pygame.font.SysFont("monospace",15)
line_margin_l = 20
line_margin_t = 20
alt_list = []
fixed_length = 220

graph_w = 220
graph_h = 120
graph_pos_x = 400
graph_pos_y = 20
heading_xy = (320,75)
heading_rad = 70
graph_line_color = (255,0,0)
heading_fill = (30,30,30)
graph_axes_color = (30,30,30)

time.clock()

class KalmanFilter(object):

    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def input_latest_noisy_measurement(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.estimated_measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

        return self.posteri_estimate

    def get_latest_estimated_measurement(self):
        return self.posteri_estimate


class LimitedList(list):
    def __init__(self, fixed_length):
        super(LimitedList,self).__init__()
        self.fixed_length = fixed_length
    def append(*args):
        super(LimitedList,self).append(*args)


def draw_heading(deg):
    pygame.draw.circle(screen, heading_fill, heading_xy, heading_rad+4, 0)
    pygame.draw.circle(screen, (0,0,0), heading_xy, heading_rad+2, 0)
    pygame.draw.circle(screen, heading_fill, heading_xy, heading_rad, 0)
    heading_vector = (math.cos(math.radians(deg))*heading_rad+heading_xy[0], math.sin(math.radians(deg))*heading_rad+heading_xy[1])
    pygame.draw.line(screen,graph_line_color,heading_xy,heading_vector,2)

def draw_graph_lines():
    x1 = graph_pos_x
    x2 = graph_pos_x + graph_w
    y1 = graph_pos_y + graph_h
    y2 = graph_pos_y

    pygame.draw.line(screen,graph_axes_color,(x1,y1),(x2,y1),2)
    pygame.draw.line(screen,graph_axes_color,(x1,y1),(x1,y2),2)
    
def draw_list_to_graph(in_list):
    ratio = fixed_length/graph_w
    iratio = graph_w/fixed_length

    if in_list:
        max_point = 0.1*(max(in_list) - min(in_list)) + max(in_list)
        #max_point = max(in_list)
        min_point = -0.1*(max(in_list) - min(in_list)) + min(in_list)
        #min_point = min(in_list)
        if max_point == min_point:
            max_point = 1.1*max(in_list) 
            min_point = 0.9*min(in_list)

    pos = last_pos = None
    for i in range(graph_w):
        start_point = int(round(-ratio*i-1))
        end_point = int(round(start_point+ratio))
        if not end_point:
            sub_list = in_list[start_point:]
        else:
            sub_list = in_list[start_point:end_point]
        if sub_list:
            y = graph_h*((sum(sub_list)/len(sub_list))-min_point)/(max_point-min_point)
            y_pos = graph_pos_y + graph_h - y - min_point
            x_pos = graph_pos_x + graph_w - i
            if pos:
                last_pos = pos
            pos = (x_pos,y_pos)
            if pos!=None and last_pos!=None:
	        pygame.draw.line(screen,graph_line_color,last_pos,pos,3)

def texts(data, line_num):
    this_data = myfont.render(str(data), 1, (255,255,255))
    screen.blit(this_data, (line_margin_l,int(line_num)*line_height + line_margin_t))

def not_nan_none(val):
    if val!=None:
        if not math.isnan(val):
            return True
    return False

latitude = None
longitude = None
heading = None
heading_fix = None
process_variance = 1e-3
estimated_meas_variance = 1e-2 
while running:
    os.system('clear')
    session.next()

    print
    print ' GPS reading'
    print '----------------------------------------'
    print 'latitude ' , session.fix.latitude
    print 'longitude ' , session.fix.longitude
    print 'time utc ' , session.utc, session.fix.time
    print 'altitude ' , session.fix.altitude
    print 'epx ' , session.fix.epx
    print 'epv ' , session.fix.epv
    print 'ept ' , session.fix.ept
    print 'speed ' , session.fix.speed
    print 'climb ' , session.fix.climb

    last_lat = latitude
    last_long = longitude
    latitude = session.fix.latitude
    longitude = session.fix.longitude
    

    alt_filter = KalmanFilter(process_variance, estimated_meas_variance)
    altitude = alt_filter.input_latest_noisy_measurement(session.fix.altitude)
    #altitude = session.fix.altitude

    if not math.isnan(altitude):
        alt_list.append(altitude)
        alt_list = alt_list[-fixed_length:]
    if alt_list:
	print "1 :" + str(round(time.clock(),2))
	ax.plot(alt_list, 'ro')
	canvas = agg.FigureCanvasAgg(fig)
	canvas.draw()
	background = fig.canvas.copy_from_bbox(ax.bbox)
	renderer = canvas.get_renderer()
	raw_data = renderer.tostring_rgb()
	print "2 :" + str(round(time.clock(),2))
	'''
	alt_line = ax.plot(alt_list, 'ro', animated=True)[0]
        canvas.restore_region(background)
        alt_line.set_xdata(range(len(alt_list)))
        alt_line.set_ydata(alt_list)
        ax.draw_artist(alt_line)
        canvas.blit(ax.bbox)
	raw_data = renderer.tostring_rgb()
    	size = canvas.get_width_height()
   	surf = pygame.image.fromstring(raw_data,size, "RGB")
    	screen.blit(surf, (0,0))
	'''

    

    event = pygame.event.poll()
    if event.type == pygame.QUIT:
        running = 0

    screen.fill((0, 0, 0))
    draw_graph_lines()
    draw_list_to_graph(alt_list)
    pygame.draw.line(screen, (0, 0, 255), (0, 0), (639, 479))
    pygame.draw.aaline(screen, (0, 0, 255), (639, 0), (0, 479))

    if not_nan_none(last_lat) and not_nan_none(last_long) and not_nan_none(latitude) and not_nan_none(longitude):
            dLon = longitude - last_long
            y = math.sin(math.radians(dLon)) * math.cos(math.radians(latitude))
            x = math.cos(math.radians(last_lat)) * math.sin(math.radians(latitude)) - (math.sin(math.radians(last_lat))*math.cos(math.radians(latitude))*math.cos(math.radians(math.radians(dLon))))
            heading = math.degrees(math.atan2(y,x))
            heading_filter = KalmanFilter(process_variance, estimated_meas_variance)
            heading_fix = heading_filter.input_latest_noisy_measurement(heading)
            draw_heading(heading_fix)

    texts("HERP A DERP",0)
    texts(session.fix.latitude,1)
    texts(session.fix.longitude,2)
    texts(altitude,3)
    if heading_fix!=None:
        texts(heading_fix,4)

    size = canvas.get_width_height()
    surf = pygame.image.fromstring(raw_data,size, "RGB")
    screen.blit(surf, (0,0))

    pygame.display.flip()
