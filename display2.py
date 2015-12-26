from PyQt4.QtGui import *
from PyQt4.QtCore import *
import pyqtgraph as pg
import math
import os, time
from gps import *
import numpy as np

xcoords = [1,2,3]
ycoords = [10,-2,4]
fixed_length = 200

session = gps("localhost", "2947")
session.stream(WATCH_ENABLE | WATCH_NEWSTYLE)

running = 1

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



def texts(data, line_num):
    this_data = myfont.render(str(data), 1, (255,255,255))
    screen.blit(this_data, (line_margin_l,int(line_num)*line_height + line_margin_t))

def not_nan_none(val):
    if val!=None:
        if not math.isnan(val):
            return True
    return False

class Window(QWidget):
    def __init__(self,parent=None):
        QWidget.__init__(self,parent)
        self.thread = UpdateThread()

        #self.connect(self.thread, SIGNAL("finished()"), self.updateUi)
        #self.connect(self.thread, SIGNAL("terminated()"), self.updateUi)
        self.connect(self.thread, SIGNAL("plotNow"), self.updatePlot)
        self.connect(self.thread, SIGNAL("plotPolar"), self.updatePolar)


        layout = QGridLayout()
        layout.setSpacing(0)
        layout.setMargin(0)
        text = QLineEdit('enter text')
        self.plot = pg.PlotWidget()
        self.plot2 = pg.PlotWidget()
        self.plot_polar = pg.PlotWidget()
        self.plot_polar.setAspectLocked()
        self.curve = self.plot.plot(pen="r")
        self.curve_polar = self.plot_polar.plot(pen="r")

        self.plot.resize(310,120)
        self.plot_polar.resize(200,200)


        layout.addWidget(self.plot_polar,0,0,1,1)
        layout.addWidget(text,1,0)
        layout.addWidget(self.plot,0,1,1,1)
        layout.addWidget(self.plot2,1,1,1,1)
        self.setLayout(layout)
        self.thread.start()

        r=1
        circle = pg.QtGui.QGraphicsEllipseItem(-r,-r,r*2,r*2)
        circle.setPen(pg.mkPen(0.2))
        self.plot_polar.addItem(circle)

    def updatePlot(self,this_data):
        self.curve.setData(y=this_data)

    def updatePolar(self,angle_deg):
        angle_rad = np.deg2rad(angle_deg)
        steps = np.linspace(0,1,10)
        x_pts = steps*np.cos(angle_rad) 
        y_pts = steps*np.sin(angle_rad) 
        self.curve_polar.setData(x=x_pts,y=y_pts)




class UpdateThread(QThread):
    def __init__(self,parent=None):
        QThread.__init__(self,parent)
        self.exiting = False
        self.size = QSize(0,0)
    def __del__(self):
        self.exiting=True
        self.wait()

    def run(self):
        latitude = None
        longitude = None
        heading = None
        heading_fix = None
        process_variance = 1e-3
        estimated_meas_variance = 1e-2 
        alt_list = np.array([])

        while not self.exiting:
            self.sleep(1)

            #os.system('clear')
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
                alt_list = np.append(alt_list,[altitude])
                alt_list = alt_list[-fixed_length:]
            if alt_list.any():
                #print altitude
                self.emit(SIGNAL('plotNow'),alt_list)

            if not_nan_none(last_lat) and not_nan_none(last_long) and not_nan_none(latitude) and not_nan_none(longitude):
                    dLon = longitude - last_long
                    y = math.sin(math.radians(dLon)) * math.cos(math.radians(latitude))
                    x = math.cos(math.radians(last_lat)) * math.sin(math.radians(latitude)) - (math.sin(math.radians(last_lat))*math.cos(math.radians(latitude))*math.cos(math.radians(math.radians(dLon))))
                    heading = math.degrees(math.atan2(y,x))
                    heading_filter = KalmanFilter(process_variance, estimated_meas_variance)
                    heading_fix = heading_filter.input_latest_noisy_measurement(heading)
                    self.emit(SIGNAL('plotPolar'),heading_fix)





app = QApplication([])
window = Window()
window.resize(620,480)
window.show()
sys.exit(app.exec_()) 






        #draw_heading(heading_fix)

