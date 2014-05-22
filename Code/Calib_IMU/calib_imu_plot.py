from calib_imu import *
from PyQt4 import QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import sys, getopt

class Mavric_calib(QtGui.QMainWindow):

    def __init__(self):
        super(Mavric_calib, self).__init__()
        self.initUI()

    def initUI(self):
        # Status bar
        self.statusBar = self.statusBar()
        self.statusBar.showMessage('Ready')

        # Create widget for 3D view
        self.view = gl.GLViewWidget(self)

        # Create center widget
        self.center = QtGui.QFrame()
        self.center = self.view        
        self.setCentralWidget(self.center)

        
        xgrid = gl.GLGridItem()
        ygrid = gl.GLGridItem()
        zgrid = gl.GLGridItem()
        
        # ## rotate x and y grids to face the correct direction
        xgrid.rotate(90, 0, 1, 0)
        ygrid.rotate(90, 1, 0, 0)

        ## scale each grid differently
        xgrid.scale(0.1, 0.1, 0.1)
        ygrid.scale(0.1, 0.1, 0.1)
        zgrid.scale(0.1, 0.1, 0.1)

        ## move grid to first cadran
        zgrid.translate(1, 1, 0)
        ygrid.translate(1, 0, 1)
        xgrid.translate(0, 1, 1)

        self.view.addItem(xgrid)
        self.view.addItem(ygrid)
        self.view.addItem(zgrid)

        self.resize(800,800)
        self.setWindowTitle('Calib GUI')
        self.show()

    def centerWindow(self):
        screen = QtGui.QDesktopWidget().screenGeometry()
        size = self.geometry()
        self.move((screen.width()-size.width())/2, 
            (screen.height()-size.height())/2)

    def update_points(self, acc=np.zeros(3), mag=np.zeros(3)):

        acc_bias, acc_scale = np.zeros(3), np.zeros(3)
        mag_bias, mag_scale = np.zeros(3), np.zeros(3)    
            
        if acc.any():
            acc_bias, acc_scale = calibrate(acc[:,0], acc[:,1], acc[:,2])
        if mag.any():
            mag_bias, mag_scale = calibrate(mag[:,0], mag[:,1], mag[:,2])

        print('Acc bias: ')
        print(acc_bias)
        print('Acc scale: ')
        print(acc_scale)
        print('Mag bias: ')
        print(mag_bias)
        print('Mag scale: ')
        print(mag_scale)

        acc_corrected = (acc-acc_bias)/acc_scale
        mag_corrected = (mag-mag_bias)/mag_scale


        self.view.opts['distance'] = 3
        p = np.random.random((1000, 3))

        self.acc_points = gl.GLScatterPlotItem(pos=acc_corrected, size=0.01, color=(0, 0, 1, 1), pxMode=False)
        # self.acc_points = gl.GLScatterPlotItem(pos=acc, size=10, color=(1, 0, 1, 1), pxMode=False)
        self.view.addItem(self.acc_points)

        self.mag_points = gl.GLScatterPlotItem(pos=mag_corrected, size=0.01, color=(0, 1, 0, 1), pxMode=False)
        # self.mag_points = gl.GLScatterPlotItem(pos=mag, size=5, color=(0, 1, 0, 1), pxMode=False)
        self.view.addItem(self.mag_points)

        self.sphere = gl.GLMeshItem(meshdata=gl.MeshData.sphere(50, 50), 
                                    color=(0.1, 0.1, 0.1, 0.01), 
                                    smooth=True,
                                    drawEdges=True,
                                    drawFaces=False)
        # self.view.addItem(self.sphere)


def main(argv):
    usage = """usage:
            calib_imu.py -f <inputfile>"""

    filename = ''

    try:
        opts, args = getopt.getopt(argv,"hf:")
    except getopt.GetoptError:
        print(usage)
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print(usage)
            sys.exit(2)
        elif opt == '-f':
            filename = arg

    if not filename:
        print(usage)
        sys.exit(2)

        
    print("Analysing '" + filename + "'")

    acc, mag = read_logfile(filename)

    ## Always start by initializing Qt (only once per application)
    app = QtGui.QApplication([])

    w = Mavric_calib()
    w.update_points(acc, mag)

    ## Start the Qt event loop
    app.exec_()

if __name__ == '__main__':
    main(sys.argv[1:])
