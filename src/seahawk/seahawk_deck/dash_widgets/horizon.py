import sys, math
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtCore import pyqtSlot, pyqtSignal, Qt, QPointF,QRectF, QPoint
from PyQt5.QtGui import QColor, QBrush, QPen, QFont, QPolygon, QGuiApplication
#from std_msgs.msg import Float32
#from .map_subscribers import *


import rclpy
from rclpy.node import Node
#creates subscriber, assuming ROV is publisher

from std_msgs.msg import String

import threading
import time
#added in order to use threading with the subscriber (runs as infinite loop)

#SUBSCRIBER
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

#HORIZON CODE
class ArtificialHorizon(QtWidgets.QWidget):
    def __init__(self):
        super(ArtificialHorizon, self).__init__()
        self.initUI()

    def initUI(self):
        self.height = 600
        self.width = 600

        self.roll = 0     # degrees
        self.pitch = 0    # degrees
        self.speed = 0    # KIAS
        self.altitude = 0 # ft MSL
        self.heading = 0  # degrees
        self.numSat = 0   # Number of Satellites (GPS)

        self.pitchInterval = 0.013 # % of height used to display 1 degree

        self.setGeometry(300, 300, self.width, self.height)
        self.setWindowTitle('Artificial Horizon')
        self.show()

    def resizeEvent(self, newSize):
        self.width = newSize.size().width()
        self.height = newSize.size().height()

    def paintEvent(self, event):
        painter = QtGui.QPainter()
        painter.begin(self)
        self.drawArtificialHorizon(event, painter)
        painter.end()

    def drawArtificialHorizon(self, event, painter):

        ## FIXME: Need to adapt this for our subscribers.
        #vars should be updated with readings from the IMU
        #what do we write to access the output (it has a compass on it so we would probably be reading from that for each var) 
        self.roll = 15
        self.pitch = 15
        self.speed = 100
        self.altitude = 0
        self.heading = 0
        self.numSat = 4

        self.drawSky(event, painter)

        painter.translate(self.width/2, self.height/2)
        painter.rotate(-self.roll)
        painter.translate(-self.width/2, -self.height/2)
        painter.translate(0,self.height*(self.pitch*self.pitchInterval))

        self.drawGround(event, painter)
        self.drawPitchIndicator(event, painter)

        painter.translate(0,self.height*(-1*self.pitch*self.pitchInterval))

        self.drawTurnIndicator(event, painter)

        painter.translate(self.width/2, self.height/2)
        painter.rotate(self.roll)
        painter.translate(-self.width/2, -self.height/2)

        self.drawAircraftSymbol(event, painter)
        self.drawAirspeedIndicator(event, painter)
        self.drawAltitudeIndicator(event, painter)
        self.drawHeadingIndicator(event, painter)
        self.drawNumSatellites(event, painter)
        #self.drawWaypointAccuracy(event, painter)

    def drawNumSatellites(self, event, painter):
        p1 = QPoint(0,0)
        p2 = QPoint(int(self.width*(0.25)),int(self.height*0.1))
        rect = QRectF(p1,p2)
        if self.numSat < 4:
            painter.setPen(QPen(QBrush(Qt.red), 2, Qt.SolidLine))
        else:
            painter.setPen(QPen(QBrush(Qt.green), 2, Qt.SolidLine))
        painter.drawText(rect,QtCore.Qt.AlignCenter,"GPS: " + str(self.numSat) + " satellites")

    #def drawWaypointAccuracy(self, event, painter):
    #    p1 = QPoint(self.width*(0.65),0)
    #    p2 = QPoint(self.width,self.height*0.1)
    #    rect = QRectF(p1,p2)
    #    painter.drawText(rect,QtCore.Qt.AlignCenter,"Wp Accuracy: " + "%.2f" % self.latestWpAccuracy + " m")

    def drawSky(self, event, painter):
        brush = QtGui.QBrush(QtGui.QColor(38, 89, 242), QtCore.Qt.SolidPattern)
        painter.fillRect(QRectF(0,0,self.width, self.height), brush)

    def drawGround(self, event, painter):
        brush = QtGui.QBrush(QtGui.QColor(84, 54, 10), QtCore.Qt.SolidPattern)
        painter.fillRect(QRectF(-300,self.height/2,self.width+600, self.height*(0.5+self.pitchInterval*180)), brush)
        painter.setPen(QPen(QBrush(Qt.white), 2, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(-300,int(self.height/2),self.width+600,int(self.height/2))


    def drawHeadingIndicator(self, event, painter):
        boxWidth = self.width*1.0
        boxHeight = self.height*0.1
        brush = QtGui.QBrush(QColor(100,100,100,200))
        painter.setPen(QPen(QBrush(Qt.yellow), 2, Qt.SolidLine))
        painter.fillRect(QRectF((self.width-boxWidth)/2,self.height-boxHeight,boxWidth,boxHeight),brush)

        directions = {0:"N",45:"NE",90:"E",135:"SE",180:"S",215:"SW",270:"W",315:"NW"}
        scale = 0.01
        for i in range(self.heading-49,self.heading+49):
            if i % 10 == 0:
                x = self.width*0.5-((self.heading-i)*scale*self.width)
                y = self.height-boxHeight
                if i < 0:
                    i += 360
                i = i % 360
                text = str(i)
                if i in directions:
                    text = directions[i]
                painter.drawLine(int(x),int(y),int(x),int(y+5))
                painter.drawText(QPoint(int(x+7-8*len(text)),int(y+22)),text)

        painter.setBrush(Qt.black)
        p1 = QPoint(int(self.width*(0.46)),int(self.height))
        p2 = QPoint(int(self.width*(0.46)),int(self.height - boxHeight*0.9))
        p3 = QPoint(int(self.width*(0.50)),int(self.height - boxHeight))
        p4 = QPoint(int(self.width*(0.54)),int(self.height - boxHeight*0.9))
        p5 = QPoint(int(self.width*(0.54)),int(self.height))
        poly = QPolygon([p1,p2,p3,p4,p5])
        painter.setPen(QPen(QBrush(QColor(0,0,0,0)), 2, Qt.SolidLine, Qt.RoundCap))
        painter.drawPolygon(poly)
        painter.setPen(QPen(QBrush(QColor(255,255,0)), 2, Qt.SolidLine, Qt.RoundCap))
        rect = QRectF(p1,p4)
        painter.drawText(rect,QtCore.Qt.AlignCenter,str(self.heading) + u'\N{DEGREE SIGN}')

    def drawAirspeedIndicator(self, event, painter):
        boxWidth = self.width*0.13
        boxHeight = self.height*0.6
        brush = QtGui.QBrush(QColor(100,100,100,200))
        painter.setPen(QPen(QBrush(Qt.yellow), 2, Qt.SolidLine))
        painter.fillRect(QRectF(0,(self.height-boxHeight)/2,boxWidth,boxHeight),brush)

        scale = 0.01
        for i in range(self.speed-29,self.speed+29):
            if i % 10 == 0 and i >= 0:
                x = boxWidth
                y = self.height*0.5+((self.speed-i)*scale*self.height)
                text = str(i)
                painter.drawLine(int(x-5),int(y),int(x),int(y))
                painter.drawText(QPoint(int(x-10-8*len(text)),int(y+5)),text)

        painter.setBrush(Qt.black)
        p1 = QPoint(0,int(self.height*(0.46)))
        p2 = QPoint(int(boxWidth*0.9),int(self.height*(0.46)))
        p3 = QPoint(int(boxWidth),int(self.height*(0.5)))
        p4 = QPoint(int(boxWidth*0.9),int(self.height*(0.54)))
        p5 = QPoint(0,int(self.height*(0.54)))
        poly = QPolygon([p1,p2,p3,p4,p5])
        painter.setPen(QPen(QBrush(QColor(0,0,0,0)), 2, Qt.SolidLine, Qt.RoundCap))
        painter.drawPolygon(poly)
        painter.setPen(QPen(QBrush(QColor(255,255,0)), 2, Qt.SolidLine, Qt.RoundCap))
        rect = QRectF(p1,p4)
        painter.drawText(rect,QtCore.Qt.AlignCenter,str(self.speed) + " kt")
        painter.drawText(QPoint(5,int((self.height-boxHeight)/2-5)),"Airspeed (KIAS)")

    def drawAltitudeIndicator(self, event, painter):
        boxWidth = self.width*0.13
        boxHeight = self.height*0.6
        brush = QtGui.QBrush(QColor(100,100,100,200))
        painter.setPen(QPen(QBrush(Qt.yellow), 2, Qt.SolidLine))
        painter.fillRect(QRectF(self.width-boxWidth,(self.height-boxHeight)/2,boxWidth,boxHeight),brush)

        scale = 0.01
        for i in range(self.altitude-29,self.altitude+29):
            if i % 10 == 0:
                x = self.width - boxWidth
                y = self.height*0.5+((self.altitude-i)*scale*self.height)
                text = str(i)
                painter.drawLine(int(x),int(y),int(x+5),int(y))
                painter.drawText(QPoint(int(x+10),int(y+5)),text)

        painter.setBrush(Qt.black)
        p1 = QPoint(int(self.width),int(self.height*(0.46)))
        p2 = QPoint(int(self.width-boxWidth*0.9),int(self.height*(0.46)))
        p3 = QPoint(int(self.width-boxWidth),int(self.height*(0.5)))
        p4 = QPoint(int(self.width-boxWidth*0.9),int(self.height*(0.54)))
        p5 = QPoint(int(self.width),int(self.height*(0.54)))
        poly = QPolygon([p1,p2,p3,p4,p5])
        painter.setPen(QPen(QBrush(QColor(0,0,0,0)), 2, Qt.SolidLine))
        painter.drawPolygon(poly)
        painter.setPen(QPen(QBrush(QColor(255,255,0)), 2, Qt.SolidLine))
        text = str(self.altitude) + " m"
        rect = QRectF(p1,p4)
        painter.drawText(rect,QtCore.Qt.AlignCenter,text)
        painter.drawText(QPoint(int(self.width-boxWidth+5),int((self.height-boxHeight)/2-5)),"Altitude")

    def drawTurnIndicator(self, event, painter):
        painter.setBrush(Qt.white)
        painter.setPen(QPen(QBrush(Qt.white), 2, Qt.SolidLine, Qt.RoundCap))
        radius = self.width*(0.3)
        yOffset = self.height*(0.10)
        painter.drawArc(
            QRectF(self.width*(0.5)-radius,yOffset, 2*radius, 2*radius),
            16*30,16*120)

        height = self.height*0.02
        x = self.width/2
        y = yOffset
        x2 = x
        y2 = y-height
        x3 = x
        y3 = y2-5
        xCenter = self.width/2
        yCenter = radius + yOffset

        angles = [-60,-45,-30,-20,-10,0,10,20,30,45,60]
        for angle in angles:
            painter.translate(xCenter,yCenter)
            painter.rotate(angle)
            painter.translate(-xCenter,-yCenter)

            painter.drawLine(int(x),int(y),int(x2),int(y2))
            text = str(angle)
            painter.drawText(QPoint(int(x3-4*len(text)),int(y3)),text)

            painter.translate(xCenter,yCenter)
            painter.rotate(-angle)
            painter.translate(-xCenter,-yCenter)

        # Draw the arrow
        height = self.height*0.025
        poly = QPolygon([
            QPoint(int(x),int(y)),
            QPoint(int(x-height/2),int(y+height)),
            QPoint(int(x+height/2),int(y+height)),])

        painter.translate(xCenter,yCenter)
        painter.rotate(self.roll)
        painter.translate(-xCenter,-yCenter)

        painter.drawPolygon(poly)

        painter.translate(xCenter,yCenter)
        painter.rotate(-self.roll)
        painter.translate(-xCenter,-yCenter)


    def drawAircraftSymbol(self, event, painter):
        brightYellow = QtGui.QColor(255, 255, 0)
        widthFraction = 0.10
        heightFraction = 0.05
        painter.setPen(QPen(QBrush(brightYellow), 5, Qt.SolidLine, Qt.RoundCap))
        painter.setBrush(brightYellow)
        poly = QPolygon([
            QPoint(int(self.width*0.5), int(self.height*0.5)),
            QPoint(int(self.width*(0.5+widthFraction/2.0)),int(self.height*(0.5+heightFraction))),
            QPoint(int(self.width*0.5), int(self.height*(0.5+heightFraction/2.0))),
            QPoint(int(self.width*(0.5-widthFraction/2.0)),int(self.height*(0.5+heightFraction)))
            ])
        painter.drawPolygon(poly)
        space = 0.25
        length = 0.1
        painter.drawLine(int(self.width*space), int(self.height/2), int(self.width*(space+length)), int(self.height/2))
        painter.drawLine(int(self.width*(1-space-length)), int(self.height/2), int(self.width*(1-space)), int(self.height/2))


    def drawPitchIndicator(self, event, painter):
        minHeight = 0.15 - self.pitch*self.pitchInterval
        maxHeight = 0.85 - self.pitch*self.pitchInterval
        for i in range(-9,9):
            text = str(10*abs(i))
            height = 0.5-self.pitchInterval*10*i
            if height > minHeight and height < maxHeight:
                painter.drawLine(
                    int(self.width*(0.4)),int(self.height*(height)),
                    int(self.width*(0.6)),int(self.height*(height)))
                painter.drawText(QPoint(int(self.width*(0.6)+5),int(self.height*(height)+5)),text)
                painter.drawText(QPoint(int(self.width*(0.4)-22),int(self.height*(height)+5)),text)

            height = height - self.pitchInterval*5
            if height > minHeight and height < maxHeight:
                painter.drawLine(
                    int(self.width*(0.45)),int(self.height*(height)),
                    int(self.width*(0.55)),int(self.height*(height)))
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    #runs in thread
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    app = QtWidgets.QApplication(sys.argv)
    ah = ArtificialHorizon()
    #threading on first infinite loop: allows it to run separately
    t = threading.Thread(target = rclpy.spin,args=(minimal_subscriber,))
    t.start()
    #"func2" (other infinite loop)
    guiValue = app.exec_()
    sys.exit(guiValue)


if __name__ == '__main__':
    main()
