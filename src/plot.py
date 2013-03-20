#!/usr/bin/env python

'''************************************************************************************************************************
This is a wxPanel to plot three lines over time
***************************************************************************************************************************
Project:	yudrone
Author:		Michael Dicke
Repository:	https://github.com/mdicke2s/yudrone
************************************************************************************************************************'''

import wx

# The recommended way to use wx with mpl is with the WXAgg
# backend. 
#
import matplotlib
#matplotlib.use('WXAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import \
    FigureCanvasWxAgg as FigCanvas, \
    NavigationToolbar2WxAgg as NavigationToolbar
import numpy as np
import pylab


class GraphPanel(wx.Panel):
    
    def handle_xyz(self, msg):
      self.xyz = msg
      #self.on_redraw_timer()
      
    def __init__(self, parent):        
	wx.Panel.__init__(self, parent , wx.NewId())
        
        #data
        self.pldatLineX = []
        self.pldatLineY = []
        self.pldatLineZ = []
        self.xyz = {'x':0, 'y':0, 'z':0}
        
        #plot
        self.init_plot()        
        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)        
        self.redraw_timer.Start(100)
        
        self.canvas = FigCanvas(self, -1, self.fig)


    def init_plot(self):
        self.fig = Figure((4.0, 2.0), dpi=100)

        self.axes = self.fig.add_subplot(111)
        self.axes.set_axis_bgcolor('black')
        #self.axes.set_title('Rotation', size=12)
        
        pylab.setp(self.axes.get_xticklabels(), fontsize=8)
        pylab.setp(self.axes.get_yticklabels(), fontsize=8)

        # plot the data as a line series, and save the reference 
        # to the plotted line series
        #
        self.plotLinex = self.axes.plot(
            self.pldatLineX, 
            linewidth=1,
            color=(1, 0, 0),
            )[0]
        self.plotLiney = self.axes.plot(
            self.pldatLineY, 
            linewidth=1,
            color=(0, 1, 0),
            )[0]
        self.plotLinez = self.axes.plot(
            self.pldatLineZ, 
            linewidth=1,
            color=(0, 0, 1),
            )[0]
        
        self.xmin = 0
        self.xmax = 0
        self.ymin = -10
        self.ymax = 10

    def draw_plot(self):
        
        self.xmax = len(self.pldatLineX) if len(self.pldatLineX) > 50 else 50
        self.xmin = self.xmax - 50
        
        self.axes.set_xbound(lower=self.xmin, upper=self.xmax)
        self.axes.set_ybound(lower=self.ymin, upper=self.ymax)
        
        self.axes.grid(True, color='gray')
        
        self.plotLinex.set_xdata(np.arange(len(self.pldatLineX)))
        self.plotLinex.set_ydata(np.array(self.pldatLineX))
        
        self.plotLiney.set_xdata(np.arange(len(self.pldatLineY)))
        self.plotLiney.set_ydata(np.array(self.pldatLineY))
        
        self.plotLinez.set_xdata(np.arange(len(self.pldatLineZ)))
        self.plotLinez.set_ydata(np.array(self.pldatLineZ))
        
        self.canvas.draw()
    
    def on_redraw_timer(self, Event=None):
	lineX = self.xyz['x']
	lineY = self.xyz['y']
	lineZ = self.xyz['z']
	
	if lineX < self.ymin:
	  self.ymin = lineX
	if lineX > self.ymax:
	  self.ymax = lineX
	if lineY < self.ymin:
	  self.ymin = lineY
	if lineY > self.ymax:
	  self.ymax = lineY
        if lineZ < self.ymin:
	  self.ymin = lineZ
	if lineZ > self.ymax:
	  self.ymax = lineZ
        
        self.pldatLineX.append(lineX)
        self.pldatLineY.append(lineY)
        self.pldatLineZ.append(lineZ)
        if len(self.pldatLineX) > 50:
	  self.pldatLineX.pop(0)
	if len(self.pldatLineY) > 50:
	  self.pldatLineY.pop(0)
	if len(self.pldatLineZ) > 50:
	  self.pldatLineZ.pop(0)
        self.draw_plot()