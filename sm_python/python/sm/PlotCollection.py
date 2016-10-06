import wxversion
wxversion.ensureMinimal('2.8')

import wx
import wx.aui
import matplotlib as mpl
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as Canvas
from matplotlib.backends.backend_wxagg import NavigationToolbar2Wx as Toolbar
import collections

class PlotCollection:
    def __init__(self, window_name="", window_size=(800,600)):
        """
        This class places matplot figures in tabs on a wx window
        (make sure to use unique figure ids between different PlotCollection instances 
         or wxwidget may segfault)
         e.g. usage:  
              from sm import PlotCollection
              import pylab as pl
        
              #create the plot as usual
              fig1=pl.figure()
              pl.plot([1,2],[2,3])
              fig2=pl.figure()
              pl.plot([3,1],[4,5])
          
              #add to collection
              plotter = PlotCollection.PlotCollection("My window name")
              plotter.add_figure("My plot1 name", fig1)
              plotter.add_figure("My plot2 name", fig2)
          
              #show collection
              plotter.show()
        """
        self.frame_name = window_name
        self.window_size = window_size
        self.figureList = collections.OrderedDict()
    
    def add_figure(self, tabname, fig):
        """
        Add a matplotlib figure to the collection
        """
        self.figureList[tabname] = fig
        
    def delete_figure(self, name):
        """
        Delete a figure from the collection given the tab name.
        """
        self.figureList.pop(name, None)
    
    def show(self):
        """
        Show the window on screen
        """
        if len(self.figureList.keys()) == 0:
            return
        app = wx.PySimpleApp()
        frame = wx.Frame(None,-1,self.frame_name, size=self.window_size)
        plotter = self.PlotNotebook(frame)
        for name in self.figureList.keys():
            plotter.add(name, self.figureList[name])
        frame.Show()
        app.MainLoop()
        
    class Plot(wx.Panel):
        def __init__(self, parent, fig, id = -1, dpi = None, **kwargs):
            wx.Panel.__init__(self, parent, id=id, **kwargs)
            fig.set_figheight(2)
            fig.set_figwidth(2)
            self.canvas = Canvas(self, -1, fig)
            self.toolbar = Toolbar(self.canvas)
            self.toolbar.Realize()
    
            sizer = wx.BoxSizer(wx.VERTICAL)
            sizer.Add(self.canvas,1,wx.EXPAND)
            sizer.Add(self.toolbar, 0 , wx.LEFT | wx.EXPAND)
            self.SetSizer(sizer)
    
    class PlotNotebook(wx.Panel):
        def __init__(self, parent, id = -1):
            wx.Panel.__init__(self, parent, id=id)
            self.nb = wx.aui.AuiNotebook(self)
            sizer = wx.BoxSizer()
            sizer.Add(self.nb, 1, wx.EXPAND)
            self.SetSizer(sizer)
    
        def add(self,name, fig):
           page = PlotCollection.Plot(self.nb, fig)
           self.nb.AddPage(page,name)
