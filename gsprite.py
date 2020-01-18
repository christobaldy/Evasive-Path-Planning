"""
A module to support flipbook animation.

This module supports efficient 2d animation using sprite filmstrips.  A sprite is an 
image divided into rectangles of equal size.  The number of rectangles is specified by 
the rows and columns of the filmstrip.  Each rectangle is a frame.  You animate the image
by changing the current frame.

Author: Walker M. White (wmw2)
Date:   November 1, 2017 (Python 3 version)
"""
from kivy.graphics import *
from kivy.graphics.instructions import *
from .grectangle import GRectangle, GObject
from .app import GameApp

# #mark -
class GSprite(GRectangle):
    """
    An class representing a filmstrip for animating.
    
    The image is given by a JPEG, PNG, or GIF file whose name is stored in the attribute 
    `source`.  Image files should be stored in the **Images** directory so that Kivy can 
    find them without the complete path name.
    
    The image is broken up in to a sequence of frames.  These frames are arranged in a
    2d grid and are arranged left-to-right, top-to-bottom.  By specifying the frame,
    you can control what image is displayed inside of this rectangle.
    
    If the attributes ``width`` and ``height`` do not agree with the actual size of a
    single frame, the image is scaled to fit.Furthermore, if you define ``fillcolor``, 
    this object will tint your image by the given color.`
    
    If the image supports transparency, then this object can be used to represent irregular 
    shapes.  However, the :meth:`contains` method still treats this shape as a  rectangle.
    """
    
    # MUTABLE PROPERTIES
    @property
    def source(self):
        """
        The source file for this image.
        
        **invariant**. Value is a string refering to a valid file.
        """
        return self._source

    @source.setter
    def source(self,value):
        assert value is None or GameApp.is_image(value), '%s is not an image file' % repr(value)
        self._source = value
        if self._defined:
            self._reset()
    
    @property
    def count(self):
        """
        The number of frames in this filmstrip
        
        **invariant**. Value is an int > 0.
        """
        return self._format[0]*self._format[1]
    
    @property
    def frame(self):
        """
        The current animation frame of this filmstrip
        
        **invariant**. Value is an int 0..count-1.
        """
        return self._frame
    
    @frame.setter
    def frame(self,value):
        assert type(value) == int, '%s is not an int' % repr(value)
        assert value >= 0 and value < self.count, '%s is out of range' % repr(value)
        self._frame = value
        if self._bounds:
            self._texture = self._images[self._frame]
            self._bounds.texture = self._texture
    
    
    # BUILT-IN METHODS
    def __init__(self,**keywords):
        """
        Creates a new sprite
        
        To use the constructor for this class, you should provide it with a list of 
        keyword arguments that initialize various attributes. For example, to load the 
        filmstrip ``alien-strip1.png``, which has 3 rows and 2 columns, use the constructor::
            
            GSprite(x=0,y=0,width=10,height=10,source='alien-strip1.png',frames=(3,2))
        
        This class supports the all same keywords as :class:`GImage`; the only new 
        keyword is ``frames``. This keyword specifies the grid size of the animation
        frames in the image.  See the documentation of :class:`GImage` and 
        :class:`GObject` for the other supported keywords.
        
        :param keywords: dictionary of keyword arguments 
        :type keywords:  keys are attribute names
        """
        self._defined = False
        self.source  = keywords['source'] if 'source' in keywords else None
        self._setFormat(keywords['format'] if 'format' in keywords else (1,1))
        self._frame  = 0
        self._images = [None]*self.count
        self._bounds = None
        self._texture = None
        GRectangle.__init__(self,**keywords)
        self._defined = True
    
    # HIDDEN METHODS
    def _setFormat(self,value):
        """
        Sets the grid size of this filmstrip.
        
        Parameter value: The filmstrip grid size
        Precondition: value is a 2-element tuple of ints > 0
        """
        assert type(value) == tuple and len(value) == 2, '%s does is not a tuple pair' % repr(value)
        assert type(value[0]) == int and type(value[1]) == int, '%s does not have int values' % repr(value)
        assert value[0] > 0 and value[1] > 0, '%s does not have valid values' % repr(value)
        self._format = value
    
    def _reset(self):
        """
        Resets the drawing cache.
        """
        GObject._reset(self)
        x = -self.width/2.0
        y = -self.height/2.0
        
        texture = GameApp.load_texture(self.source)
        if texture:
            width  = texture.width/self._format[1]
            height = texture.height/self._format[0]
            
            ty = 0
            for row in range(self._format[0]):
                tx = 0
                for col in range(self._format[1]):
                    self._images[row*self._format[1]+col] = texture.get_region(int(tx),texture.height-int(ty)-int(height),int(width),int(height))
                    tx += width
                ty += width
        else:
            print('Failed to load',repr(self.source))
        
        self._texture = self._images[self._frame]
        self._bounds = Rectangle(pos=(x,y), size=(self.width, self.height),texture=self._texture)
        if not self._fillcolor is None:
            self._cache.add(self._fillcolor)
        else:
            self._cache.add(Color(1,1,1))
        self._cache.add(self._bounds)
        
        if not self._linecolor is None and self.linewidth > 0:
            line = Line(rectangle=(x,y,self.width,self.height),joint='miter',close=True,width=self.linewidth)
            self._cache.add(self._linecolor)
            self._cache.add(line)
        
        self._cache.add(PopMatrix())

