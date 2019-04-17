#import Morse
#from pymorse import Morse


import ccm
#from ccm.pattern import Pattern

import re
#from ccm.pattern import *

#vision_cam = Morse().robot.GeometricCamerav1
import numpy
import math


#vision_cam = ccm.middle.robot.GeometricCamerav1
#from test2 import simu

from ccm.morserobots import middleware





class BlenderVision(ccm.Model):
    def __init__(self,buffer1,buffer2,delay=0.0,log=None,delay_sd=None):
        
        #self._vision_cam = Morse().robot.GeometricCamerav1
        self._b1=buffer1
        self._b2=buffer2
        self.delay=delay
        self.delay_sd=delay_sd
        self.error=False
        self.busy=False
        #self.blender_camera = Morse().robot.GeometricCamerav1
    
    def getScreenVector(self,x,y):
        x = middleware.request('getScreenVector',[x,y])
        print(x)
        #print(math.sqrt(x[0]**2 + x[1]**2 + x[2]**2))
        #x = numpy.array(middleware.request('getScreenVector',[x,y]))
        #print(numpy.linalg.norm(x))
    
    def cScan(self,openingDepth):
        x  = middleware.request('cScan', [openingDepth])
        print(x)
    
    def scan(self,delay=0.00):
        #print(ccm.middle)
        #import time
        #now = time.time()
        self._objects = middleware.request('scan_image',[])
        #print("Time:")
        #print(time.time() - now)
        print(self._objects, "objects")
        #yield 1.3

    def refresh(self):
        print ("refresh")
        x = vision_cam.get_visible_objects().result()
        print("refresh2")
        #x = x.result()
        if not x:
            print("No x. ERROR")
            self.error=True
            self.busy=False
            return
        for i in range(len(x)):
            obj_label = x[i]
            if not self._b1.chunk:
                self._b1.set('obj' + repr(i) + ':' + obj_label)
            else:
                #print(dir(self._b1.chunk))
                ck = repr(self._b1.chunk)
                self._b1.set(ck + ' obj' + repr(i) + ':' + obj_label)
        
        #with Morse() as morse:
        #    x = morse.robot.GeometricCamerav1.get_visible_objects()
        #x = vision_cam.get_visible_objects()
        #x = x['visible_objects']
        #for y in x:
        #    print(x, "visible objects")
    
    def check_visibility(self, label):
        return vision_cam.check_visibility(label)
    
    def parse(self,sv_pair):
        '''Given a slot:value pair, returns left and right'''
        return sv_pair.split(':')

    def get_visible_angles(self, label):
        print(vision_cam.get_visible_angles(label))
   
    def request(self,pattern=''):
        if self.busy: return

        self.error=False
        
        pattern_list = pattern.split()
        for p in pattern_list:
            (slot,value) = self.parse(p)
            if 'obj' in slot:
                print('obj')
            
        
#  def request(self,pattern=''):
#    if self.busy: return
#
#    matcher=Pattern(pattern)
#      
#    self.error=False
#    r=[]
#    for obj in self.parent.parent.get_children():
#      if matcher.match(obj)!=None:
#        if not hasattr(obj,'salience') and not hasattr(obj,'visible'):
#          continue
#        
#        if hasattr(obj,'salience'):
#          if self.random.random()>obj.salience:
#            continue
#        if hasattr(obj,'visible'):
#          if obj.visible==False:
#            continue
#        if hasattr(obj,'value'):
#          if obj.value==None:
#            continue
#        r.append(obj)
#
#    self.busy=True
#    d=self.delay
#    if self.delay_sd is not None:
#        d=max(0,self.random.gauss(d,self.delay_sd))
#    yield d
#    self.busy=False
#
#    if len(r)==0:
#      self._buffer.clear()
#      self.error=True
#    else:
#      obj=self.random.choice(r)
#      if obj not in self.parent.parent.get_children():
#        self._buffer.clear()
#        self.error=True
#      elif hasattr(obj,'visible') and obj.visible==False:
#        self._buffer.clear()
#        self.error=True
#      else:
#        self._buffer.set(obj)
      
      
      
        
