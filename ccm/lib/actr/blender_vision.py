#import Morse
#from pymorse import Morse

import matplotlib.pyplot as plt
import numpy as np

import ccm
from ccm.pattern import Pattern

import re
#from ccm.pattern import *

#vision_cam = Morse().robot.GeometricCamerav1
import numpy
import math

from decimal import *

#vision_cam = ccm.middle.robot.GeometricCamerav1
#from test2 import simu

from ccm.morserobots import middleware

#Decimal Precision
getcontext().prec = 5

def rolling_window(a,window):
    shape = a.shape[:-1] + (a.shape[-1] - window + 1, window)
    strides = a.strides + (a.strides[-1],)

    return numpy.lib.stride_tricks.as_strided(a, shape=shape, strides=strides)


class InternalEnvironment(ccm.Model):
    dial=ccm.Model(isa='dial',value=-1000)
    camp=ccm.Model(isa='dial',value=2000)


class BlenderVision(ccm.Model):
    def __init__(self,buffer1,buffer2,delay=0.0,log=None,delay_sd=None):
        #from ccm.morserobots import middleware
        #self._vision_cam = Morse().robot.GeometricCamerav1
        self._b1=buffer1
        self._b2=buffer2
        self.delay=delay
        self.delay_sd=delay_sd
        self.error=False
        self.busy=False
        self._objects = {}
        self._openings = {}
        self._oldopenings = {}
        self._edges = {}
        self._internalChunks = []
        self._screenLeft = numpy.arange(Decimal(0.60),Decimal(1.0),Decimal(0.002))
        self._screenCenter = numpy.arange(Decimal(0.30),Decimal(0.60),Decimal(0.002))
        self._screenRight = numpy.arange(Decimal(0.0),Decimal(0.30),Decimal(0.002))
        self._internalChunks.append(ccm.Model(isa='dial'))

        #self._internalEnvironment = InternalEnvironment(self._b1)
        #self._internalEnvironment.doConvert(parent=self)
        #self.blender_camera = Morse().robot.GeometricCamerav1
    
    def getScreenVector(self,x,y):
        x = middleware.request('getScreenVector',[x,y])
        print(x)
        #print(math.sqrt(x[0]**2 + x[1]**2 + x[2]**2))
        #x = numpy.array(middleware.request('getScreenVector',[x,y]))
        #print(numpy.linalg.norm(x))
    
    def cScan(self,openingDepth='0.3'):
        x  = middleware.request('cScan', [openingDepth])
        print(x)

    def xScan(self,openingDepth,y):
        x = middleware.request('xScan', [openingDepth,y])
        print(x)
    
    def almost_equal(self,x1,x2):
        return abs(x1-x2) < 0.01 #1cm?

    def shares_edge(self,list1,list2):
        print(list1, list2)
        if self.almost_equal(list1[0],list2[0]):
            return 1
        if self.almost_equal(list1[0],list2[1]):
            return 1
        if self.almost_equal(list1[1],list2[0]):
            return 1
        if self.almost_equal(list1[1],list2[1]):
            return 1
        return 0


    def find_edges(self):
        for y in sorted(self._objects.keys()):
            if len(list(self._objects[y].keys())) > 1:#there needs to be more than one object to be a depth gap
                edges = []
                edgeCount = 0
                while True:
                    #print(edgeCount,'EDGECOUNT')
                    try:
                        firstLabel,firstvalue = list(self._objects[y].items())[edgeCount]
                    except IndexError:
                        break #or return something

                    for label,value in list(self._objects[y].items())[edgeCount+1:]:
                        if self.shares_edge(firstvalue,value): #if they form an edge
                            edges.append([firstLabel,label])
                    edgeCount+=1
                self._edges[y] = edges
                print(edges,"EDGES")
                #break

    def find_feature(self,**kwargs):
        chunkValues = set()
        if self.busy: return

        self.error = False
        openings = {}
        #A map of y-values and x,y pairs

        if 'feature' in kwargs and kwargs['feature'] == 'opening':
            openings = self.find_opening(depth=float(kwargs['depth']))
            if openings == {}:
                self._openings={}
                self._b1.clear()
                self.error = True
                return

            if 'width' in kwargs:
                # indices = self.indices_of_smallest_angle([self._objects[Decimal('0.500')][self._openings[Decimal('0.500')][0]][4],
                #                                    self._objects[Decimal('0.500')][self._openings[Decimal('0.500')][0]][5]],
                #                                 [self._objects[Decimal('0.500')][self._openings[Decimal('0.500')][1]][4],
                #                                    self._objects[Decimal('0.500')][self._openings[Decimal('0.500')][1]][5]])


                openingsKey = Decimal('0.500')
                if not openingsKey in self._openings:
                    openingsKey = Decimal('0.495')
                    if not openingsKey in self._openings:
                        openingsKey = Decimal('0.510')
                        if not openingsKey in self._openings:
                            self._openings = self._oldopenings
                            #Need to FIX this.


                indices = self.indices_of_screen_position(self._objects[Decimal(openingsKey)][self._openings[openingsKey][0]],
                                                          self._objects[Decimal(openingsKey)][self._openings[openingsKey][1]])

                y1 = self._objects[Decimal(openingsKey)][self._openings[openingsKey][0]][4+indices[0]]
                y2 = self._objects[Decimal(openingsKey)][self._openings[openingsKey][1]][4+indices[1]]
                #y1 and y2 are the angle from normal to the first object [0] and the second object, respectively
                #y is the total angle between the two
                y = y1+y2

                a = self._objects[Decimal(openingsKey)][self._openings[openingsKey][0]][2+indices[0]]
                b = self._objects[Decimal(openingsKey)][self._openings[openingsKey][1]][2+indices[1]]
                c = math.sqrt(float(a)**2 - 2*float(a)*float(b)*math.cos(float(math.radians(y)))+float(b)**2)
                print("indices",indices)
                print("C",c,y1,y2,openingsKey)
                if not float(kwargs['width']) < c:
                    self._b1.clear()
                    self.error=True
                    return

                #FDOprint(indices,' ', c)
                #FDOprint("y",y,"a",a,"b",b,"c",c)

            #Section is commented out, no longer think it is useful
            #depth is handled line 121
            #if 'depth' in kwargs:
            #
            #
            #     #print("openings..................")
            #
            for key in sorted(openings.keys()):
                #FDOprint("PC",key,openings[key])
                #indices = self.indices_of_smallest_angle(self._objects[key][openings[key][0]][4:6],
                #                                         self._objects[key][openings[key][1]][4:6])
                indices = self.indices_of_screen_position(self._objects[key][openings[key][0]],
                                                          self._objects[key][openings[key][1]])
                #FDOprint("DT",indices)
                #FDOprint("DT",self._objects[key][openings[key][0]][indices[0]],self._objects[key][openings[key][1]][indices[1]])
                x1 = self._objects[key][openings[key][0]][indices[0]]
                x2 = self._objects[key][openings[key][1]][indices[1]]
                xs = [x1,x2]
                xs.sort()
                #FDOprint("DT",xs)

                if numpy.intersect1d(self._screenLeft,numpy.arange(xs[0],xs[1])).any():               #if openings[key]
                    chunkValues.add('screenLeft')
                if numpy.intersect1d(self._screenCenter,numpy.arange(xs[0],xs[1])).any():
                    chunkValues.add('screenCenter')
                if numpy.intersect1d(self._screenRight,numpy.arange(xs[0],xs[1])).any():
                    chunkValues.add('screenRight')

        #Result Error if not 'feature' (for now)
        else:
            self._b1.clear()
            self.error=True

        # self.busy=True
        # d=self.delay
        # if self.delay_sd is not None:
        #     d=max(0,self.random.gauss(d,self.delay_sd))
        # yield d
        # self.busy=False
        print("chunkvalues",chunkValues)
        self._b1.set(kwargs['feature']+':'+'_'.join(chunkValues))
        #print("OPENINGS", openings)
        # self._internalChunks.append(ccm.Model(feature='opening',
        #                                       screenRight=))


    def similar_depth(self,list1,list2,depth=0.0):
        '''Returns True if list1[2:] and list2[2:] have similar depths.
           Similarity is based on <= depth/5'''
        #print("similar_depth")
        #print("Depth/5",depth/5)
        #print(list1,list2)
        #this function should suffer from the same issue, not specific (see within_depth)
        #dividend = 5
        #return abs(list1[2] - list2[2]) < depth/dividend or \
        #       abs(list1[2] - list2[3]) < depth/dividend or \
        #       abs(list1[3] - list2[2]) < depth/dividend or \
        #       abs(list1[3] - list2[3]) < depth/dividend

        allData1 = self._objects[list1[0]][list1[1]]
        allData2 = self._objects[list2[0]][list2[1]]

        #indices = self.indices_of_smallest_angle(allData1[4:6],allData2[4:6])
        indices = self.indices_of_screen_position(allData1,allData2)
        minDepth = depth/5.0

        linearDepth1 = float(self._objects[list1[0]][list1[1]][2+indices[0]]) * math.cos(math.radians(self._objects[list1[0]][list1[1]][4+indices[0]]))
        linearDepth2 = float(self._objects[list2[0]][list2[1]][2+indices[1]]) * math.cos(math.radians(self._objects[list2[0]][list2[1]][4+indices[1]]))
        #print("linearDepth",linearDepth1,linearDepth2)
        #FDOprint("EX",list1,self._objects[list1[0]][list1[1]][2+indices[0]])
        #FDOprint("EX",list2,self._objects[list2[0]][list2[1]][2+indices[1]])
        #FDOprint("EX",abs(self._objects[list1[0]][list1[1]][2+indices[0]] - self._objects[list2[0]][list2[1]][2+indices[1]]))
        #FDOprint("EX",abs(self._objects[list1[0]][list1[1]][2+indices[0]] - self._objects[list2[0]][list2[1]][2+indices[1]]) < depth/5.0)

        return abs(linearDepth1-linearDepth2) < 0.50
        #return abs(self._objects[list1[0]][list1[1]][2+indices[0]] - self._objects[list2[0]][list2[1]][2+indices[1]]) < depth/4.5


    def within_depth(self,list1,list2,depth=0.0):
        '''Returns True if list1[2:] and list2[2:] have a minimum difference of depth'''
        #print("withinDepth", list1,list2)
        #This is not specific enough.  Because the wall could be (at the farthest) close to the depth of the
        #target.
        #You need the actual indices

        allData1 = self._objects[list1[0]][list1[1]]
        allData2 = self._objects[list2[0]][list2[1]]


        #indices = self.indices_of_smallest_angle(allData1[4:6],allData2[4:6])
        #Replacement
        indices = self.indices_of_screen_position(allData1,allData2)

        #This should give two values e.g. [0,1]
        #add these indexes to the indexes of interest, in this case [2]
        #You will get either [2] or [3] (in example [2,3]
        #compare the depths using those indices
        linearDepth1 = float(self._objects[list1[0]][list1[1]][2+indices[0]]) * math.cos(math.radians(self._objects[list1[0]][list1[1]][4+indices[0]]))
        linearDepth2 = float(self._objects[list2[0]][list2[1]][2+indices[1]]) * math.cos(math.radians(self._objects[list2[0]][list2[1]][4+indices[1]]))
        #FDOprint("DX",list1,self._objects[list1[0]][list1[1]][2+indices[0]])
        #FDOprint("DX",list2,self._objects[list2[0]][list2[1]][2+indices[1]])
        #FDOprint("DX",abs(self._objects[list1[0]][list1[1]][2+indices[0]] - self._objects[list2[0]][list2[1]][2+indices[1]]) < depth)

        return abs(linearDepth1-linearDepth2) < depth

        #return abs(self._objects[list1[0]][list1[1]][2+indices[0]] - self._objects[list2[0]][list2[1]][2+indices[1]]) < depth


        #return abs(list1[2] - list2[2]) < depth or \
        #       abs(list1[2] - list2[3]) < depth or \
        #       abs(list1[3] - list2[2]) < depth or \
        #       abs(list1[3] - list2[3]) < depth


    def indices_of_screen_position(self,list1,list2):
        '''Takes the entire list of data and determines the correct indices'''

        l1Index = 0
        l2Index = 0
        value = 999
        for i in range(2):
            for z in range(2):
                if abs(list1[i]-list2[z]) < value:
                    value = abs(list1[i]-list2[z])

                    l1Index = i
                    l2Index = z
        return [l1Index,l2Index]

    def indices_of_smallest_angle(self,list1,list2):

        l1Index = 0
        l2Index = 0
        value=999
        for i in range(2):
            for z in range(2):
                #print("i,z",i,' ',z)
                #FDOprint("prevalue", list1[i]+list2[z])
                if abs(list1[i]-list2[z]) < value:
                    value = abs(list1[i]-list2[z])
                    #FDOprint("value", value)

                    l1Index = i
                    l2Index = z
        return [l1Index,l2Index]


    def smallest_angle(self,list1,list2):
        '''Returns the minimum combined angle between list1[0,1] and list2[0,1]'''
        return min([list1[0]+list2[0],list1[0]+list2[1],list1[1]+list2[0],list1[1]+list2[1]])

    def vectors_of_smallest_angles(self,listOfVectors1,listOfVectors2):
        '''Returns the vectors which compose the smallest angle between 2 sets of vectors'''

        #Get the combination of all the vectors
        combs = [(x,y) for x in listOfVectors1 for y in listOfVectors2]





    def find_opening(self,depth=0.0):
        '''Uses numpy for now.'''
        #Forces a scan first
        self._oldopenings = self._openings
        self.scan()
        openings = {}

        fullRange = numpy.array([])
        similar_key_major = []
        #print( "DEPTH", depth)
        for y in sorted(self._objects.keys()):
            # if y == Decimal('0.500'):
            #     print('0.500')

            fullX = numpy.arange(Decimal('0.000'),Decimal('1.0'),Decimal('0.002'))
            if len(self._objects[y].keys()) > 1:
                similar_keys_minor = []
                for ky,ty in rolling_window(numpy.array(sorted(self._objects[y],key=lambda lst: min(self._objects[y][lst][2:4]))),2):#self._objects[y]:
                    #print("DDDDDD",y,ky,ty,similar_keys_minor)
                    if ky in self._ignoreLabels or ty in self._ignoreLabels:
                        continue
                    # if ky == 'target' or ty == 'target':
                    #     print("ASDF")
                    #if self.similar_depth(self._objects[y][ky],self._objects[y][ty],depth):
                    if self.similar_depth([y,ky],[y,ty],depth):
                        similar_keys_minor.append(ky)
                        similar_keys_minor.append(ty)

                if not similar_keys_minor:
                    continue
                #print(similar_keys_minor)
                for key in similar_keys_minor:
                    #FDOprint("KEY",key)
                    #print(self._objects[y][key],"ASDFASDFA")
                    #print("FULLX",fullX)
                    t = numpy.arange(Decimal(self._objects[y][key][0]),Decimal(self._objects[y][key][1]),Decimal(0.002))

                    fullX = numpy.setdiff1d(fullX,t)
                    #FDOprint("FullX AFTER",len(fullX))
                for key in self._objects[y].keys():
                #Check against ALL the keys
                    if key in similar_keys_minor:
                        continue
                    else:
                        #Are they beyond the distance?
                        #If they within, they're too close
                        #Only need 1 item below
                        #if self.within_depth(self._objects[y][key],self._objects[y][similar_keys_minor[0]],depth):
                        if self.within_depth([y,key],[y,similar_keys_minor[0]]):
                            #FDOprint("key",key,"This happened")
                            openings[y] = []
                        else:
                            openings[y] = similar_keys_minor
                #At this point I should have a collection of objects with similar depths

                    #print(self._objects[y][ky])
                    #fullX = numpy.setdiff1d(fullX,numpy.arange(self._objects[y][ky][0],self._objects[y][ky][1],0.01))
            else:
                pass #not sure what to do if there's only 1 object.
            #print("FR",len(fullRange)).
        print("OBJECTS",self._objects)
        print("OPENINGS", openings)
        self._openings = openings
        return openings


    def process_image(self):
        img = middleware.request('get_image',[])



    def scan(self,delay=0.00):

        #print(ccm.middle)
        #import time
        #now = time.time()

        #'scan_image' will now use scan_image_multi
        #a new multiprocessing version of scan_image
        self._objects = middleware.request('scan_image',[])

        ###!!!Note the keys here are strings, not floats
        ###!!Converti them to float below
        #self._objects = dict((float(k), v) for k,v in self._objects.items())
        self._objects = dict((Decimal(k).quantize(Decimal('.001'),rounding=ROUND_HALF_UP), dict((kk,[Decimal(x).quantize(Decimal('.001'),rounding=ROUND_HALF_UP) for x in kv]) for kk,kv in v.items())) for k,v in self._objects.items())
        self._ignoreLabels = ['None','Ground']
        #self.find_edges()
        ##print(self._edges)

        #print("Obejects:",self._objects)


        #for y in sorted(self._objects.keys()):
        #    print(y,self._objects[y],"WTF...")

        #Just to plot some stuff
        #######################
        ####PLOTTING###########
        #
        # #Find all labels
        # labels = set()
        # for y in self._objects.keys():
        #     for label in self._objects[y]:
        #         labels.add(label)
        #
        # import pdb
        # #pdb.set_trace()
        # if 'None' in labels:
        #     labels.remove('None')
        # diffs = {}
        # thisValue = 0.0
        # lastValue = 0.0
        # labels = list(labels)
        # for label in labels:
        #     lastValue = -1.0
        #     raws=[]
        #     for y in sorted(self._objects.keys()):
        #         if label in self._objects[y].keys():
        #             thisValue = self._objects[y][label][2]
        #             #if label == 'Ground' and y > 0.2:
        #             #    pdb.set_trace()
        #             #raws.append(thisValue)
        #             if not label in diffs.keys():
        #                 diffs[label] = [[],[]]
        #             if lastValue < 0:
        #                 lastValue = thisValue
        #                 continue
        #             if np.isnan(np.average(np.array(diffs[label][0]))) and lastValue >= 0:
        #                 diffs[label][0].append(abs(thisValue-lastValue))
        #                 diffs[label][1].append(y)
        #                 lastValue = thisValue
        #                 continue
        #             if abs(thisValue-lastValue) <= 2:
        #                 diffs[label][0].append(abs(thisValue-lastValue))
        #                 diffs[label][1].append(y)
        #                 lastValue = thisValue
        # self._ignoreLabels = ['None']#Ignore None from now on
        #
        # for lbl in diffs.keys():
        #     plt.plot(diffs[lbl][1],diffs[lbl][0],label=lbl)
        #     z = np.polyfit(diffs[lbl][1],diffs[lbl][0],3,full=True)
        #     p = np.poly1d(z[0])
        #     xp = np.linspace(min(diffs[lbl][1]),max(diffs[lbl][1]),100)
        #     plt.plot(xp,p(xp),'--')
        #     print(lbl + 'z: ' + repr(z[1]))
        #     if z[1][0] > 0.1:
        #         self._ignoreLabels.append(lbl)
        # plt.show()


                # for ky in self._objects[y]:
                #     #if ky == 'target':
                #         #pdb.set_trace()
                #     if not ky in diffs:
                #         diffs[ky] = [[],[]]
                #     thisValue = self._objects[y][ky][2]
                #     if abs(thisValue-lastValue) < 1:
                #         diffs[ky][0].append(abs(thisValue-lastValue))
                #         diffs[ky][1].append(y)
                #     lastValue = thisValue



        #Collect the ground, near centre
        # diffValues = []
        # Grounds = []
        # ys = []
        # lastValue = 0.0
        # thisValue = 0.0
        # for y in sorted(self._objects.keys()):
        #
        #     try:
        #         thisValue = self._objects[y]['target'][2]
        #     except KeyError:
        #             continue
        #     if not abs(thisValue-lastValue) > 1:
        #         diffValues.append(abs(thisValue-lastValue))
        #         ys.append(y)
        #     lastValue = thisValue
        #
        # diffValues = np.array(diffValues)
        # print("AVERAGE",np.average(diffValues))
        # ys = np.array(ys)
        # z = np.polyfit(ys,diffValues,2,full=True)
        # print("Z", z)
        # p = np.poly1d(z[0])
        # #print(ys)
        # #print(len(y))
        # print(diffValues)
        # xp = np.linspace(min(ys),max(ys),100)
        # plt.plot(ys,diffValues,'-',xp,p(xp),'--')
        # plt.show()

        #Standard Deviations
        # vals = {}
        # for y in sorted(self._objects.keys()):
        #     for ky in self._objects[y]:
        #         if not ky in vals:
        #             vals[ky] = []
        #         vals[ky].append(self._objects[y][ky][3])
        # #print("stds",vals)
        # stds = []
        # plt.plot(vals['RightWall'])
        # plt.show()
        # for key in vals.keys():
        #     print(key)
        #     dVals = np.array(vals[key])
        #     stds.append(np.std(dVals))
        # print("STDS",stds)
        # stds.sort()





        #print(self,._objects.keys(), "HEYSSSSSS")
        #print("Time:")
        #print(time.time() - now)
        #print(self._objects, "objects")
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
   
    def check_match(self, **kwargs):
        if 'opening' in kwargs:
            if hasattr(self,'_'+kwargs['opening']):#Is this _screenCentre
                #In the space of the screen centre,
                #Is there a patch that is wide enough?
                print("PASS")
            else:
                self.error=True
                self.busy=False
        else:
            self.error=True
            self.busy=False

    def request(self,pattern=''):
        print("REQUEST")
        if self.busy: return

        matcher = Pattern(pattern)

        self.error=False
        r=[]
        for obj in self._internalChunks:
            print("one")
            if matcher.match(obj)!=None:
                print("Not None", obj)
        #self._internalEnvironment.__convert()
        #print(dir(self._internalEnvironment), "InternalEnvironment")
        #print(self._internalEnvironment.poop.isa,"poooooooop")
       ## print(matcher.match(self._internalEnvironment), "matcher")

        #print(self.parent, "parent")
        #print(self.parent.parent, "parent.parent")
       # print(self.parent.parent.get_children(), "get_children()")

       # print("CHILDREN")

        #for obj in self.parent.parent.get_children():
        #    print("CHILD",obj)
      ##  if matcher.match(self._internalEnvironment) is not  None:
      ##      print("not none")
        #pattern_list = pattern.split()
        #for p in pattern_list:
        #    (slot,value) = self.parse(p)
        #    if 'obj' in slot:
        #        print('obj')
            
        
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
      
      
      
        
