#from pymorse import Morse

from math import pi

import ccm
from ccm.lib.actr import Buffer
from ccm.pattern import Pattern
from decimal import *

import re

from ccm.morserobots import middleware





class BlenderMotorModule(ccm.Model):
    def __init__(self,buffer1,delay=0.0,log=None,delay_sd=None):
        self._b1=buffer1
        #self._b2=buffer2
        self.delay=delay
        self.delay_sd=delay_sd
        self.error=False
        self.busy=False
        self._internalChunks = []
        self._boundingBox = []
        self.get_bounding_box()

        # self._monitor = MotorMonitor()
                             #internal name  #external       #addition arguments for external
        self.function_map = {'rotate_torso':['set_rotation',{'bone':'ribs'}],
                             'lower_arms':['lower_arms',{}],
                             'extend_shoulder':['set_rotation',{'axis':2}],
                             'compress_shoulder':['set_rotation',{'axis':2}],
                             'move_forward':['move_forward',{}]}

        self._bones = self.get_bones()
                                    #NAME      #min/max by axis: 0, 1, 2
        self._boneProperties = {'part.torso':[[0,0],[-pi/4,pi/4],[0,0]],
                                'shoulder.L':[[0,0],[0,0],[-pi/6,pi/6]],
                                'shoulder.R':[[0,0],[0,0],[-pi/6,pi/6]]}
        #Tick
        self._internalChunks.append(ccm.Model(type='posture',
                                              standing='true',
                                              prone='no',
                                              minimal_width='false',
                                              walkable='true',
                                              runnable='true',

                                              ))
        self._internalChunks.append(ccm.Model(type='proprioception',
                                              feature='shoulders_quality',
                                              quality='none'))

        self._internalChunks.append(ccm.Model(type='proprioception',
                                              feature='rotation',
                                              bone='upper_arm.R',
                                              rotation0='0.0',
                                              rotation0_quality='none',
                                              rotation1='0.0',
                                              rotation1_quality='none',
                                              rotation2='0.0',
                                              rotation_2_quality='none',
                                              overall_quality='none'))

        self._internalChunks.append(ccm.Model(type='proprioception',
                                              feature='rotation',
                                              bone='upper_arm.L',
                                              rotation0='0.0',
                                              rotation0_quality='none',
                                              rotation1='0.0',
                                              rotation1_quality='none',
                                              rotation2='0.0',
                                              rotation_2_quality='none',
                                              overall_quality='none'))

        self._internalChunks.append(ccm.Model(type='proprioception',
                                              feature='rotation',
                                              bone='shoulder.R',
                                              rotation0='0.0',
                                              rotation0_quality='none',
                                              rotation1='0.0',
                                              rotation2='0.0',
                                              rotation_quality='none'))

        self._internalChunks.append(ccm.Model(type='proprioception',
                                              feature='rotation',
                                              bone='shoulder.L',
                                              rotation0='0.0',
                                              rotation0_quality='none',
                                              rotation1='0.0',
                                              rotation2='0.0',
                                              rotation_quality='none'))

        self._internalChunks.append(ccm.Model(type='proprioception',
                                              feature='rotation',
                                              bone='torso',
                                              rotation0_quality='none',
                                              rotation0_direction='none',
                                              rotation0='0.0',
                                              rotation1='0.0',
                                              rotation2='0.0'))

        ##self._boundingBox = middleware.request('getBoundingBox', [])
        self._internalChunks.append(ccm.Model(type='proprioception',
                                              feature='bounding_box',
                                              width=repr(0.0),
                                              depth=repr(0.0),
                                              height=repr(0.0)))
        #self.blender_camera = Morse().robot.GeometricCamerav1

    def update_posture(self):
        pattern1='type:proprioception bone:upper_arm.R'# overall_quality:lowered'
        pattern2='type:proprioception bone:upper_arm.L'# overall_quality:lowered'
        pattern3='type:proprioception bone:shoulder.R'# rotation0_quality:max'
        pattern4='type:proprioception bone:shoulder.L'# rotation0_quality:min' #probably should find those, instead of supplying them
        pattern5='type:proprioception bone:torso'# rotation_direction:right'

        matcher1=Pattern(pattern1)
        matcher2=Pattern(pattern2)
        matcher3=Pattern(pattern3)
        matcher4=Pattern(pattern4)
        matcher5=Pattern(pattern5)

        pattern = 'type:posture'
        matcher = Pattern(pattern)
        for obj in self._internalChunks:
            #if axis='0.0'
            if matcher.match(obj)!=None:
                obj.minimal_width = 'false'



        matches = [matcher1,matcher2,matcher3,matcher4,matcher5]
        objs = []
        for m in matches:
            for obj in self._internalChunks:
                if m.match(obj) != None:
                    objs.append(obj)

        try:#For posture, minimal width, yes
            if objs[0].overall_quality=='lowered' and \
                objs[1].overall_quality=='lowered' and\
                objs[2].rotation0_quality=='max' and\
                objs[3].rotation0_quality=='min' and\
                objs[4].rotation_direction == 'right':

                pattern = 'type:posture'
                matcher = Pattern(pattern)


                for obj in self._internalChunks:
                    #if axis='0.0'
                    if matcher.match(obj)!=None:
                        obj.minimal_width = 'true'
                        return
        except Exception:
            pass

        try:#For posture, minimal width, yes
            if objs[0].overall_quality=='lowered' and \
                objs[1].overall_quality=='lowered' and\
                objs[2].rotation0_quality=='min' and\
                objs[3].rotation0_quality=='max' and\
                objs[4].rotation_direction == 'left':

                pattern = 'type:posture'
                matcher = Pattern(pattern)


                for obj in self._internalChunks:
                    #if axis='0.0'
                    if matcher.match(obj)!=None:
                        obj.minimal_width = 'true'
                        return
        except Exception:
            pass


    def print_state(self):
        for x in self._internalChunks:
            print("MODEL")
            for y in x.__dict__:
                if not '_' in y[0:2]:
                    print(y,x.__dict__[y])


    def lower_arms(self,function_name,**kwargs):
        '''
        This function lowers the arms completly.
        :param function_name: 'lower_arms'
        :param kwargs: {}
        :return: Moves the arms by so they are pointing downward
        :special note: The arms currently do not map properly to the desired arms for the model.
           Because the armature start somewhere near 45 (or -45) deg., when you lower the
            arms, you have to lower them to -45 (or 45) deg.,. On the ACT-R side, I'm calling
            this 0 deg. '''

        print("LOWER ARMS", kwargs)
        if self.busy:
            return
        self.busy = True

        # minR,maxR = self._boneProperties['part.torso'][kwargs['axis']]
        # #print("MINR", minR)
        # if kwargs['radians'] > maxR:
        #     #print("SET TO MAX")
        #     maxReached=True
        #     kwargs['radians'] = maxR
        # if kwargs['radians'] < minR:
        #     #print("SET TO MIN")
        #     minReached = True
        #     kwargs['radians'] = minR

        patterns = ['type:proprioception bone:upper_arm.R',
                   'type:proprioception bone:upper_arm.L']
        for pattern in patterns:
            matcher=Pattern(pattern)
            for obj in self._internalChunks:
                #if axis='0.0'
                if matcher.match(obj)!=None:
                    obj.rotation0='0'
                    obj.overall_quality='lowered'
            self.busy=False
        self.update_posture()

        middleware.send(self.function_map[function_name][0],**kwargs)


    def send(self,function_name,**kwargs):
        '''Checks for function_name in motor module. Calls function_name with kwargs on ACTR side.
            Sends function_name command through middleware to perform function on Morse side.'''
        #ACTR SIDE


        func = getattr(self,function_name)
        func(function_name,**kwargs)

        #Middleware Side
        #Each method should on this side should take care of that
        #So that any restrictions can be handled first
        #kwargs.update(self.function_map[function_name][1])

        #middleware.send(self.function_map[function_name][0],**kwargs)

    def get_bones(self):
        '''This will retrieve all the bones' names'''
        return middleware.request('get_bones',[])


    def rotate_torso(self,function_name,**kwargs):
        '''Rotate ribs on axis by radians'''
        #Check the max rotation
        print("ROTATE TORSO")
        if self.busy:
            return
        self.busy = True
        print("ROTATE TORSO2")
        maxReached = False
        minReached = False

        kwargs.update(self.function_map[function_name][1])
        print("KW", kwargs)

        minR,maxR = self._boneProperties['part.torso'][kwargs['axis']]
        #print("MINR", minR)
        if kwargs['radians'] >= maxR:
            #print("SET TO MAX")
            maxReached=True
            kwargs['radians'] = maxR
        if kwargs['radians'] <= minR:
            #print("SET TO MIN")
            minReached = True
            kwargs['radians'] = minR

        #print("RADIANS",radians)
        middleware.send(self.function_map[function_name][0],**kwargs)
        #middleware.send('rotate_torso',axis=axis, radians=radians)
        pattern='type:proprioception bone:torso'
        matcher=Pattern(pattern)
        for obj in self._internalChunks:
            #if axis='0.0'
            if matcher.match(obj)!=None:
                obj.rotation0=kwargs['radians']
                if kwargs['radians'] < 0:
                    obj.rotation_direction = 'right'
                elif kwargs['radians'] > 0:
                    obj.rotation_direction = 'left'
                if maxReached:
                    obj.rotation0_quality='max'
                if minReached:
                    obj.rotation0_quality='min'
        self.busy=False
        self.update_posture()

    def extend_shoulder(self,function_name,**kwargs):
        '''
         This function extends the shoulder (should be in ribs rotation direction)
         to help reduce agent width.
         :param function_name:
         :param kwargs: bone='shoulder.L' OR bone='shoulder.R'
         :return:applies the shoulder extension with set_rotation on Morse side
         '''
        #Check the max rotation
        print("Extend Shoulder")
        if self.busy:
            return
        self.busy = True

        maxReached = False
        minReached = False


        kwargs.update(self.function_map[function_name][1])
        if kwargs['bone'] == 'shoulder.R':
            kwargs['radians'] = kwargs['radians'] * -1
        #elif kwargs['bone'] == 'shoulder.L':
        #    kwargs['radians'] = kwargs['radians'] * 1


        minR,maxR = self._boneProperties[kwargs['bone']][kwargs['axis']]
        #print("MINR", minR)
        if Decimal(kwargs['radians']).quantize(Decimal('0.000'),rounding=ROUND_HALF_UP) >= Decimal(maxR).quantize(Decimal('0.000'),rounding=ROUND_HALF_UP):
            print("SET TO MAX")
            maxReached=True
            kwargs['radians'] = maxR
        elif Decimal(kwargs['radians']).quantize(Decimal('0.000'),rounding=ROUND_HALF_UP) <= Decimal(minR).quantize(Decimal('0.000'),rounding=ROUND_HALF_UP):
            print("SET TO MIN")
            minReached = True
            kwargs['radians'] = minR

        print("RADIANS",kwargs['radians'])
        middleware.send(self.function_map[function_name][0],**kwargs)
        #middleware.send('rotate_torso',axis=axis, radians=radians)
        pattern='type:proprioception ' + 'bone:' + kwargs['bone']
        matcher=Pattern(pattern)
        for obj in self._internalChunks:
            #if axis='0.0'
            if matcher.match(obj)!=None:
                obj.rotation0=kwargs['radians']
                if kwargs['radians'] < 0:
                    obj.rotation_direction = 'right'
                elif kwargs['radians'] > 0:
                    obj.rotation_direction = 'left'
                if maxReached: #opposite for extension
                    obj.rotation0_quality='max'
                if minReached:
                    obj.rotation0_quality='min'
        self.busy=False
        self.update_posture()


    def compress_shoulder(self,function_name,**kwargs):
        '''
         This function compresses the shoulder (should be in ribs rotation direction)
         to help reduce agent width.
         :param function_name:
         :param kwargs: bone='shoulder.L' OR bone='shoulder.R'
         :return:applies the shoulder compression with set_rotation on Morse side
         '''
        #Check the max rotation
        print("Compress Shoulder")
        if self.busy:
            return
        self.busy = True

        maxReached = False
        minReached = False


        kwargs.update(self.function_map[function_name][1])
        #if kwargs['bone'] == 'shoulder.R':
        #    kwargs['radians'] = kwargs['radians'] * 1
        if kwargs['bone'] == 'shoulder.L':
            kwargs['radians'] = kwargs['radians'] * -1


        minR,maxR = self._boneProperties[kwargs['bone']][kwargs['axis']]
        #print("MINR", minR)
        if Decimal(kwargs['radians']).quantize(Decimal('0.000'),rounding=ROUND_HALF_UP) >= Decimal(maxR).quantize(Decimal('0.000'),rounding=ROUND_HALF_UP):
            print("SET TO MAX")
            maxReached=True
            kwargs['radians'] = maxR
        if Decimal(kwargs['radians']).quantize(Decimal('0.000'),rounding=ROUND_HALF_UP) <= Decimal(minR).quantize(Decimal('0.000'),rounding=ROUND_HALF_UP):
            print("SET TO MIN")

            minReached = True
            kwargs['radians'] = minR

        print("RADIANS",kwargs['radians'])
        middleware.send(self.function_map[function_name][0],**kwargs)
        #middleware.send('rotate_torso',axis=axis, radians=radians)
        pattern='type:proprioception ' + 'bone:' + kwargs['bone']
        matcher=Pattern(pattern)
        for obj in self._internalChunks:
            #if axis='0.0'
            if matcher.match(obj)!=None:
                obj.rotation0=kwargs['radians']
                if kwargs['radians'] < 0:
                    obj.rotation_direction = 'right'
                elif kwargs['radians'] > 0:
                    obj.rotation_direction = 'left'
                if maxReached:
                    obj.rotation0_quality='max'
                if minReached:
                    obj.rotation0_quality='min'
        self.busy=False
        self.update_posture()


    def rotate_shoulders_to(self,radians):
        '''Rotates the shoulders by some percentage of maximum rotation.
            Negative rotations are possible.'''
        #print("this is happening....")
        x = torso.set_rotation('ribs',1,radians).result()

            
    def get_bounding_box(self):
        print("get_bounding_box")
        if self.busy:
            return

        self.busy = True
        self._boundingBox = [x * 1.00 for x in middleware.request('getBoundingBox', [])]

        pattern='type:proprioception feature:bounding_box'
        matcher=Pattern(pattern)
        objs = 0
        for obj in self._internalChunks:
            if matcher.match(obj)!= None:
                objs+=1
                obj.width=repr(self._boundingBox[0])
                obj.depth=repr(self._boundingBox[1])
                obj.height=repr(self._boundingBox[2])
            if objs > 1:
                raise Exception("There shouldn't be more than one match...")
        print("get_bounding_box done.")
        self.busy = False
        self.update_posture()

        # self._internalChunks.append(ccm.Model(type='proprioception',
        #                                       width=repr(self._boundingBox[0]),
        #                                       depth=repr(self._boundingBox[1]),
        #                                       height=repr(self._boundingBox[2])))
        # self._internalChunks.append(ccm.Model(isa='dial'))

    def move(self):
        pass

    def request(self,pattern=''):
        print("REQUEST")
        if self.busy: return

        #for obj in self._internalChunks:
        #    print("OBJ............")
        #    for attr, value in obj.__dict__.items():
        #        print(attr,value)

        matcher = Pattern(pattern)
        print("Matcher",matcher)

        self.error=False
        r=[]
        for obj in self._internalChunks:
            #print("one",obj)
            if matcher.match(obj)!= None:
                r.append(obj)

        self.busy = True
        d = self.delay
        if self.delay_sd is not None:
            d=max(0,self.random.gauss(d,self.delay_sd))
        yield d
        print("YEILDED", d)
        self.busy=False
        if len(r) == 0:
            self._b1.clear()
            self.error = True
        else:
            obj=self.random.choice(r)
            self._b1.set(obj)
    # if self.busy: return
    #
    # matcher=Pattern(pattern)
    #
    # self.error=False
    # r=[]
    # for obj in self.parent.parent.get_children():
    #   if matcher.match(obj)!=None:
    #     print("Not None")
    #     if not hasattr(obj,'salience') and not hasattr(obj,'visible'):
    #       continue
    #
    #     if hasattr(obj,'salience'):
    #       if self.random.random()>obj.salience:
    #         continue
    #     if hasattr(obj,'visible'):
    #       if obj.visible==False:
    #         continue
    #     if hasattr(obj,'value'):
    #       if obj.value==None:
    #         continue
    #     r.append(obj)
    #
    # self.busy=True
    # d=self.delay
    # if self.delay_sd is not None:
    #     d=max(0,self.random.gauss(d,self.delay_sd))
    # yield d
    # self.busy=False
    #
    # if len(r)==0:
    #   self._buffer.clear()
    #   self.error=True
    # else:
    #   obj=self.random.choice(r)
    #   if obj not in self.parent.parent.get_children():
    #     self._buffer.clear()
    #     self.error=True
    #   elif hasattr(obj,'visible') and obj.visible==False:
    #     self._buffer.clear()
    #     self.error=True
    #   else:
    #     self._buffer.set(obj)

    # def lower_arms(self):
    #     '''Lower the arms'''
    #     print("Motormodule_sending lower arms")
    #     middleware.send('lower_arms',[])

    def set_speed(self,speed=0.01):
        '''Move forward @speed in m/s'''
        middleware.send('set_speed',[speed])

    def get_time(self):
        print (middleware.request('get_time',[]))
    
    def async_test2(self,value):
        middleware.send('async_test2',[value])

    def move_forward(self,function_name,**kwargs):
        '''Move forward by some distance'''
        #middleware.send('move_forward',[distance])
        middleware.send(self.function_map[function_name][0],**kwargs)

    def set_rotation(self,bone,axis,radians):
        '''Rotate bone on axis by radians'''
        pass
        #Handle the ACT-R Stuff here
        #  middleware.send('set_rotation',[repr(bone),axis,radians])


