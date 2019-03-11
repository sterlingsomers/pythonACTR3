from .morseconnection import *
#import morseconnection
import time


import signal
from contextlib import contextmanager
class TimeoutException(Exception): pass

@contextmanager
def time_limit(seconds):
    def signal_handler(signum, frame):
        print("Time out!")
        raise TimeoutException
    signal.signal(signal.SIGALRM, signal_handler)
    signal.alarm(seconds)
    try:
        yield
    finally:
        signal.alarm(0)


class morse_middleware():
    '''The middleware to handle the connection between ACT-R and Morse.'''
    
    def __init__(self):
        #from .morseconnection import robot_simulation
            #.robot_simulation
        if robot_simulation == None:
            raise Exception("pymorse was not detected, or connection was not successful.")

        self.robot_simulation = robot_simulation
        self.mustTick = False
        self.active=False
        self.modes = ['best_effort']
        self.send_dict = {'set_speed':True,
                        'move_forward':False,
                        'set_rotation':False,
                        'lower_arms':True,
                        'set_rotation_ribs':False} #{function_name:blocking?}

        self.request_dict = {'scan_imageD':True,
                'getScreenVector':True,
                'cScan':True,
                'get_time':True,
                'xScan':True,
                'getBoundingBox':True,
                'get_image':True,
                'scan_image':True,
                'get_bones':True
                }
            
        self.action_dict = {'set_rotation_ribs':['self.robot_simulation.robot.torso','.set_rotation'],
                            'get_image':['self.robot_simulation.robot.GeometricCamerav1','.get_image'],
                            'scan_image':['self.robot_simulation.robot.GeometricCamerav1','.scan_image_multi'],
                            'scan_sub_image':['self.robot_simulation.robot.GeometricCamerav1','.scan_sub_image'],
                            'getBoundingBox':['self.robot_simulation.robot','.getBoundingBox'],
                            'lower_arms':['self.robot_simulation.robot.torso','.lower_arms'],
                            'get_bones':['self.robot_simulation.robot','.getBones'],
                            'move_forward':['self.robot_simulation.robot','.move_forward']}
        #self.action_dict = {'scan_imageD':['self.robot_simulation.robot.GeometricCamerav1', '.scan_imageD'],
		#		'set_speed':['self.robot_simulation.robot','.set_speed'],
        #        'move_forward':['self.robot_simulation.robot','.move_forward'],
        #        'set_rotation':['self.robot_simulation.robot.armature','.set_rotation'],
        #        'getScreenVector':['self.robot_simulation.robot.GeometricCamerav1', '.getScreenVector'],
        #        'cScan':['self.robot_simulation.robot.GeometricCamerav1', '.cScan'],
        #        'xScan':['self.robot_simulation.robot.GeometricCamerav1', '.xScan'],
        #        'getBoundingBox':['self.robot_simulation.robot','.getBoundingBox'],
        #        'lower_arms':['self.robot_simulation.robot.armature','.lower_arms']}

        self.danger_list = ['get_time']
        self.modules_in_use = {}
        #{function_name:['absolute path to function', ['args', 'list']]}
#    def set_speed(self,speed=0.01):
#        '''Move forward @speed in m/s'''
#        x = robo.set_speed(speed).result()
        self.send_queue = [] #contains [[datastr,argslist],[datastr,argslist]]
        self.request_queue = []
        


    # def send(self, datastr, argslist):
    #     print("Trying to send", datastr)
    #     if not type(argslist) == list:
    #         raise Exception("arglist parameter must be a list")
    #     if not all(isinstance(x,str) for x in argslist):
    #         raise Exception("All arguments have to be strings")
    #     if not datastr in self.send_dict:
    #         raise Exception(datastr + " is not in send_dict. Command does not exist or must be added.")
    #     if self.mustTick:#Something has blocked
    #         self.send_queue.append([datastr,argslist])
    #     else:
    #         if self.send_dict[datastr]: #it is blocking
    #             if self.action_dict[datastr][0] in self.modules_in_use:
    #                 raise Exception("Module " + self.action_dict[datastr][0] + " is already in use this cycle by "
    #                 + self.modules_in_use[self.action_dict[datastr][0]])
    #             else:
    #                 self.modules_in_use[self.action_dict[datastr][0]] = datastr #set it as in use
    #                 #print(self.modules_in_use)
    #                 rStr = self.action_dict[datastr][0] + self.action_dict[datastr][1] + '(' + ','.join(argslist) + ')'
    #                 #print(rStr)
    #                 eval(rStr)
    #                 self.mustTick=True
    #         else: #if it's not blocking, we need to add it's location to a list, and check if it is already in the list
    #             if self.action_dict[datastr][0] in self.modules_in_use:
    #                 raise Exception("Module " + self.action_dict[datastr][0] + " is already in use this cycle by "
    #                 + self.modules_in_use[self.action_dict[datastr][0]])
    #             else:
    #                 self.modules_in_use[self.action_dict[datastr][0]] = datastr #set it as in use
    #                 #print(self.modules_in_use)
    #                 rStr = self.action_dict[datastr][0] + self.action_dict[datastr][1] + '(' + ','.join(argslist) + ')'
    #                 #print(rStr)
    #                 eval(rStr)
                                
            
    def send(self,function_name,**kwargs):


        self.send_queue.append([function_name,kwargs])


        
    def request(self, datastr, argslist):
       
        print("Trying to request", datastr)
        self.robot_simulation.tick()
        #print("mustTick", self.mustTick)
        result = None
        if not type(argslist) == list:
            raise Exception("argslist parameter must be a list")
        if not all(isinstance(x,str) for x in argslist):
            raise Exception("All arguments have to be strings")
        if not datastr in self.request_dict:
            raise Exception(datastr + " is not in request_dict. Command does not exist or must be added.")
        if self.mustTick:
            raise Exception("Blocking request already made.")#make something more informative        
        self.mustTick = True
        #print("setting mustTrick", self.mustTick)
        #print("Sending...", self.action_dict[datastr][1], argslist)
        rStr = self.action_dict[datastr][0] + self.action_dict[datastr][1] + '(' + ','.join(argslist) + ').result()'
        try:
            with time_limit(1):
                result = eval(rStr)
                #print("Here")
                #result = result()
                self.robot_simulation.tick()
                self.mustTick=False
                #print("Result:",result)
                return result

                #print("Here")

                result = result.result()
                #print("Here2")
        except TimeoutException:
            self.mustTick=False
            self.robot_simulation.tick()
            return self.request(datastr,argslist)

        print("Recieved", result)
        #if 'return' in dir(result):
        #    result = result.result()
        #while result == None:
        #   time.sleep(0.0001)
        return result




    def set_mode(self,mode,rate):
        if not mode in mode:
            raise Exception("Modes must be", self.modes)
        self.mode = mode
        self.rate = rate


    def reset(self):
        self.robot_simulation.reset()

    def tick(self,sync=False):
        #eimport time
        if self.mode == 'best_effort':
            #print("mustTick - in tick", self.mustTick)
            self.mustTick = False
            #print("mustTick - in tick 2", self.mustTick)
            self.modules_in_use = {}
            for rate in range(self.rate):
                print("Middleware tick!")

                if self.send_queue:
                    try:
                        with time_limit(1):
                            self.robot_simulation.robot.accept_send_request(self.send_queue)
                    except TimeoutException:
                        self.robot_simulation.tick()

                    self.send_queue = []
                t = self.robot_simulation.tick()
                print(t)
                #print("TIME:",self.robot_simulation.time())
                #time.sleep(0.01)

            if self.send_queue:
                raise Exception("Send queue not clear. Too many commands per cycle.")

                
                
                
                
            

                    
                    
                

        

middleware = morse_middleware()
#middleware.tick()               

#connection = robot_simulation

#def tick():
   # print("tick called")
#    connection.tick()



#def _time():
#    return connection.time()
