#Run with a morse environment already running.

#import MiddleMorse
#import Morse
#from pymorse import Morse


#import ACT-R Stuff
import ccm

#middleware = ccm.morserobots.morse_middleware()
from ccm.lib.actr import *
from ccm.lib.actr.blender_vision import BlenderVision
from ccm.lib.actr.blender_motor_module import BlenderMotorModule
#log=ccm.log()
from ccm.morserobots import middleware

class MyEnvironment(ccm.Model):
    v1 = ccm.Model(isa='dial')

#class VisionModule(ccm.Model):
#    poop=ccm.Model(isa='dial',value=-1000)

class VisionMethods(ccm.ProductionSystem):
    production_time = 0.030
    fake_buffer = Buffer()
    
    def init():
        fake_buffer.set('asdf')#should be 'fake'

    def repeat(fake_buffer='fake'):
        #This could be used during movement, actively doing the task

        #self.parent.vision_module.scan()
        #self.parent.vision_module.getScreenVector('0.4999','0.5')    
        #self.parent.vision_module.cScan('0.5')#50cm minimum depth for an opening.
        #self.parent.vision_module.xScan('0.3','0.5')
        




# define the model
class MyModel(ACTR):
    goal=Buffer()
    
    b_motor = Buffer()#motor buffer. will be VERY complex, likely.
    b_plan_unit=Buffer()
    b_unit_task=Buffer()
    b_cue=Buffer() #allows you to do things systematically, or at least based on previous action - don't end up staring at 1 dial
    b_method=Buffer()
    b_operator=Buffer()
    
    b_vision1 = Buffer()
    b_vision2 = Buffer()
    #vm = SOSVision(b_vision)    
    vision_module = BlenderVision(b_vision1,b_vision2)
    #p_vision=VisionModule(b_vision1)


    motor_module = BlenderMotorModule(b_motor)
    
    vm = VisionMethods()
    
    DMbuffer=Buffer()
    DM=Memory(DMbuffer,latency=0.0)




    def init():
        DM.add('planning_unit:find_target unit_task:find_target')
              
        #DM.add('planning_unit:prepare_for_Take_off unit_task:starter cue:break_on')
        
        b_plan_unit.set('planning_unit:find_target')
        b_unit_task.set('unit_task:none')
        b_operator.set('operator:none')
        b_cue.set('cue:none')
        goal.set('stop')

        import math
        #input new model stuffs here:
        #self.motor_module.rotate_torso('0',repr(math.radians(90)))

        #self.motor_module.lower_arms()
        #self.motor_module.set_speed('0.01')

        #import math

        #self.motor_module.set_rotation('ribs','0',repr(math.radians(90)))
        #self.motor_module.set_rotation('arm_upper.R','0',repr(math.radians(35)))

        #get bounding box here.
        #self.middleware.request('getBoundingBox', [])

    def estimate_passability_retrieveUT(b_plan_unit='planning_unit:find_target', b_unit_task='unit_task:none',
                                        b_operator='operator:none'):
        print("fire estimate_passsability_retrieveUT")
        DM.request('planning_unit:find_target unit_task:?')
        b_operator.set('operator:retrieveUT')
        #b_cue.set('cue:retrieving_task')
        #goal.set('stop')
        #b_plan_unit.set('planning_unit:none')

    def estimate_passability_recalledUT(b_plan_unit='planning_unit:find_target', b_unit_task='unit_task:none',
                                        b_operator='operator:retrieveUT',
                                        DMbuffer='unit_task:?UT'):
        b_unit_task.set('unit_task:' + UT)
        b_operator.set('operator:none')
        DMbuffer.clear()

    def estimate_passability_find_opening(b_plan_unit='planning_unit:find_target', b_unit_task='unit_task:find_target',
                                            b_operator='operator:none'):
        #vision_module.cScan()
        print("estimate_passability_find_opening")
        #vision_module.request('isa:dial')
        goal.set('stop')
        b_plan_unit.set('planning_unit:none')
        

    #def estimate_passability_two(b_plan_unit='planning_unit:estimate_passability',
    #                             b_unit_task='unit_task:get_task',
    #                             b_cue='cue:retrieving_task', DMbuffer='unit_task:?UT'):
    #    
    #    print(UT)
    #    goal.set('stop')

    def three(goal='action:three'):

        goal.set('action:four')

    def four(goal='action:four'):


      
        goal.set('action:five')

    def five(goal='action:five'):

        goal.set('action:six')

    def six(goal='action:six'):
        goal.set('stop')


    def stop(goal='stop'):
        self.keepAlive = False
    
        
        

model=MyModel()
model.middleware = middleware
#vInternal = VisualEnvironment()
env = MyEnvironment()
env.agent = model
ccm.log_everything(env)
model.goal.set('action:greet')

#initialize ACT-R
model.run(0)
model.keepAlive = True
print("Pre-run")

middleware.set_mode('best_effort',2)
#best effort will try to clear the stack
#will tick n times for every tick. the defaul py must be set right.

#initial sync
middleware.tick()

while model.keepAlive:

    #model.vision_module.scan()
    model.run(0.01)
    print("TICK...................")

    middleware.tick(sync=True)

print("post run")
  
   



