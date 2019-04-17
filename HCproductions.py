#################### ham cheese production model ###################

### this is a model of making a ham and cheese sandwich

## the environment hasn't been built yet so print statements
## are used to state the actions of the agent

# this is the simplest type of act-r model
# it uses only the production system and one buffer
# the buffer represents the focus of thought
# we call it the focus buffer but it is often called the goal buffer
# productions fire if they match the focus buffer
# each production changes the contents of focus buffer so a different production will fire on the next cycle


import ccm

log=ccm.log()   

from ccm.lib.actr import *  

# Python ACT-R requires an environment
# but in this case we will not be using anything in the environment
# so we 'pass' on putting things in there

class MyEnvironment(ccm.Model):
    pass


# create an act-r agent

class MyAgent(ACTR):
    
    focus=Buffer()
    focus.set('ingredient:bread_bottom')

    def bread_bottom(focus='ingredient:bread_bottom'):   # if focus buffer has this chunk then....
        print ("I have a piece of bread")                # print
        focus.set('ingredient:cheese')                   # change chunk in focus buffer

    def cheese(focus='ingredient:cheese'):               # the rest of the productions are the same
        print ("I have put cheese on the bread")         # but carry out different actions
        focus.set('ingredient:ham')                      # because the focus buffer chunk was changed

    def ham(focus='ingredient:ham'):
        print ("I have put ham on the cheese")
        focus.set('ingredient:bread_top')

    def bread_top(focus='ingredient:bread_top'):
        print ("I have put bread on the ham")
        focus.set('ingredient:none')   

    def stop_production(focus='ingredient:none'):
        print ("I have made a ham and cheese sandwich")
        self.stop()                                      # stop the agent


tim=MyAgent()                              # name the agent
subway=MyEnvironment()                     # name the environment
subway.agent=tim                           # put the agent in the environment
ccm.log_everything(subway)                 # print out what happens in the environment
subway.run()                               # run the environment
ccm.finished()                             # stop the environment
