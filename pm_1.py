# initial code to set up Python ACT-R
import ccm
from ccm.lib.actr import *
log=ccm.log(html=True)

# define the model
class MyModel(ACTR):
    focus = Buffer()
    retrieve = Buffer()
    dm = Memory(retrieve)
    pgc = PMPGC()
    compiler = PMCompile(keep='focus', request='dm.request', retrieve='retrieve')

    def init():
        dm.add('pattern a 7')
        dm.add('pattern b 3')

    def prod1(focus='see ?x'):
        dm.request('pattern ?x ?')
        focus.set('remember ?x')

    def prod2(focus='remember ?x', retrieve='pattern ?x ?y'):
        focus.set('see b')
        #self.success()
        #self.stop()


class Model2(ACTR):
    focus = Buffer()

    retrieval = Buffer()
    memory = BlendingMemory(retrieval, threshold=0)
    DMNoise(memory, noise=0.25)
    DMBaseLevel(memory)

    #pgc = PMPGC()
    #compiler = PMCompile(keep='focus', request='dm.request', retrieve='retrieve')

    def init():
        memory.add('state 1 action 1')
        memory.add('state 2 action 4')
        memory.add('state 4 action 8')


    def prod1(focus='req',memory='busy:False error:False'):
        memory.request('state 3 action ?x')
        focus.set('rec')

    def prod2(focus='rec',retrieval='state ?x action ?y'):
        print(x,y)
        focus.set('stop')



# run the model
model=Model2()
ccm.log_everything(model)
model.focus.set('req')
#model.goal.set('action:greet')
model.run(limit=10.0)


