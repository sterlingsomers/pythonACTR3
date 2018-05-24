#from pymorse import Morse


try:
    from pymorse import Morse
except ImportError:
    robot_simulation = None

else:
    robot_simulation = Morse()




