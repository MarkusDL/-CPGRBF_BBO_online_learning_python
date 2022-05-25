import signal
from MORFcontrollers import RBFNNControl
from MorfInstance import morfInstance
import argparse

# Create the parser
my_parser = argparse.ArgumentParser(description='List the content of a folder')

# Add the arguments
my_parser.add_argument('ControllerType', metavar='Controller', type=str, help='The controller to use: exmaple RBFN')
my_parser.add_argument('MotorMapping', metavar='Mapping', type=str, help='Motor mapping to use  either: indirect, semiIndirect, direct ')
my_parser.add_argument('ResetCPG', metavar='ResetCpg', type=str, help='Defining if the controller should reset the CPG during learning ')
my_parser.add_argument('ClipJointValues', metavar='clipJointValues', type=str, help='Defining if the joint values should be clipped')
my_parser.add_argument('usePrior', metavar='usePrior', type=str, help='Defines if the learner should start from prior knowledge')
my_parser.add_argument('Itterations', metavar='n_iter', type=int, help='Number of itterations to run ')
my_parser.add_argument('Rollouts', metavar='n_roll', type=int, help='Number of rollouts')
my_parser.add_argument('Rollout_length', metavar='roll_length', type=int, help='rollout lenght in seconds')
my_parser.add_argument('learn_rate', metavar='lr', type=float, help='learning rate for optimization')
my_parser.add_argument('variance', metavar='sigma', type=float, help='Variance for optimization')
my_parser.add_argument('decay', metavar='delta', type=float, help='decay fo variance')
my_parser.add_argument('DataDir', metavar='Directory', type=str, help='Directory for dataStorage')

# Execute the parse_args() method
args = my_parser.parse_args()
controllerType = args.ControllerType
MotorMapping = args.MotorMapping

resetCPG = args.ResetCPG
clipJointValues = args.ClipJointValues
usePrior = args.usePrior

n_rollout = args.Rollouts
n_iterations = args.Itterations
rolloutLength = args.Rollout_length
dataDir = args.DataDir

learn_rate = args.learn_rate
variance = args.variance
decay = args.decay

# make instance of morf with controller type, add controllers to controllerDict if different/new controllers are made
controllerDict = {'RBFN': RBFNNControl}

MORF = morfInstance(controllerDict[controllerType], 0, n_rollout, rolloutLength, n_iterations, MotorMapping,
                    dataDir=dataDir, decay_rate=decay, learn_rate=learn_rate, variance=variance, resetOnEval=resetCPG,
                    clipJointValues=clipJointValues, usePriorKnowledge=usePrior)


def keyboard_interrupt_handler(keysignal, frame):
    print("KeyboardInterrupt (ID: {}) has been caught. Cleaning up...".format(keysignal))
    MORF.rosHandler.stop_sim()
    print("closing.")
    exit(0)


signal.signal(signal.SIGINT, keyboard_interrupt_handler)

MORF.controller.set_disabled_legs([False, False, False, False, False, False])
MORF.run_multi_controller_learning(iterations=1)
print("learned for all legs")
MORF.controller.set_disabled_legs([False, False, True, False, False, False])
MORF.run_multi_controller_learning(iterations=2)
print("learned with 1 leg disabled")
MORF.controller.set_disabled_legs([False, False, True, True, False, False])
MORF.run_multi_controller_learning(iterations=2)
print("learned with 2 legs disabled")

MORF.rosHandler.stop_sim()
