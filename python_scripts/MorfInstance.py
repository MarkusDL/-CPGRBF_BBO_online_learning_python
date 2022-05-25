import numpy as np
import matplotlib.pyplot as plt
import os
import time

from PythonMORFRosHandler import PythonMORFRosHandler
from GaitMemory import GaitMemory
from Optimizers import PiBB
from RewardFuctions import rewardFunction

import plottingTools


class morfInstance():
    def __init__(self, controller, clientID, n_rollout, rolloutLength, maxIter=None, motorMapping="indirect",
                 decay_rate=None, learn_rate=None, variance=None, dataDir=None, legsDisabled=None, resetOnEval=False,
                 clipJointValues=False, usePriorKnowledge=False):
        self.clientID = clientID
        self.dt = 1 / 60
        if legsDisabled is None:
            self.legsDisabled = [False, False, False, False, False, False]
        else:
            self.legsDisabled = legsDisabled

        self.rosHandler = PythonMORFRosHandler(self.clientID)
        self.controller = controller(self.rosHandler, motorMapping, legsDisabled=self.legsDisabled,
                                     clipJointValues=clipJointValues)
        self.maxItr = maxIter
        self.rolloutLength = rolloutLength
        self.itr = 0
        self.resetOnEval = resetOnEval

        self.costHist = []
        self.dataDir = dataDir
        self.creationTime = time.time()
        self.memory = GaitMemory(path=dataDir)
        self.usePriorKnowledge = usePriorKnowledge

        if None not in (decay_rate, learn_rate, variance,):
            self.learner = PiBB(n_rollout, None, self.run_controller_with_params,
                                self.controller.get_params(), rooloutLegth=rolloutLength, sigma=variance, lamb=learn_rate
                                , decayRate=decay_rate, dataDir=dataDir, gaitNr=self.memory.idx)
        else:
            self.learner = PiBB(n_rollout, None, self.run_controller_with_params, self.controller.get_params(),
                                rooloutLegth=rolloutLength, dataDir=dataDir, gaitNr=self.memory.idx)

        self.fig, self.axs = plt.subplots(1, 2)

    @staticmethod
    def running_mean(x, N):
        return np.convolve(x, np.ones(N) / N, mode='valid')

    def cost_stable(self, percentage, visualize=True):
        self.axs[0].clear()
        if len(self.costHist) < 5:
            if visualize:
                plottingTools.plot_singel_timeseries(self.axs[0], self.costHist, titel="Cost hist", ylabel="Cost",
                                                     xlabel="Itterations",
                                                     color="b", legend="cost")
                plt.pause(0.001)

            return False, False, False

        moving_mean = self.running_mean(self.costHist, 5)

        if visualize:
            plottingTools.plot_singel_timeseries(self.axs[0], self.costHist, titel="Cost hist", ylabel="Cost",
                                                 xlabel="Itterations",
                                                 color="b", legend="cost")
            plottingTools.plot_singel_timeseries(self.axs[0], moving_mean, titel="Cost hist", ylabel="Cost",
                                                 xlabel="Itterations",
                                                 color="m", legend="movMean cost")
            plt.pause(0.001)

        n_mean = 7
        if moving_mean.shape[0] > n_mean:
            mean_cost = np.mean(moving_mean[-n_mean:])
            # if gradinet in cost is very small or positive over the last x itterations

            threshold = np.abs(percentage * mean_cost)

            if visualize:
                plottingTools.plot_singel_timeseries(self.axs[0], [mean_cost + threshold] * len(self.costHist),
                                                     titel="Cost hist", ylabel="Cost"
                                                     , xlabel="Itterations", color="g", legend="Upper bound")
                plottingTools.plot_singel_timeseries(self.axs[0], [mean_cost - threshold] * len(self.costHist),
                                                     titel="Cost hist", ylabel="Cost"
                                                     , xlabel="Itterations", color="r", legend="Lower bound")

                plt.pause(0.001)

            within_upper_bound = moving_mean[-n_mean:] < mean_cost + threshold
            within_lower_bound = moving_mean[-n_mean:] > mean_cost - threshold

            print(" Within upper " + str(within_upper_bound))
            print(" Within lower " + str(within_lower_bound))

            #      within lower bound      , withinUpperbound        , is the newest cost degraded
            return np.all(within_lower_bound), np.all(within_upper_bound), not (within_upper_bound[-1])
        else:
            return False, False, False

    def save_params(self, params, idx, fileExtension=""):
        file_suffix = "_onlineLearningBestParams.txt"
        param_file_path = self.dataDir + str(idx)+ fileExtension + file_suffix

        if not os.path.isfile(param_file_path):
            param_file = open(param_file_path, "w")
            param_file.close()

        param_file = open(param_file_path, "a")
        print(*[str(value) for value in params], sep=",", file=param_file)
        param_file.close()

    def save_cost(self, costdict, idx, fileExtension=""):
        file_suffix = "_onlineLearning.txt"

        training_file_path = self.dataDir + str(idx) + fileExtension + file_suffix

        if not os.path.isfile(training_file_path):
            training_file = open(training_file_path, "w")
            training_file.close()

        training_file = open(training_file_path, "a")
        if os.path.getsize(training_file_path) == 0:
            print(*costdict.keys(), sep=",", file=training_file)
        print(*[str(value) for value in costdict.values()], sep=",", file=training_file)
        training_file.close()

    def run_learning(self, stopWhenConverged=True, savewithTimestamp = True):

        print("Learning Gate: ")
        converged = False

        if savewithTimestamp:
            t = time.localtime()
            timestamp = time.strftime('%b-%d-%Y_%H%M', t)
            file_prefix = timestamp
        else:
            file_prefix = ""


        while (self.maxItr is None or self.itr < self.maxItr) and not converged:

            cost = self.learner.run_once()
            self.costHist.append(cost[0])

            self.save_cost(cost[1],self.memory.idx, fileExtension=file_prefix)
            self.save_params(self.learner.bestParams,self.memory.idx, fileExtension=file_prefix)
            # if all movmean cost > lower-bound -> converged
            if stopWhenConverged:
                converged, _, _ = self.cost_stable(0.025)

            self.itr += 1
            print("Completed  itteration: " + str(self.itr))

        self.controller.reset()
        self.learner.reset()

        print("Learning Terminated")
        self.itr = 0
        self.costHist = []
        return self.learner.bestParams, cost

    def run_controller(self, params, saveCost=False, idx=0):

        print("Running Controller learned pararams")
        cost_degraded = False
        while (self.maxItr is None or self.itr < self.maxItr) and not cost_degraded:
            cost, costdict = self.run_controller_with_params(self.rolloutLength, params=params, reset=False)
            self.costHist.append(np.mean(cost))

            if saveCost:
                self.save_cost(costdict, self.memory.idx, fileExtension="_%irunController" % idx)
            _, _, cost_degraded = self.cost_stable(0.10)
            self.itr += 1

            print("Been running for: " + str(self.itr * self.rolloutLength) + " secs", end='\r')
        self.itr = 0
        cost = self.costHist[-1]
        self.costHist = []
        return cost

    def test_foot_force(self, params):
        self.reset_CPG()
        self.rosHandler.footForceHist = []

        cost = self.run_controller_with_params(self.rolloutLength, params)

        forces = np.array(self.rosHandler.footForceHist)

        std = np.std(forces, axis=0)
        mean = np.mean(forces, axis=0)

        self.rosHandler.footForceHist = []
        return cost, mean, std

    def run_multi_controller_learning(self, method="TactileEval", iterations=None):

        if len(self.memory.controllerParams.keys()) <= 0:
            # save fist learned controller, stops when not improving anymore or max it
            print("learning First gate")

            new_controller_params, new_controller_cost = self.run_learning()
            cost, force_mean, force_std = self.test_foot_force(new_controller_params)
            self.memory.add(params=new_controller_params, classificationData=force_mean, cost=cost)
            self.memory.save_to_file()
            self.costHist = []

            self.learner.reset(self.memory.idx)


            print("saving force feedback for first gait")

        it = 0
        while True and (not iterations or it < iterations):

            if method == "TryAll":
                ret, params, idx = self.memory.choose_gait_try_all(func=self.test_foot_force, procentageThreshold=0.20)
            elif method == "TactileEval":
                ret, params, idx = self.memory.choose_gait_tactile_difference(func=self.test_foot_force
                                                                              , procentageThreshold=0.20)
            else:
                exit("Wrong method defined")

            if ret:
                self.run_controller(params, saveCost=True, idx=idx)

            else:
                _, force_mean, force_std = self.test_foot_force(self.memory.controllerParams[str(0)])

                if self.usePriorKnowledge == "true":
                    self.learner.bestParams = params
                    print("using prior")
                else:
                    self.learner.bestParams = self.learner.startingParams

                params, cost = self.run_learning()
                self.memory.add(params=params, classificationData=force_mean, cost=cost)
                self.memory.save_to_file()
                self.learner.reset(self.memory.idx)

            it += 1

    def reset_CPG(self):
        self.controller.reset_CPG()

        for i in range(750):
            # update cpg for transient period
            # can be changed with fixed save state for cpg
            self.controller.cpg.update()

    def run_controller_with_params(self, secs, params=None, reset=True):

        # this code is not synronized with ros, since the speed of publication from the sim is dependint on simulation speed
        # Therefore this code is writtten so that it will wait for next publication from the sim and in this way syncronize
        # If the controller is too slow, i.e. that it takes longer to run than a sim timestep, the code will lag, and a warning printed

        if self.resetOnEval == "true" and reset:
            self.reset_CPG()

        # reset objectives/test-params
        self.rosHandler.reset_test_params()

        while self.rosHandler.objectives is None:
            # wait until first response after reset
            time.sleep(0.001)

        # set controller weights
        if params is not None:
            try:
                self.controller.set_params(params)
            except:
                exit(
                    "Controller used for does not have setParams func, if other controller is wanted add "
                    "setParams function to controller")

        initial_sim_step = int(self.rosHandler.objectives[11])
        for i in range(initial_sim_step, int(secs / self.dt) + initial_sim_step):

            self.controller.run()
            while self.rosHandler.objectives[11] < i:
                time.sleep(0.001)

            if self.rosHandler.objectives[11] - i > 1:
                print("Controller lagging behind sim ros publications, slow down sim or optimize controller")

        # read objectives from roshandler
        objectives = self.rosHandler.objectives
        position_heading = self.rosHandler.positionHeadingHist

        reward, reward_dict = rewardFunction(objectives, position_heading)
        return -reward, reward_dict

    def set_disabled_legs(self, bools):
        self.controller.set_disabled_legs(bools)

    @staticmethod
    def get_percentage_diff(previous, current):
        try:
            percentage = abs(previous - current) / max(previous, current) * 100
        except ZeroDivisionError:
            percentage = float('inf')
        return percentage


