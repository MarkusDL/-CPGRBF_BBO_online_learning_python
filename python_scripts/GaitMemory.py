import os
import json
import numpy as np


class GaitMemory:
    def __init__(self, path="GaitMemories/memory1/", loadFromFile=True):
        self.idx = 0
        self.controllerParams = {}
        self.classificationDatas = {}
        self.costs = {}

        self.path = path

        if loadFromFile:
            if not os.path.exists(self.path):
                exit("Folder for gait memory does not exist")
            self.load_from_file()
            self.idx = len(self.controllerParams.keys())

        if not os.path.exists(self.path):
            os.makedirs(self.path)

    def add(self, params, classificationData, cost):

        self.controllerParams[str(self.idx)] = params.tolist()
        self.classificationDatas[str(self.idx)] = classificationData.tolist()
        self.costs[str(self.idx)] = cost

        self.idx += 1

    def remove(self, memID):
        pass

    def choose_gait_try_all(self, func, procentageThreshold):

        print("Choosing based on try all")
        eval_costs = []

        for i in range(len(self.controllerParams)):

            cost_dict, _, _ = func(self.controllerParams[str(i)])

            eval_costs.append(cost_dict[0])

        best_idx = np.argmin(eval_costs)

        prior_cost = self.costs[str(best_idx)][0]

        if cost_dict[0] < prior_cost + abs(prior_cost*procentageThreshold):

            return True, self.controllerParams[str(best_idx)], best_idx

        else:
            return False, self.controllerParams[str(best_idx)], None

    def choose_gait_tactile_difference(self, func, procentageThreshold):
        print("Choosing based on tactile data")

        cost, eval_mean, std = func(self.controllerParams[str(0)])

        mses = []
        for key in self.classificationDatas.keys():
            mses.append(((eval_mean - self.classificationDatas[key]) ** 2).mean(axis=0))

        sorted_idxs = np.argsort(mses)

        for idx in sorted_idxs:

            cost_dict, _, _ = func(self.controllerParams[str(idx)])

            prior_cost = self.costs[str(idx)][0]

            print(cost_dict[0], prior_cost + abs(prior_cost*procentageThreshold))
            if cost_dict[0] < prior_cost + abs(prior_cost*procentageThreshold):

                return True, self.controllerParams[str(idx)], idx

        return False, self.controllerParams[str(sorted_idxs[0])], None

    def save_to_file(self):

        with open(self.path+"controllerParams.json", "w") as outfile:
            json.dump(self.controllerParams, outfile, indent=2)
        with open(self.path+"classificationDatas.json", "w") as outfile:
            json.dump(self.classificationDatas, outfile, indent=2)
        with open(self.path+"costs.json", "w") as outfile:
            json.dump(self.costs, outfile, indent=2)

    def load_from_file(self):

        if os.path.isfile(self.path+"controllerParams.json"):
            with open(self.path+"controllerParams.json") as json_file:
                self.controllerParams = json.load(json_file)
            with open(self.path+"classificationDatas.json") as json_file:
                self.classificationDatas = json.load(json_file)
            with open(self.path+"costs.json") as json_file:
                self.costs = json.load(json_file)

    @staticmethod
    def get_percentage_diff(previous, current):
        try:
            percentage = abs(previous - current) / max(previous, current) * 100
        except ZeroDivisionError:
            percentage = float('inf')
        return percentage
