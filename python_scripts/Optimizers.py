# Container for optimizers
import numpy as np
from sklearn.metrics import mean_squared_error


class PiBB():
    def __init__(self, n_rollouts, itterations, func, params, rooloutLegth=4, sigma=1.0, lamb=0.01, decayRate=0.99,
                 dataDir="data/", gaitNr=0):
        self.startingParams = params
        self.n_params = len(params)

        self.bestParams = params

        self.lowestCost = 100000

        self.alltimeBestParam = params

        self.itteratins = itterations
        self.n_rollouts = n_rollouts
        self.rolloutLenght = rooloutLegth

        # hyperparams
        self.initialSigma = sigma
        self.sigma = sigma
        self.lamb = lamb
        self.decayRate = decayRate

        self.func = func

        self.currentItteration = 0

        # internal params
        self.epsilons = None
        self.S = np.zeros(self.n_rollouts)
        self.P = None
        self.deltaParams = None

        self.cost = []
        self.paramHist = np.expand_dims(params, -1)

    def reset(self, gaitNr=0):

        self.lowestCost = 100000

        # hyperparams
        self.sigma = self.initialSigma

        self.currentItteration = 0

        # internal params
        self.epsilons = None
        self.S = np.zeros(self.n_rollouts)
        self.P = None
        self.deltaParams = None

        self.cost = []
        self.paramHist = np.expand_dims(self.startingParams, -1)

    @staticmethod
    def limit_params(params):
        lim = 1 / (6.2347) * 0.45
        return np.clip(params, -lim, lim)

    def exploration(self):
        self.epsilons = np.random.normal(0, self.sigma, (self.n_rollouts, self.n_params))

        # create new params with noise
        new_params = self.epsilons + self.bestParams

        for i in range(self.n_rollouts):
            self.S[i], _ = self.func(self.rolloutLenght, new_params[i, :])

        self.cost.append(self.S)
        self.paramHist = np.append(self.paramHist, np.expand_dims(self.bestParams, -1), axis=-1)

    def compute_prob(self):

        nominator = np.exp(-(1 / self.lamb) * self.S)
        denom = np.sum(nominator)

        self.P = nominator / (denom + 0.000001)

    def cost_weight_averaging(self):
        self.deltaParams = np.sum(self.epsilons.T * self.P, axis=-1)

    def update_params(self):

        self.bestParams = self.bestParams + self.deltaParams

    def optimization(self):
        self.exploration()
        self.compute_prob()
        self.cost_weight_averaging()
        self.update_params()

    def eval_best_controller(self):

        eval_cost, eval_dict = self.func(self.rolloutLenght, self.bestParams)

        return eval_cost, eval_dict

    def run(self):

        print("Opimizing PIBB: rollout ")
        for i in range(self.itteratins):
            self.optimization()

            self.sigma = self.sigma * self.decayRate

            self.currentItteration += 1
            print(str(self.currentItteration))

    def run_once(self):

        self.optimization()

        # self.sigma = self.sigma * self.decayRate
        self.sigma = self.initialSigma * (1. / (1 + 0.04 * self.currentItteration))

        self.currentItteration += 1

        cost = self.eval_best_controller()

        return cost


# Test functions for optimizer
def sphere_function(time, params):
    return np.sum(np.power(params, 2))


def rosenbrock_function(time, params):
    fx = 0
    for i in range(params.shape[0] - 1):
        fx += 100 * (params[i + 1] - params[i] ** 2) ** 2 + (1 - params[i]) ** 2

    return fx


def rastrigin_function(time, params):
    A = 10
    n = params.shape[0]

    return A * n + np.sum(np.power(params, 2) - A * np.cos(2 * np.pi * params))


def test_func(time, params):
    test = np.array(range(len(params)))
    return mean_squared_error(test, params)
