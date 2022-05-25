import itertools
import os
rootDir = "Experiments/"


def setup_experiement(ExName, file, controllers, mappings, resetCPGS, clipJointValues, its, rollouts
                      , rolloutTimes, learningRates, variances, decays, repeats=1):

    exp_dir = rootDir + ExName
    if os.path.isdir(exp_dir):
        exit("Experiment folder is already made")
    os.mkdir(exp_dir)
    print("Created Experiement folder")

    inputs = [file, controllers, mappings, resetCPGS, clipJointValues, its, rollouts, rolloutTimes, learningRates,
              variances, decays]
    n_experiments = len(inputs[0])
    for i in range(1, len(inputs)):
        n_experiments *= len(inputs[i])

    for i in range(n_experiments):
        os.mkdir(exp_dir+"/ex"+str(i+1).zfill(3))
    print("Created subfolders")

    f = open(exp_dir+"/"+ExName+".sh", "w")
    f.close()

    experiments = list(itertools.product(*inputs))

    file = open(exp_dir+"/"+ExName+".sh", "a")

    for i, exp in enumerate(experiments):
        for _ in range(repeats):
            file.write("python3 ")
            for element in exp:
                file.write(str(element)+" ")
            file.write(exp_dir + "/ex" + str(i+1).zfill(3)+"/")
            file.write("\n")
    file.close()


sigma = [0.04, 0.02, 0.01]
lamb = [0.1]
decays = [0.995]
rollouts = [4]
its = [100]

setup_experiement("HyperTest", file = ["Learning.py"], controllers=["RBFN"], mappings=["direct"], resetCPGS=["false"],
                  clipJointValues=["true"], its=its, rollouts=rollouts, rolloutTimes=[4], learningRates=lamb,
                  variances=sigma, decays=decays, repeats=4)
