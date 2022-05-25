import rospy
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout, Bool, Int32
import numpy as np
import time


class PythonMORFRosHandler:
    def __init__(self, clientID, isSim=True):

        if isSim:
            self.morfTopicName = "/morf_sim" + str(clientID)
            self.controlTopicName = "/sim_control" + str(clientID)

        rospy.init_node('pythonMorf', anonymous=True)
        self.rate = rospy.Rate(60)

        # Topics morf
        self.imuTopic = "/morf_sim/imu" + str(clientID)
        self.eulerTopic = self.morfTopicName + "/euler"
        self.jointPosTopic = self.morfTopicName + "/joint_positions"
        self.jointTorTopic = self.morfTopicName + "/joint_torques"
        self.jointVelTopic = self.morfTopicName + "/joint_velocities"
        self.jointComTopic = self.morfTopicName + "/multi_joint_command"
        self.positionHeadingTopic = self.morfTopicName + "/position_heading"
        self.legForceTopic = self.morfTopicName + "/leg_forces"

        # Topics control
        self.simStateTopic = self.controlTopicName + "/simulationState"
        self.simStepDoneTopic = self.controlTopicName + "/simulationStepDone"
        self.simTimeTopic = self.controlTopicName + "/simulationTime"
        self.startSimTopic = self.controlTopicName + "/startSimulation"
        self.stopSimTopic = self.controlTopicName + "/stopSimulation"
        self.trigNextStepTopic = self.controlTopicName + "/triggerNextStep"
        self.enableSuncTopic = self.controlTopicName + "/enableSyncMode"
        self.pausSimTopic = self.controlTopicName + "/pauseSimulation"

        self.blackoutTopic = self.controlTopicName + "/blackout"
        self.privatMsgTopic = self.controlTopicName + "/privateMsgAux"
        self.killControlTopic = self.controlTopicName + "/terminateController"

        self.resetObjectivesTopic = self.controlTopicName + "/resetObjectives"
        self.testParamTopic = self.controlTopicName + "/testParameters"

        self.JointCommandPub = rospy.Publisher(self.jointComTopic, Float32MultiArray, queue_size=1)
        self.resetObjectivesPub = rospy.Publisher(self.resetObjectivesTopic, Bool, queue_size=1)

        self.startSimPub = rospy.Publisher("/sim_control0/startSimulation/", Bool, queue_size=1)
        self.stopSimPub = rospy.Publisher(self.stopSimTopic, Bool, queue_size=1)

        self.objectives = None
        self.positionHeadingHist = []
        self.jointPositionsHist = []
        self.footForceHist = []
        rospy.Subscriber(self.testParamTopic, Float32MultiArray, self.save_test_params_cb)
        rospy.Subscriber(self.positionHeadingTopic, Float32MultiArray, self.save_position_heading_params_cb)
        rospy.Subscriber(self.legForceTopic, Float32MultiArray, self.save_force_hist_cb)
        rospy.Subscriber(self.jointPosTopic, Float32MultiArray, )

        print("Publishers and subs created")

        while self.objectives is None:
            self.start_sim()
            time.sleep(0.1)

    def save_test_params_cb(self, msg):
        data = msg.data
        self.objectives = data

    def save_position_heading_params_cb(self, msg):
        data = msg.data
        self.positionHeadingHist.append(data)

    def save_force_hist_cb(self, msg):
        data = msg.data
        self.footForceHist.append(data)

    def save_joint_pos_hist_cb(self, msg):
        data = msg.data
        self.jointPositionsHist.append(data)

    def set_target_config(self, config):

        # [M1, M2, M3, M4, M5, M6, M7, M8, M9, M10, M11, M12, M13, M14, M15, M16, M17, M18]
        mapping = [1, 7, 13, 2, 8, 14, 3, 9, 15, 4, 10, 16, 5, 11, 17, 6, 12, 18]

        if np.isnan(config).any() or np.isinf(config).any():
            exit("Nan encountered in setConfig")
        data = []

        for i in range(len(mapping)):
            data.append(i)
            data.append(config[i])

        layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = "array"
        dim.size = 1
        dim.stride = 36
        layout.dim = [dim]
        layout.data_offset = 0

        j_pos_msg = Float32MultiArray()
        j_pos_msg.data = data

        self.JointCommandPub.publish(j_pos_msg)

    def reset_test_params(self):
        self.resetObjectivesPub.publish(True)

        time.sleep(0.1)
        self.positionHeadingHist = []
        self.footForceHist = []
        self.jointPositionsHist = []
        self.objectives = None

    def start_sim(self):
        self.startSimPub.publish(True)
        time.sleep(1.0)

    def stop_sim(self):
        self.stopSimPub.publish(True)


if __name__ == '__main__':
    morf = PythonMORFRosHandler(0)
