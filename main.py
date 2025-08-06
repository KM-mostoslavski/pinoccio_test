from pathlib import Path

import pinocchio


class Robot:
    def __init__(self):
        self.model = ""
        self.data = ""
        self.frame = ""

    def init(self):
        urdf_filename = "./robot.urdf"
        # Load the urdf model
        self.model = pinocchio.buildModelFromUrdf(urdf_filename)
        print("model name: " + self.model.name)
        # Create data required by the algorithms
        self.data = self.model.createData()

        self.set_collection_of_angle_q()

    def set_collection_of_angle_q(self):
        # Sample a random configuration
        self.frame = pinocchio.randomConfiguration(self.model)
        print(f"q: {self.frame.T}")

    def forwardKinematics(self):
        # Perform the forward kinematics over the kinematic tree
        pinocchio.forwardKinematics(self.model, self.data, self.frame)
        self.printForwardKinematicsResult()  # gives carthesian points

    def printForwardKinematicsResult(self):
        # Print out the placement of each joint of the kinematic tree
        model = self.model
        data = self.data
        for name, oMi in zip(model.names, data.oMi):
            print(
                "{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat)
            )

    def moveJoint(self):
        print("mv joint")
        # theta angle
        # Robot.Frame.moveMotorOfFrame() -> Perfect BUT new Frame class that is kinda useless
        # Robot.moveMotor() -> Which "joint or frame"
        # pinoccio.moveMotor() -> Which robot and which Motor ?


def main():
    r = Robot()
    r.init()
    r.forwardKinematics()
    r.moveJoint()


if __name__ == "__main__":
    main()
