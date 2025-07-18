from pathlib import Path

import pinocchio


class Motor:
    def __init__(self, id):
        self.id = id
        self.position = ""
        self.current_command = ""

    def move_motor(self, joint, theta):
        print(joint)
        # print(r.x[r.frame1].id)
        # print(r.y[r.frame2].id)

        # Prints the received command that a real motor will execute
        print(f"The {joint} joint moved +{theta} degrees")


class Robot:
    def __init__(self):
        self.model = ""
        self.data = ""
        self.frame1 = ""
        self.frame2 = ""
        self.motor1 = Motor(1)
        self.motor2 = Motor(2)
        x = {}
        y = {}

    def init(self):
        urdf_filename = "./robot.urdf"
        # Load the urdf model
        self.model = pinocchio.buildModelFromUrdf(urdf_filename)
        print("model name: " + self.model.name)
        # Create data required by the algorithms
        self.data = self.model.createData()

        self.set_frame()

    def set_frame(self):
        # Sample a random configuration
        self.frame = pinocchio.randomConfiguration(self.model)
        print(f"frame: {self.frame.T}")
        self.linkMotorsToFrame()

    def linkMotorsToFrame(self):
        self.x = {self.frame1: self.motor1}
        self.y = {self.frame2: self.motor2}

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

    def moveSomething(self, frame_name, motor):
        print("mv joint")
        # thing is either:
        # r.x[r.frame1].id OR
        # r.y[r.frame2].id
        motor.move_motor(frame_name, 15)
        # theThingToMove =
        # theta angle
        # Robot.Frame.moveMotorOfFrame() -> Perfect BUT new Frame class that is kinda useless
        # Robot.moveMotor() -> Which "joint or frame"
        # pinoccio.moveMotor() -> Which robot and which Motor ?


def main():
    r = Robot()
    r.init()
    r.forwardKinematics()
    print(r.x[r.frame1].id)
    print(r.y[r.frame2].id)

    # m.move_motor("hi mom")
    r.moveSomething(r.frame1, r.x[r.motor1.id])


if __name__ == "__main__":
    main()
