#!/usr/bin/env python3
import ctre
import navx
import wpilib
import wpilib.buttons
from wpilib.drive import MecanumDrive


class MyRobot(wpilib.SampleRobot):

    def robotInit(self):

        # Channels on the roboRIO that the motor controllers are plugged in to
        frontLeftChannel = 0
        rearLeftChannel = 2
        frontRightChannel = 1
        rearRightChannel = 3

        # CAN bus identities of the CAN bus motor controllers.
        CargoIntakeChannel1 = 6
        CargoIntakeChannel2 = 7
        CargoPivotChannel1 = 4
        CargoPivotChannel2 = 5

        # Channel of the winch used to lift the lotus module.
        lotusWinchChannel = 4

        # The channel on the driver station that the joystick is connected to
        self.joystickChannel = 0
        self.functStickChannel = 1

        # launches our cameras
        wpilib.CameraServer.launch()

        # self.automodes = autonomous.AutonomousModeSelector("autonomous")

        # sets up the smartdashboard and timer for the navx gyro system.
        # self.sd = wpilib.SmartDashboard
        # self.timer = wpilib.Timer()

        # sets up the navx itself. There are two different connection methods.
        # self.navx = navx.AHRS.create_spi()
        # self.navx = navx.AHRS.create_i2c()

        # self.analog = wpilib.AnalogInput(navx.getNavxAnalogInChannel(0))

        # sets up the drivetrain motors on our robot.
        frontLeftMotor = wpilib.Spark(frontLeftChannel)
        rearLeftMotor = wpilib.Spark(rearLeftChannel)
        frontRightMotor = wpilib.Spark(frontRightChannel)
        rearRightMotor = wpilib.Spark(rearRightChannel)

        # sets up the other motors on our robot.
        self.lotusWinch = wpilib.Spark(lotusWinchChannel)

        self.CargoPivot1 = ctre.victorspx.VictorSPX(CargoPivotChannel1)
        self.CargoPivot2 = ctre.victorspx.VictorSPX(CargoPivotChannel2)
        self.CargoIntake1 = ctre.talonsrx.TalonSRX(CargoIntakeChannel1)
        self.CargoIntake2 = ctre.talonsrx.TalonSRX(CargoIntakeChannel2)

        # sets up the joysticks to control the robot.
        self.driveStick = wpilib.Joystick(self.joystickChannel)
        self.functStick = wpilib.Joystick(self.functStickChannel)

        # sets up the solenoids to run the lotus.
        self.LotusOut = wpilib.Solenoid(0, 0)
        self.LotusIn = wpilib.Solenoid(0, 1)

        # sets our motors into the drive function.
        self.drive = MecanumDrive(frontLeftMotor,
                                  rearLeftMotor,
                                  frontRightMotor,
                                  rearRightMotor)

        # turns off motors after 0.1 seconds
        self.drive.setExpiration(0.1)

    # autonomous mode. INCOMPLETE, DO NOT USE!!!
    # todo: complete autonomous mode
    def autonomous(self):
        """Called when autonomous mode is enabled"""

        while self.isAutonomous() and self.isEnabled():

            # alternate drive functions for the driver
            # sideways only
            if self.driveStick.getRawButton(1):
                self.drive.driveCartesian(
                    self.driveStick.getX() * (-self.driveStick.getZ(
                    ) + 1) / 2,
                    0,
                    0
                )
            # sideways only
            elif self.driveStick.getRawButton(2):
                self.drive.driveCartesian(
                    0,
                    self.driveStick.getY() * (-self.driveStick.getZ(
                    ) + 1) / 2,
                    0
                )
            # turning only
            elif self.driveStick.getRawButton(7):
                self.drive.driveCartesian(
                    0,
                    0,
                    self.driveStick.getThrottle() * (-self.driveStick.getZ(
                    ) + 1) / 2
                )
            # normal drive function if no button is being pressed
            else:
                self.drive.driveCartesian(
                    self.driveStick.getX() * (
                            -self.driveStick.getZ() + 1)
                    / 2,
                    -self.driveStick.getY() * (
                            -self.driveStick.getZ() + 1)
                    / 2,
                    self.driveStick.getThrottle() * (
                            -self.driveStick.getZ() + 1) / 2
                )

            self.LotusIn.set(not self.functStick.getTrigger())
            self.LotusOut.set(self.functStick.getTrigger())

            if self.functStick.getRawButton(2):
                self.lotusWinch.set(-1.0)

            elif self.functStick.getRawButton(7):
                self.lotusWinch.set(1.0)

            else:
                self.lotusWinch.set(0)

            if self.functStick.getRawButton(5):
                self.CargoIntake1.set(ctre.ControlMode.PercentOutput, 0.9)

                self.CargoIntake2.set(ctre.ControlMode.PercentOutput, -0.9)

            # runs the cargo intake the other way.
            elif self.functStick.getRawButton(6):
                self.CargoIntake1.set(ctre.ControlMode.PercentOutput, -0.9)

                self.CargoIntake2.set(ctre.ControlMode.PercentOutput, 0.9)

            else:
                self.CargoIntake1.set(ctre.ControlMode.PercentOutput, 0)
                self.CargoIntake2.set(ctre.ControlMode.PercentOutput, 0)

            # moves our cargo handler based on the y axis of the controller.
            if self.functStick.getY() > -0.2 and self.driveStick.getY() < 0.2:
                self.CargoPivot1.set(ctre.ControlMode.PercentOutput, -0.25)

                self.CargoPivot2.set(ctre.ControlMode.PercentOutput, -0.25)
            else:
                self.CargoPivot1.set(ctre.ControlMode.PercentOutput,
                                     -0.45 * self.functStick.getY())

                self.CargoPivot2.set(ctre.ControlMode.PercentOutput,
                                     -0.45 * self.functStick.getY())

            wpilib.Timer.delay(0.005)

    def operatorControl(self):

        # non functional takeover functional, not to important, so i will finish it later.
        # todo: finish the driver takeover
        # if self.functStick.getRawButton(3):
        #     self.driveStick = 0

        # freezes the driver if they are out of control. this is useful if they are doing something stupid.
        # todo: fix the issue with the freeze function.

        # enables safety.
        self.drive.setSafetyEnabled(True)
        while self.isOperatorControl() and self.isEnabled():

            if self.functStick.getRawButton(4):
                self.driveStick = wpilib.Joystick(3)
            else:
                self.driveStick = wpilib.Joystick(self.joystickChannel)

            # alternate drive functions for the driver
            # sideways only
            if self.driveStick.getRawButton(1):
                self.drive.driveCartesian(
                    self.driveStick.getX() * (-self.driveStick.getZ(
                    ) + 1) / 2,
                    0,
                    0
                )
            # sideways only
            elif self.driveStick.getRawButton(2):
                self.drive.driveCartesian(
                    0,
                    self.driveStick.getY() * (-self.driveStick.getZ(
                    ) + 1) / 2,
                    0
                )
            # turning only
            elif self.driveStick.getRawButton(7):
                self.drive.driveCartesian(
                    0,
                    0,
                    self.driveStick.getThrottle() * (-self.driveStick.getZ(
                    ) + 1) / 2
                )
            # normal drive function if no button is being pressed
            else:
                self.drive.driveCartesian(
                    self.driveStick.getX() * (
                            -self.driveStick.getZ() + 1)
                    / 2,
                    -self.driveStick.getY() * (
                            -self.driveStick.getZ() + 1)
                    / 2,
                    self.driveStick.getThrottle() * (
                            -self.driveStick.getZ() + 1) / 2
                )

            # up and down functions for the lotus elevator.
            if self.functStick.getRawButton(2):
                self.lotusWinch.set(-1.0)

            elif self.functStick.getRawButton(7):
                self.lotusWinch.set(1.0)

            else:
                self.lotusWinch.set(0)

            # sets the cargo intake motors to be a speed controlled by the throttle on a joystick
            if self.functStick.getRawButton(5):
                self.CargoIntake1.set(ctre.ControlMode.PercentOutput, 0.9)

                self.CargoIntake2.set(ctre.ControlMode.PercentOutput, -0.9)

            # runs the cargo intake the other way.
            elif self.functStick.getRawButton(6):
                self.CargoIntake1.set(ctre.ControlMode.PercentOutput, -0.9)

                self.CargoIntake2.set(ctre.ControlMode.PercentOutput, 0.9)

            else:
                self.CargoIntake1.set(ctre.ControlMode.PercentOutput, 0)
                self.CargoIntake2.set(ctre.ControlMode.PercentOutput, 0)

            # moves our cargo handler based on the y axis of the controller.

            self.CargoPivot1.set(ctre.ControlMode.PercentOutput,
                                 -0.45 * self.functStick.getY())

            self.CargoPivot2.set(ctre.ControlMode.PercentOutput,
                                 -0.45 * self.functStick.getY())

            # opens and closes the lotus using the trigger.
            # start in the open position and closes when you press the trigger.
            self.LotusIn.set(not self.functStick.getTrigger())
            self.LotusOut.set(self.functStick.getTrigger())

            # puts values into the smartdashboard.
            # todo: get smartdashboard
            # self.sd.putBoolean("IsCalibrating", self.navx.isCalibrating())
            # self.sd.putBoolean("IsConnected", self.navx.isConnected())
            # self.sd.putNumber("Angle", self.navx.getAngle())
            # self.sd.putNumber("Pitch", self.navx.getPitch())
            # self.sd.putNumber("Yaw", self.navx.getYaw())
            # self.sd.putNumber("Roll", self.navx.getRoll())
            # self.sd.putNumber("Analog", self.analog.getVoltage())
            # self.sd.putNumber("Timestamp", self.navx.getLastSensorTimestamp())

            wpilib.Timer.delay(0.005)  # wait 5ms to avoid hogging CPU cycles


if __name__ == '__main__':
    wpilib.run(MyRobot)
