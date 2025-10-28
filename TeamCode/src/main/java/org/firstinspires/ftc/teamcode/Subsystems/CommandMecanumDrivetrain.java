package org.firstinspires.ftc.teamcode.Subsystems;

import static com.google.blocks.ftcrobotcontroller.hardware.HardwareType.BNO055IMU;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class CommandMecanumDrivetrain extends SubsystemBase {
    private final Telemetry telemetry;
    public Motor frontRightMotor;
    public Motor frontLeftMotor;
    public Motor backLeftMotor;
    public Motor backRightMotor;
    public MecanumDrive drivetrain;
    public GoBildaPinpointDriver odo;
    public IMU imu;
    double rotateSpeed = 0;
    double desiredAngle = 0;
    private PIDFController rotatePID = new PIDFController(0.003,0,0,0);
    public CommandMecanumDrivetrain(HardwareMap hwMap, Telemetry telemetry) {
//        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        rotatePID.setTolerance(3);
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        this.telemetry = telemetry;
        frontRightMotor = new Motor(hwMap, "frontRight");
        frontLeftMotor = new Motor(hwMap, "frontLeft");
        backLeftMotor = new Motor(hwMap, "backLeft");
        backRightMotor = new Motor(hwMap, "backRight");
        // Set the run mode to RawPower
        frontLeftMotor.setRunMode(Motor.RunMode.RawPower);
        frontRightMotor.setRunMode(Motor.RunMode.RawPower);
        backLeftMotor.setRunMode(Motor.RunMode.RawPower);
        backRightMotor.setRunMode(Motor.RunMode.RawPower);
        // Set the zero power behavior to BRAKE
        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // Set the direction of the motors
        frontLeftMotor.setInverted(true);
        backLeftMotor.setInverted(true);
        frontRightMotor.setInverted(false);
        backRightMotor.setInverted(false);
        drivetrain = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

    }
    public void setFrontLeftMotor(double power) {
        frontLeftMotor.set(power);
    }
    public void setFrontRightMotor(double power) {
        frontRightMotor.set(power);
    }
    public void setBackLeftMotor(double power) {
        backLeftMotor.set(power);
    }
    public void setBackRightMotor(double power) {
        backRightMotor.set(power);
    }
    public void driveWithRotatePID(double forward, double right, double Commandedrotate) {
        desiredAngle += Commandedrotate* Constants.rotationAmplifier;
        rotateSpeed = rotatePID.calculate(getyaw(),desiredAngle);
        driveFieldCentric(forward,right,rotateSpeed);
        telemetry.addData("yaw",getyaw());
        telemetry.addData("desiredAngle",desiredAngle);
        telemetry.addData("PID",rotateSpeed);
        telemetry.update();
    }
    public void driveFieldCentric(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        driveRobotCentric(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void driveRobotCentric(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftMotor.set(maxSpeed * (frontLeftPower / maxPower));
        frontRightMotor.set(maxSpeed * (frontRightPower / maxPower));
        backLeftMotor.set(maxSpeed * (backLeftPower / maxPower));
        backRightMotor.set(maxSpeed * (backRightPower / maxPower));
    }
    public double getyaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public void resetGyro() {
        imu.resetYaw();
    }

}
