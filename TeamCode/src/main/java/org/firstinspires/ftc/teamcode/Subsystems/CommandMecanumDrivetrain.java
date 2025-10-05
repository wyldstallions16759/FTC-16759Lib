package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class CommandMecanumDrivetrain extends SubsystemBase {
    private final Telemetry telemetry;
    public Motor frontRightMotor;
    public Motor frontLeftMotor;
    public Motor backLeftMotor;
    public Motor backRightMotor;
    public MecanumDrive drivetrain;
    public GoBildaPinpointDriver odo;

    public CommandMecanumDrivetrain(HardwareMap hwMap, Telemetry telemetry) {
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
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
    public void driveRobotCentric(double x, double y, double rotation) {
        drivetrain.driveRobotCentric(x, y, rotation);
    }
    public void driveFieldCentric(double x, double y, double turnSpeed) {
        drivetrain.driveFieldCentric(x, y, turnSpeed, odo.getHeading(AngleUnit.DEGREES), false);
    }
    public void resetGyro() {
        odo.resetPosAndIMU();
    }

}
