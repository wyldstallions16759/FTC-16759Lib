package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PivotSubsystem extends SubsystemBase {
    private final Telemetry telemetry;
    public ServoEx pivotServo;
    public Motor shooterMotor;
    public PIDController pivotPID;

    public PivotSubsystem(HardwareMap hwMap, Telemetry telemetry, Boolean isInverted) {
        this.telemetry = telemetry;
        pivotPID = new PIDController(0,0,0);
        pivotServo = new SimpleServo(hwMap,"pivotServo",0,90, AngleUnit.DEGREES);
        pivotServo.setInverted(isInverted);
        shooterMotor = new Motor(hwMap, "turretMotor");
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        shooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
    public void goToPivotPosition(double position) {
        pivotServo.turnToAngle(position,AngleUnit.DEGREES);
    }
    public void setShooterVeloFeedBack(double kp, double ki, double kd) {
        shooterMotor.setVeloCoefficients(kp,ki,kd);
    }
    public void setShooterSpeed(double speed) {
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        shooterMotor.set(speed);
    }
}
