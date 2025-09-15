package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PivotSubsystem extends SubsystemBase {
    private final Telemetry telemetry;
    public Motor pivotMotor;
    public Motor shooterMotor;
    public PIDController pivotPID;

    public PivotSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        pivotPID = new PIDController(0,0,0);
        pivotMotor = new Motor(hwMap, "turretMotor");
        pivotMotor.setRunMode(Motor.RunMode.VelocityControl);
        pivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooterMotor = new Motor(hwMap, "turretMotor");
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        shooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
    public void setTurretVeloFeedForward(double ks, double kv, double ka) {
        pivotMotor.setFeedforwardCoefficients(ks,kv,ka);
    }
    public void setTurretVeloFeedBack(double kp, double ki, double kd) {
        pivotMotor.setVeloCoefficients(kp,ki,kd);
    }
    public void setTurretPositionFeedBack(double kp, double ki, double kd) {
        pivotPID.setPID(kp,ki,kd);
    }
    public void goToTurretPosition(double position) {
        pivotMotor.setRunMode(Motor.RunMode.VelocityControl);
        pivotMotor.set(pivotPID.calculate(position));
    }
    public void setTurretSpeed(double power) {
        pivotMotor.setRunMode(Motor.RunMode.RawPower);
        pivotMotor.set(power);
    }
    public void setShooterVeloFeedForward(double ks, double kv, double ka) {
        pivotMotor.setFeedforwardCoefficients(ks,kv,ka);
    }
    public void setShooterVeloFeedBack(double kp, double ki, double kd) {
        pivotMotor.setVeloCoefficients(kp,ki,kd);
    }
    public void setShooterSpeed(double speed) {
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        shooterMotor.set(speed);
    }
}
