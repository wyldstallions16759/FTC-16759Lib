package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem extends SubsystemBase {
    private final Telemetry telemetry;
    public Motor turretMotor;
    public PIDController turretPID;

    public TurretSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        turretPID = new PIDController(0,0,0);
        turretMotor = new Motor(hwMap, "turretMotor");
        turretMotor.setRunMode(Motor.RunMode.VelocityControl);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
    public void setVeloFeedForward(double ks, double kv, double ka) {
        turretMotor.setFeedforwardCoefficients(ks,kv,ka);
    }
    public void setVeloFeedBack(double kp, double ki, double kd) {
        turretMotor.setVeloCoefficients(kp,ki,kd);
    }
    public void setPositionFeedBack(double kp, double ki, double kd) {
        turretPID.setPID(kp,ki,kd);
    }
    public void goToPosition(double position) {
        turretMotor.setRunMode(Motor.RunMode.VelocityControl);
        turretMotor.set(turretPID.calculate(position));
    }
    public void setTurretSpeed(double power) {
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.set(power);
    }

}
