package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final Telemetry telemetry;
    public Motor intakeMotor;
    public ServoEx firstDoor;
    public ServoEx secondDoor;
    public ServoEx thirdDoor;
    public ColorSensor firstColorSensor;
    public ColorSensor secondColorSensor;
    public ColorSensor thirdColorSensor;

    public IntakeSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = new Motor(hwMap, "intakeMotor");
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    }
    public void setIntakeMotor(double power) {
        intakeMotor.set(power);
    }

}
