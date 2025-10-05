package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VisionSS {
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final Limelight3A limelight;

    public VisionSS(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hwMap;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
    }
}
