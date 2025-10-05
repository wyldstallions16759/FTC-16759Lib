package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class VisionSS {
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final Limelight3A limelight;
    private final IMU imu;
    private Pose3D visionPose;

    public VisionSS(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hwMap;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
    }
    public Pose3D get3DPose(double turretAngle) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES)+turretAngle);
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                visionPose = result.getBotpose_MT2();
            }
        }
        return visionPose;
    }
}
