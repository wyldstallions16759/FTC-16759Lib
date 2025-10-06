package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class VisionSubsystem {
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final Limelight3A limelight;
    private final IMU imu;
    private Pose3D visionPose;
    private GoBildaPinpointDriver poseEstimator;

    public VisionSubsystem(HardwareMap hwMap, Telemetry telemetry, GoBildaPinpointDriver poseEstimator) {
        this.telemetry = telemetry;
        this.hardwareMap = hwMap;
        this.poseEstimator = poseEstimator;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        poseEstimator.resetPosAndIMU();
    }

    public Pose3D getVisionPose3D(double turretAngle) {
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
    public Pose2D getRealPose(double turretAngle) {
        Pose3D visionPose3D = getVisionPose3D(turretAngle);
        Pose2D visionPose2D = new Pose2D(DistanceUnit.METER,visionPose.getPosition().x, visionPose.getPosition().y, AngleUnit.DEGREES,visionPose.getOrientation().getYaw(AngleUnit.DEGREES));
        if (imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate<100 && limelight.getTimeSinceLastUpdate()<0.1) {
            return visionPose2D;
        }
        return poseEstimator.getPosition();
    }
}
