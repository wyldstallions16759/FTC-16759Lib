package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.CommandMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSS;

public class Superstructure {
    private CommandMecanumDrivetrain drivetrain;
    private IntakeSubsystem IntakeSS;
    private PivotSubsystem pivotSS;
    private TurretSubsystem turretSS;
    private VisionSS visionSS;
    private Pose3D visionPose;
    private enum TurretState {
        TRACK_YES_TAG,
        TRACK_NO_TAG,
        GO_TO_POSE,
        SET_SPEEDS,
        NONE
    }
    public enum Alliance {
        RED,
        BLUE
    }
    private TurretState turretState;
    private Alliance alliance;

    public Superstructure(CommandMecanumDrivetrain drivetrain, IntakeSubsystem IntakeSS, PivotSubsystem pivotSS, TurretSubsystem turretSS, VisionSS visionSS) {
        this.drivetrain = drivetrain;
        this.IntakeSS = IntakeSS;
        this.pivotSS = pivotSS;
        this.turretSS = turretSS;
        this.visionSS = visionSS;
        turretState = TurretState.TRACK_NO_TAG;
    }
    public void getTranslationToGoal(Alliance alliance) {
        if (alliance == Alliance.RED) {

        } else {
            //
        }
    }
    public void execute() {
        visionPose = visionSS.get3DPose(turretSS.getTurretPose()*Constants.turretAngleMultiplier);
        switch (turretState) {
            case TRACK_YES_TAG:
                break;
            case TRACK_NO_TAG:
                break;
            case GO_TO_POSE:
                break;
            case SET_SPEEDS:
                break;
            case NONE:
                break;
        }
    }
}
