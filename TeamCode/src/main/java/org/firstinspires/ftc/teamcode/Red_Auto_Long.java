package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Shooter_Logic;

public class Red_Auto_Long extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;


    // ------------ Hardware ---------------
    private Servo   encoderLift = null;
    private Servo   headLight = null;
    private Servo   rgbLight = null;
    private Servo   linearActuator1 = null;
    private Servo   linearActuator2 = null;
    private DcMotorSimple intake = null;
    private DcMotorSimple conveyor = null;
    private DcMotorSimple prelaunch = null;


    // ------------ Shooter ----------------
    private Shooter_Logic shoot = new Shooter_Logic();
    private boolean shotsTriggered = false;


    // Autonomous State Machine
    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVING AROUND FIELD
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        drive_startPOS_shootPOS,
        shoot_preload,
        drive_shootPOS_pickUp1POS,
        drive_pickUp1POS_pickUp1EndPOS,
        drive_pickUp1EndPOS_goToShootPOS,
        drive_goToShootPosShootPosPath,
        drive_shootPOS_EndPOS
    }
        PathState pathState;


    //------------ Declare Poses ---------------
    private final Pose startPose = new Pose(90, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose(82,82, Math.toRadians(46));
    private final Pose pickUp1Pose = new Pose(98,36, Math.toRadians(0));
    private final Pose pickUp1EndPose = new Pose(135,36, Math.toRadians(0));
    private final Pose goToShootPose = new Pose(98,36, Math.toRadians(111));
    private final Pose endPose = new Pose(90,50, Math.toRadians(0));


    //------------ Declare Paths ---------------
    private PathChain driveStartPosShootPosPath,
            driveShootPosPickUp1PosPath,
            drivePickUp1PosPickUp1EndPosPath,
            drivePickUp1EndPosGoToShootPosPath,
            driveGoToShootPosShootPosPath,
            driveShootPosEndPosPath;


    //------------- Build Paths ---------------
    public void buildPaths() {
        driveStartPosShootPosPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosPickUp1PosPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickUp1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickUp1Pose.getHeading())
                .build();
        drivePickUp1PosPickUp1EndPosPath = follower.pathBuilder()
                .addPath(new BezierLine(pickUp1Pose, pickUp1EndPose))
                .setLinearHeadingInterpolation(pickUp1Pose.getHeading(), pickUp1EndPose.getHeading())
                .build();
        drivePickUp1EndPosGoToShootPosPath = follower.pathBuilder()
                .addPath(new BezierLine(pickUp1EndPose, goToShootPose))
                .setLinearHeadingInterpolation(pickUp1EndPose.getHeading(), goToShootPose.getHeading())
                .build();
        driveGoToShootPosShootPosPath = follower.pathBuilder()
                .addPath(new BezierLine(goToShootPose, shootPose))
                .setLinearHeadingInterpolation(goToShootPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosEndPosPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }


    //------------- State Machine ---------------
    public void statePathUpdate() {
        switch (pathState) {
            case drive_startPOS_shootPOS:
                follower.followPath(driveStartPosShootPosPath, true);
                setPathState(PathState.shoot_preload);
                break;
            case shoot_preload:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shoot.Shoot(true);
                        shotsTriggered = true;

                    } else if (shotsTriggered && !shoot.isBusy()) {
                        follower.followPath(driveShootPosPickUp1PosPath, true);
                        setPathState(PathState.drive_shootPOS_pickUp1POS);
                    }
                }
                break;
            case drive_shootPOS_pickUp1POS:
                if (!follower.isBusy()) {
                    intake.setPower(1.0);
                    conveyor.setPower(-1.0);
                    prelaunch.setPower(-0.30);
                    follower.followPath(drivePickUp1PosPickUp1EndPosPath, true);
                    setPathState(PathState.drive_pickUp1POS_pickUp1EndPOS);
                }
                break;
            case drive_pickUp1POS_pickUp1EndPOS:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 2) {
                        follower.followPath(drivePickUp1EndPosGoToShootPosPath, true);
                        setPathState(pathState.drive_pickUp1EndPOS_goToShootPOS);
                    }
                }
                break;
            case drive_pickUp1EndPOS_goToShootPOS:
                if (!follower.isBusy()) {
                    follower.followPath(driveGoToShootPosShootPosPath, true);
                    setPathState(pathState.drive_goToShootPosShootPosPath);
                }
                break;
            case drive_goToShootPosShootPosPath:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shoot.Shoot(true);
                        shotsTriggered = true;

                    } else if (shotsTriggered && !shoot.isBusy()) {
                        follower.followPath(driveShootPosEndPosPath, true);
                        setPathState(PathState.drive_shootPOS_EndPOS);
                    }
                }
                break;
            case drive_shootPOS_EndPOS:
                if (!follower.isBusy()) {
                    telemetry.addLine("ALl Done");
                }
                break;
            default:
                telemetry.addLine("No state Commanded");
                break;
        }
    }


    //------- State Machine Helper Functions -------
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;
    }


    public void init() {
        pathState = pathState.drive_startPOS_shootPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        shoot.init(hardwareMap);
        intake = hardwareMap.get(DcMotorSimple.class,"Intake");
        conveyor = hardwareMap.get(DcMotorSimple.class,"Conveyor");
        prelaunch = hardwareMap.get(DcMotorSimple.class,"Prelaunch");
        linearActuator1 = hardwareMap.get(Servo.class,"Linear");
        linearActuator2 = hardwareMap.get(Servo.class,"Linear2");
        headLight = hardwareMap.get(Servo.class,"Headlight");
        rgbLight = hardwareMap.get(Servo.class,"RGBLight");
        buildPaths();
        follower.setPose(startPose);
        encoderLift = hardwareMap.get(Servo.class,"Odometry");
        encoderLift.setPosition(0.5);
        linearActuator1.setPosition(0.5);
        linearActuator2.setPosition(0.5);
        headLight.setPosition(0.35);
        rgbLight.setPosition(0.47);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    public void loop() {
        follower.update();
        shoot.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("time", opModeTimer.getElapsedTime());
        telemetry.update();
    }
}
