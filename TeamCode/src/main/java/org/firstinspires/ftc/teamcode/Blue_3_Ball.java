package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Shooter_Logic_Slow;

@Autonomous(name = "BLUE 3 Ball Close", group = "BlueAutos")
public class Blue_3_Ball extends OpMode {
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
    private Shooter_Logic_Slow shoot = new Shooter_Logic_Slow();
    private boolean shotsTriggered = false;


    // ----- Autonomous State Machine ------
    public enum PathState   {
        // START POSITION_END POSITION
        // DRIVE > MOVING AROUND FIELD
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        drive_startPOS_shootPOS,
        shoot_preload,
        drive_shootPOS_EndPOS
    }
    PathState pathState;


    //------------ Declare Poses ---------------
    private final Pose startPose = new Pose(24,128, Math.toRadians(143));
    private final Pose shootPose = new Pose(33,120, Math.toRadians(143));
    private final Pose endPose = new Pose(47,138, Math.toRadians(180));


    //------------ Declare Paths ---------------
    private PathChain driveStartPosShootPosPath,
            driveShootPosEndPosPath;


    //------------- Build Paths ---------------
    public void buildPaths() {
        driveStartPosShootPosPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
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
                        follower.followPath(driveShootPosEndPosPath, true);
                        setPathState(PathState.drive_shootPOS_EndPOS);
                    }
                }
                break;
            case drive_shootPOS_EndPOS:
                if (!follower.isBusy()) {
                    telemetry.addLine("ALl Done");
                }
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


    @Override
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
        encoderLift.setPosition(0.0);
        linearActuator1.setPosition(0.5);
        linearActuator2.setPosition(0.5);
        headLight.setPosition(0.35);
        rgbLight.setPosition(0.47);
    }


    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }


    @Override
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
