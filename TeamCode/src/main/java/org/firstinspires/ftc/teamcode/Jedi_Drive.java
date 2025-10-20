package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class Jedi_Drive extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;


    private Servo   encoderLift;
    private double servoPosition = 0.10;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        encoderLift = hardwareMap.get(Servo.class, "Servo1");
        encoderLift.setPosition(0);

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            double turningMultiplier = 0.5;
            if (!slowMode) follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x * turningMultiplier,
                    false // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    gamepad1.left_stick_y * slowModeMultiplier,
                    gamepad1.left_stick_x * slowModeMultiplier,
                    gamepad1.right_stick_x * slowModeMultiplier * turningMultiplier,
                    false // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        slowMode = gamepad1.right_bumper;

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            // Decrease multiplier, but not below 0.1
            slowModeMultiplier = Math.max(0.1, slowModeMultiplier - 0.1);
        }

        //Optional way to change slow mode strength
        if (gamepad1.yWasPressed()) {
            // Increase multiplier, but not above 0.7
            slowModeMultiplier = Math.min(0.7, slowModeMultiplier + 0.1);
        }



        //Servos - This section handles all servo movements in TeleOp

        /*
        Odometry pod lift servo:
        This servo lifts up the Odometry pods at the start of Teleop.
        During INIT the servo is set to position 0
        */

        // If DP-Up button is pressed on GP-1 move servo to 0
        if (gamepad1.dpadUpWasPressed()) {
            encoderLift.setPosition(0);
        }

        // If DP-Down button is pressed on GP-1 move servo to servoPosition variable
        if (gamepad1.dpadDownWasPressed()) {
            encoderLift.setPosition(servoPosition);
        }

        // If DP-Left button is pressed on GP-1 decrease servoPosition variable by 0.01
        if (gamepad1.dpadLeftWasPressed()) {
            servoPosition = Math.max(0.1, servoPosition - 0.01);
        }

        // If DP-Right button is pressed on GP-1 increase servoPosition variable by 0.01
        if (gamepad1.dpadRightWasPressed()) {
            servoPosition = Math.min(0.5, servoPosition + 0.01);
        }




        //This section handles all custom telemetry
        telemetry.addData("Slow Mode Multiplier", slowModeMultiplier);
        telemetry.addData("Slow Mode", slowMode);
        telemetry.addData("Servo Set Position", servoPosition);
        telemetry.addData("Current Servo Position", encoderLift.getPosition());

        //This telemetry was here from the example pedro pathing code
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}
