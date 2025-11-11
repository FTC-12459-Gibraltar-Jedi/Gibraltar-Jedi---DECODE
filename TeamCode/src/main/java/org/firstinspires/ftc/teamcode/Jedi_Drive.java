package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.function.Supplier;

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// OP MODE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
@Configurable
@TeleOp
public class Jedi_Drive extends OpMode {

    // Pedro Pathing
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;

    // Telemetry
    private TelemetryManager telemetryM;

    // Hardware
    private Servo   encoderLift = null;
    private Servo   headLight = null;
    private Servo   rgbLight = null;
    private DcMotorSimple intake = null;
    private DcMotorSimple shooter = null;
    private DcMotorSimple conveyor = null;

    // Driving states
    private boolean driveMode = false;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Attachment states
    private boolean odometryUp = true;
    private boolean shooterOn = false;


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// INIT
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
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


        //-------------------
        //----INIT SERVOS----
        //-------------------
        encoderLift = hardwareMap.get(Servo.class, "Odometry");
        headLight =hardwareMap.get(Servo.class,"Headlight");
        rgbLight = hardwareMap.get(Servo.class,"RGBLight");
        encoderLift.setPosition(.65);
        headLight.setPosition(.75);
        rgbLight.setPosition(.475);


        //-------------------
        //----INIT MOTORS----
        //-------------------
        intake = hardwareMap.get(DcMotorSimple.class,"Intake");
        shooter = hardwareMap.get(DcMotorSimple.class,"Shooter");
        conveyor = hardwareMap.get(DcMotorSimple.class,"Conveyor");


    }


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// START
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// LOOP
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    @Override
    public void loop() {

        //Call this once per loop
        follower.update();
        telemetryM.update();


    //-------------------
    // --DRIVE CONTROLS--
    //-------------------
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x / 2,
                    driveMode // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * (slowModeMultiplier / 2),
                    driveMode // Robot Centric
            );
        }

        /*
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
        */

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

        // Change Drive Mode to Robot Centric
        if (gamepad1.backWasPressed()) {
            driveMode = !driveMode;
        }


    //------------------
    //--SERVO CONTROLS--
    //------------------

        // If DP-Up button is pressed on GP-1 toggle the odometryUp variable
        if (gamepad1.dpadUpWasPressed()) {
            odometryUp = !odometryUp;
        }
        // Set odometry position based on the toggled state
        encoderLift.setPosition(odometryUp ? 0.65 : 0);


    //------------------
    //--MOTOR CONTROLS--
    //------------------

        // If DP-Left button is pressed on GP-1 toggle the shooterOn variable
        shooterOn = gamepad2.right_trigger > 0.75;

        // Set shooter power based on the toggled state
        shooter.setPower(shooterOn ? -1.0 : 0.0);

        // Control Intake and Conveyor
        if (gamepad2.a) {
            // Run forward when 'a' is pressed
            intake.setPower(1.0);
            conveyor.setPower(-1.0);
        } else if (gamepad2.b) {
            // Run in reverse when 'b' is pressed
            intake.setPower(-1.0);
            conveyor.setPower(1.0);
        } else {
            // Turn off motors if neither 'a' nor 'b' is pressed
            intake.setPower(0.0);
            conveyor.setPower(0.0);
        }



    //------------------
    //--Loop telemetry--
    //------------------

        telemetry.addData("Slow Mode Multiplier", slowModeMultiplier);
        telemetry.addData("Slow Mode", slowMode);
        telemetry.addData("Current Servo Position", encoderLift.getPosition());

        if (odometryUp) {
            telemetry.addLine("Odometry UP");
        } else {
            telemetry.addLine("Odometry DOWN");

        }

        //This telemetry was here from the example pedro pathing code
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}
