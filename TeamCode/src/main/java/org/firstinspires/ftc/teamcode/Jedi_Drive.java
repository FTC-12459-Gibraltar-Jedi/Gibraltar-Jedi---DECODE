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
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private DcMotorSimple prelaunch = null;

    // Driving states
    private boolean driveMode = true;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Attachment states
    private boolean odometryUp = true;
    
    // Launch sequence states
    private ElapsedTime launchTimer;
    private boolean launchSequenceActive = false;


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
        headLight.setPosition(0.35);
        rgbLight.setPosition(0.47);


        //-------------------
        //----INIT MOTORS----
        //-------------------
        intake = hardwareMap.get(DcMotorSimple.class,"Intake");
        shooter = hardwareMap.get(DcMotorSimple.class,"Shooter");
        conveyor = hardwareMap.get(DcMotorSimple.class,"Conveyor");
        prelaunch = hardwareMap.get(DcMotorSimple.class,"Prelaunch");

        // Init launch timer
        launchTimer = new ElapsedTime();
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
                    -gamepad1.right_stick_x * (slowModeMultiplier / 1.5),
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
        slowMode = gamepad1.left_bumper;

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


    //-------------------
    //-- RGB INDICATOR --
    //-------------------
    if (launchSequenceActive) {
        // Create a flash effect by casting the timer result to a whole number before the modulo
        if (((long) (launchTimer.milliseconds() / 250)) % 2 == 0) {
            rgbLight.setPosition(0.28); // Red
            headLight.setPosition(0.3);
        } else {
            // Lights off for the rest of the time
            rgbLight.setPosition(0.0); // Green (or off)
            headLight.setPosition(0.0);
        }
    } else {
        // When not launching, keep the light solid green
        rgbLight.setPosition(0.47);
        headLight.setPosition(0.3);
    }


    //---------------------------
    //--LAUNCHER MOTOR CONTROLS--
    //---------------------------

        // --- AUTOMATED LAUNCH SEQUENCE ---
        boolean launchTriggerPressed = gamepad2.right_trigger > 0.75;

        // Default motor powers
        double shooterPower = 0.0;
        double intakePower = 0.0;
        double conveyorPower = 0.0;
        double prelaunchPower = 0.0;

        // Check if the trigger is pressed to start or continue the sequence
        if (launchTriggerPressed) {
            // If this is the first moment the trigger is pressed, reset the timer
            if (!launchSequenceActive) {
                launchSequenceActive = true;
                launchTimer.reset();
            }

            // Shooter spins up immediately
            shooterPower = -1.0;

            // After 2 seconds, also run the feeder motors
            if (launchTimer.seconds() > 1.75) {
                intakePower = 1.0;
                conveyorPower = -1.0;
                prelaunchPower = 1.0;
            }
        } else {
            // If the trigger is not pressed, the sequence is not active
            launchSequenceActive = false;
        }

        // --- MANUAL REVERSE (for clearing jams) ---
        // This overrides the launch sequence if 'b' is pressed.
        if (gamepad2.y) {
            intakePower = -1.0;
            conveyorPower = 1.0;
            prelaunchPower = 0.0; // Make sure prelaunch doesn't run in reverse
            shooterPower = 0.0;   // Stop shooter during reverse
        }

        // --- SET FINAL MOTOR POWERS ---
        shooter.setPower(shooterPower);
        intake.setPower(intakePower);
        conveyor.setPower(conveyorPower);
        prelaunch.setPower(prelaunchPower);


    //--------------------------
    //--GENERAL MOTOR CONTROLS--
    //--------------------------

        // Run Intake forward with Driver 1 or Driver 2
        if (gamepad2.a || gamepad1.right_bumper) {
            intake.setPower(1.0);
            conveyor.setPower(-1.0);
            prelaunch.setPower(-0.30);
        }


    //------------------
    //--Loop telemetry--
    //------------------

        telemetry.addData("Slow Mode Multiplier", slowModeMultiplier);
        telemetry.addData("Slow Mode", slowMode);
        telemetry.addData("Current Servo Position", encoderLift.getPosition());
        telemetry.addData("Launch Sequence Active", launchSequenceActive);

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
