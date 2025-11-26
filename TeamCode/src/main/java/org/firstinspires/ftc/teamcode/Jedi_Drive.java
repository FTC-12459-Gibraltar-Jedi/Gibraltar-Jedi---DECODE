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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    // Shooter constants
    private static final double SHOOTER_RPM = 2850;
    private static final double SHOOTER_CPR = 28; // Counts Per Revolution for the motor encoder
    private static final double SHOOTER_TARGET_VELOCITY = (SHOOTER_RPM * SHOOTER_CPR) / 60;
    private static final double SHOOTER_VELOCITY_TOLERANCE = 0.96; // Allow feeding when RPM is at 98% of target

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
    private Servo   linearActuator = null;
    private DcMotorSimple intake = null;
    private DcMotorEx shooter = null;
    private DcMotorSimple conveyor = null;
    private DcMotorSimple prelaunch = null;

    // Driving states
    private boolean driveMode = false;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.3;

    // Attachment states
    private boolean odometryUp = true;
    
    // Launch sequence states
    private boolean launchSequenceActive = false;

    private ElapsedTime lightFlash = new ElapsedTime();
    private boolean isEndGame;


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
        linearActuator = hardwareMap.get(Servo.class,"Linear");

        encoderLift.setPosition(.65);
        headLight.setPosition(0.35);
        rgbLight.setPosition(0.47);
        linearActuator.setPosition(0.0);


        //-------------------
        //----INIT MOTORS----
        //-------------------
        intake = hardwareMap.get(DcMotorSimple.class,"Intake");
        conveyor = hardwareMap.get(DcMotorSimple.class,"Conveyor");
        prelaunch = hardwareMap.get(DcMotorSimple.class,"Prelaunch");

        // Shooter motor setup for velocity control
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // TODO: These PIDF values are a starting point. You will need to tune them for your specific shooter.
        //  - Start by finding the correct F value, which is F = 32767 / max_ticks_per_second.
        //  - Then, increase P until you get oscillations, and then back it off.
        //  - D can then be used to dampen oscillations. I is usually not needed for velocity control.
        shooter.setVelocityPIDFCoefficients(100.0, 0.0, 3.0, 17.245);
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

        if ((getRuntime() >= 99) && !isEndGame){
            gamepad1.rumbleBlips(5);
            isEndGame = true;
        }


    //-------------------
    // --DRIVE CONTROLS--
    //-------------------

        if (gamepad1.y) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }


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
        slowMode = (gamepad1.left_trigger > 0.5);

        /*
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
        if (gamepad1.dpad_right) {
            driveMode = !driveMode;
        }
        */

    //------------------
    //--SERVO CONTROLS--
    //------------------

        // If DP-Up button is pressed on GP-1 toggle the odometryUp variable
        if (gamepad1.dpadUpWasPressed()) {
            odometryUp = !odometryUp;
        }
        // Set odometry position based on the toggled state
        encoderLift.setPosition(odometryUp ? 0.65 : 0);

        if (gamepad2.dpad_up) {
            linearActuator.setPosition(0.9);
        } else if (gamepad2.dpad_right) {
            linearActuator.setPosition(0.75);
        } else if (gamepad2.dpad_left) {
            linearActuator.setPosition(0.50);
        } else if (gamepad2.dpad_down) {
            linearActuator.setPosition(0.1);
        }

    //-------------------
    //---RGB INDICATOR---
    //-------------------

    if (launchSequenceActive || (isEndGame && getRuntime() < 102)) {
        // Create a flash effect by casting the timer result to a whole number before the modulo
        if (((long) (lightFlash.milliseconds() / 250)) % 2 == 0) {
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
        headLight.setPosition(0.35);
    }



    //---------------------------
    //--LAUNCHER MOTOR CONTROLS--
    //---------------------------

        // --- AUTOMATED LAUNCH SEQUENCE (RPM-BASED) ---
        boolean launchTriggerPressed = gamepad2.right_trigger > 0.50;
        double currentShooterVelocity = shooter.getVelocity();
        boolean shooterAtSpeed = currentShooterVelocity >= (SHOOTER_TARGET_VELOCITY * SHOOTER_VELOCITY_TOLERANCE);

        // Default motor powers/velocities
        double shooterTargetVelocity = 0.0;
        double intakePower = 0.0;
        double conveyorPower = 0.0;
        double prelaunchPower = 0.0;

        if (launchTriggerPressed) {
            launchSequenceActive = true;
            // Command the shooter to spin up to the target velocity
            shooterTargetVelocity = SHOOTER_TARGET_VELOCITY;

            // If the shooter is at the required speed, run the feeder motors
            if (shooterAtSpeed) {
                intakePower = 1.0;
                conveyorPower = -1.0;
                prelaunchPower = 1.0;
            }
        } else {
            launchSequenceActive = false;
            // All motors stop when the trigger is released, so powers remain 0.0
        }

        // --- MANUAL REVERSE (for clearing jams) ---
        if (gamepad2.y) {
            intakePower = -1.0;
            conveyorPower = 1.0;
            prelaunchPower = 0.0;
            shooterTargetVelocity = 0.0; // Stop shooter during reverse
            launchSequenceActive = false; // Ensure sequence stops
        }

        // --- SET FINAL MOTOR POWERS ---
        shooter.setVelocity(shooterTargetVelocity);
        intake.setPower(intakePower);
        conveyor.setPower(conveyorPower);
        prelaunch.setPower(prelaunchPower);


    //--------------------------
    //--GENERAL MOTOR CONTROLS--
    //--------------------------

        // Run Intake forward with Driver 1 or Driver 2
        if (gamepad2.a || (gamepad1.right_trigger > 0.50)) {
            intake.setPower(1.0);
            conveyor.setPower(-1.0);
            prelaunch.setPower(-0.30);
        }


    //------------------
    //--Loop telemetry--
    //------------------
        double currentShooterRPM = (shooter.getVelocity() * 60) / SHOOTER_CPR;
        telemetry.addData("Shooter Target RPM", SHOOTER_RPM);
        telemetry.addData("Shooter Current RPM", currentShooterRPM);
        telemetry.addData("Shooter At Speed", shooterAtSpeed);

        telemetry.addData("Slow Mode Multiplier", slowModeMultiplier);
        telemetry.addData("Slow Mode", slowMode);
        telemetry.addData("Current Servo Position", encoderLift.getPosition());
        telemetry.addData("Launch Sequence Active", launchSequenceActive);

        if (odometryUp) {
            telemetry.addLine("Odometry UP");
        } else {
            telemetry.addLine("Odometry DOWN");
        }

        telemetry.addData("Run Time", getRuntime());

        //This telemetry was here from the example pedro pathing code
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}
