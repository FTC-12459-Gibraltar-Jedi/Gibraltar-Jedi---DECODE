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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
public class Jedi_Drive_Red extends OpMode {

    // --- Shooter Constants ---
    // Gear ratio: (teeth on driven gear) / (teeth on driving gear)
    private static final double SHOOTER_GEAR_RATIO = 14.0 / 10.0;
    // Motor encoder counts per revolution
    private static final double SHOOTER_CPR = 28;
    // Velocity tolerance (allow feeding when RPM is at 96% of target)
    private static final double SHOOTER_VELOCITY_TOLERANCE = 0.96;

    // --- High-Speed (Truss) Shot Settings ---
    private static final double TRUSS_SHOT_RPM = 2900;
    private static final double TRUSS_SHOT_MOTOR_RPM = TRUSS_SHOT_RPM * SHOOTER_GEAR_RATIO;
    private static final double TRUSS_SHOT_VELOCITY = (TRUSS_SHOT_MOTOR_RPM / 60.0) * SHOOTER_CPR;
    private static final double TRUSS_SHOT_ACTUATOR_POS = 0.1;

    // --- Low-Speed (Subwoofer) Shot Settings ---
    private static final double SUBWOOFER_SHOT_RPM = 2035;
    private static final double SUBWOOFER_SHOT_MOTOR_RPM = SUBWOOFER_SHOT_RPM * SHOOTER_GEAR_RATIO;
    private static final double SUBWOOFER_SHOT_VELOCITY = (SUBWOOFER_SHOT_MOTOR_RPM / 60.0) * SHOOTER_CPR;
    private static final double SUBWOOFER_SHOT_ACTUATOR_POS = 0.5;


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
    private Servo   linearActuator1 = null;
    private Servo   linearActuator2 = null;
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
    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean isEndGame;

    // Limelight aiming
    private Limelight3A limelight;
    private boolean isAiming;
    private static final double ROTATIONAL_kP = -0.012;


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
        //-----LIMELIGHT-----
        //-------------------

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(1);


        //-------------------
        //----INIT SERVOS----
        //-------------------
        encoderLift = hardwareMap.get(Servo.class, "Odometry");
        headLight =hardwareMap.get(Servo.class,"Headlight");
        rgbLight = hardwareMap.get(Servo.class,"RGBLight");
        linearActuator1 = hardwareMap.get(Servo.class,"Linear");
        linearActuator2 = hardwareMap.get(Servo.class,"Linear2");

        encoderLift.setPosition(.65);
        headLight.setPosition(0.35);
        rgbLight.setPosition(0.47);
        linearActuator1.setPosition(0.1);
        linearActuator2.setPosition(0.1);



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
        limelight.start();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.addLine("GOOD LUCK JEDI!");
        telemetry.update();
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// LOOP
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    @Override
    public void loop() {

        //Call this once per loop
        follower.update();
        telemetryM.update();

        // When TeleOP time is over 100 seconds, Rumble Gamepad1
        if ((getRuntime() >= 99) && !isEndGame){
            gamepad1.rumbleBlips(5);
            isEndGame = true;
        }


        //-------------------
        // --DRIVE CONTROLS--
        //-------------------

        if (gamepad2.left_trigger > 0.50) {
            isAiming = true;
            telemetry.addData("isAiming", isAiming);
        } else {
            isAiming = false;
            telemetry.addData("isAiming", isAiming);
        }


        // General Limelight testing
        LLResult result = limelight.getLatestResult();

        telemetry.addData("tx", result.getTx());
        telemetry.addData("txnc", result.getTxNC());
        telemetry.addData("ty", result.getTy());
        telemetry.addData("tync", result.getTyNC());


        // Pressing Y on gamepad 1 sets a new pose in the follower causing the heading to reset
        if (gamepad1.y) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }

        /*
        // Change Drive Mode to Robot Centric
        if (gamepad1.dpad_right) {
            driveMode = !driveMode;
        }
         */

        // Slow Mode
        slowMode = (gamepad1.left_trigger > 0.5);

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (isAiming && result.isValid()) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    limelight.getLatestResult().getTx() * ROTATIONAL_kP,
                    driveMode // Robot Centric is true
            );

            else if (!slowMode) follower.setTeleOpDrive(
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

        //------------------
        //--SERVO CONTROLS--
        //------------------

        /*
        // If DP-Up button is pressed on GP-1 toggle the odometryUp variable
        if (gamepad1.dpad_right) {
            odometryUp = !odometryUp;
        }
         */

        // Set odometry position based on the toggled state
        encoderLift.setPosition(odometryUp ? 0.65 : 0);

        // This manual control is now overridden by the shooter logic, but can be kept for testing
        if (gamepad2.dpad_up) {
            linearActuator1.setPosition(0.50);
            linearActuator2.setPosition(0.50);
        } else if (gamepad2.dpad_down) {
            linearActuator1.setPosition(0.1);
            linearActuator2.setPosition(0.1);
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

        boolean longShotTrigger = gamepad2.right_trigger > 0.50;
        boolean shortShotTrigger = gamepad2.right_bumper;

        double currentShooterVelocity = shooter.getVelocity();
        double shooterTargetVelocity = 0.0;
        double intakePower = 0.0;
        double conveyorPower = 0.0;
        double prelaunchPower = 0.0;
        boolean shooterAtSpeed = false;

        if (longShotTrigger) {
            launchSequenceActive = true;
            shooterTargetVelocity = TRUSS_SHOT_VELOCITY;
            linearActuator1.setPosition(TRUSS_SHOT_ACTUATOR_POS);
            linearActuator2.setPosition(TRUSS_SHOT_ACTUATOR_POS);

            shooterAtSpeed = currentShooterVelocity >= (TRUSS_SHOT_VELOCITY * SHOOTER_VELOCITY_TOLERANCE);
            if (shooterAtSpeed || (shootTimer.seconds() >= 3.0)) {
                intakePower = 1.0;
                conveyorPower = -1.0;
                prelaunchPower = 1.0;
            }
        } else if (shortShotTrigger) {
            launchSequenceActive = true;
            shooterTargetVelocity = SUBWOOFER_SHOT_VELOCITY;
            linearActuator1.setPosition(SUBWOOFER_SHOT_ACTUATOR_POS);
            linearActuator2.setPosition(SUBWOOFER_SHOT_ACTUATOR_POS);

            shooterAtSpeed = currentShooterVelocity >= (SUBWOOFER_SHOT_VELOCITY * SHOOTER_VELOCITY_TOLERANCE);
            if (shooterAtSpeed || (shootTimer.seconds() >= 3.0)) {
                intakePower = 1.0;
                conveyorPower = -1.0;
                prelaunchPower = 1.0;
            }
        } else {
            launchSequenceActive = false;
            shootTimer.reset();
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
        if ((gamepad2.a || (gamepad1.right_trigger > 0.50)) && !launchSequenceActive) {
            intake.setPower(1.0);
            conveyor.setPower(-1.0);
            prelaunch.setPower(-0.30);
        }


        //------------------
        //--Loop telemetry--
        //------------------
        // Correctly convert motor velocity (ticks/sec) to shooter wheel RPM
        double currentMotorRPM = (shooter.getVelocity() / SHOOTER_CPR) * 60.0;
        double currentShooterRPM = currentMotorRPM / SHOOTER_GEAR_RATIO;

        if(launchSequenceActive) {
            if (longShotTrigger) {
                telemetry.addData("Shooter Target RPM", (int) TRUSS_SHOT_RPM);
            } else if (shortShotTrigger) {
                telemetry.addData("Shooter Target RPM", (int) SUBWOOFER_SHOT_RPM);
            }
        } else {
            telemetry.addData("Shooter Target RPM", 0);
        }
        telemetry.addData("Current Shooter RPM", (int)currentShooterRPM);
        telemetry.addData("Shooter At Speed", shooterAtSpeed);
        telemetry.update();
    }
}

