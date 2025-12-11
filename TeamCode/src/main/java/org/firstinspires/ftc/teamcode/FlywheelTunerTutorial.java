package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp // Registers this OpMode as a TeleOp.
@Disabled
public class FlywheelTunerTutorial extends OpMode {
    public DcMotorEx flywheelMotor;

    // --- Standardized Constants ---
    // The gear ratio is (teeth on driven gear) / (teeth on driving gear)
    public static final double GEAR_RATIO = 14.0 / 10.0; // 1.4
    // The motor's encoder counts (ticks) per revolution
    public static final double ENCODER_CPR = 28;

    // --- Target Velocities (in Ticks per Second) ---
    // This is the desired RPM of the FINAL shooter wheel
    public static final double HIGH_SHOT_RPM = 2035;
    public static final double LOW_SHOT_RPM = 100;

    // Correctly calculate the required motor velocity in ticks per second
    public double highVelocity = (HIGH_SHOT_RPM * GEAR_RATIO / 60) * ENCODER_CPR;
    public double lowVelocity = (LOW_SHOT_RPM * GEAR_RATIO / 60) * ENCODER_CPR;


    double curTargetVelocity = highVelocity;

    // Initial PIDF coefficients for tuning.
    double F = 14.098; // Feedforward gain to counteract constant forces like friction.
    double P = 265;    // Proportional gain to correct error based on how far off the velocity is.

    // Array of step sizes for making fine or coarse adjustments to P and F.
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    // Index to select the current step size from the array.
    int stepIndex = 1;


    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        // --- Gamepad Controls for Tuning ---

        // 'Y' button toggles the target velocity between the high and low presets.
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity; }
        }

        // 'B' button cycles through the different step sizes for tuning precision.
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; // Modulo wraps the index back to 0.
        }

        // D-pad left/right adjusts the F (Feedforward) gain.
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        // D-pad up/down adjusts the P (Proportional) gain.
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        // Apply the new coefficients to the motor in every loop iteration.
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Command the motor to run at the current target velocity.
        flywheelMotor.setVelocity(curTargetVelocity);

        // --- Telemetry Output ---

        // Correctly convert current motor velocity (ticks/sec) to actual wheel RPM
        double currentMotorVelocityTicks = flywheelMotor.getVelocity();
        double currentMotorRPM = (currentMotorVelocityTicks / ENCODER_CPR) * 60.0;
        double currentWheelRPM = currentMotorRPM / GEAR_RATIO;

        // Calculate error based on wheel RPM for intuitive tuning
        double targetWheelRPM = (curTargetVelocity == highVelocity) ? HIGH_SHOT_RPM : LOW_SHOT_RPM;
        double error = targetWheelRPM - currentWheelRPM;

        telemetry.addData("Target Wheel RPM", "%.0f", targetWheelRPM);
        telemetry.addData("Current Wheel RPM", "%.2f", currentWheelRPM);
        telemetry.addData("Error (RPM)", "%.2f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.addLine();
        telemetry.addData("Raw Target Velocity (Ticks/s)", "%.2f", curTargetVelocity);
        telemetry.addData("Raw Current Velocity (Ticks/s)", "%.2f", currentMotorVelocityTicks);
    }
}
