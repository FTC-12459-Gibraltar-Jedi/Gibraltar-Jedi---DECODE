package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter_Logic_Fast {

    private DcMotorSimple intake = null;
    private DcMotorEx shooter = null;
    private DcMotorSimple conveyor = null;
    private DcMotorSimple prelaunch = null;

    private ElapsedTime stateTimer = new ElapsedTime();




    private enum ShooterState {
        IDLE,
        SPIN_UP,
        SHOOT,
        RESET
    }
    private ShooterState shooterState;





    //---------- SHOOTER CONSTANTS -----------
    // Shooter constants
    // The gear ratio is (teeth on driven gear) / (teeth on driving gear) -> 14 / 10 = 1.4
    private static final double SHOOTER_GEAR_RATIO = 14.0 / 10.0;
    // This is the desired RPM of the FINAL shooter wheel.
    private static final double SHOOTER_WHEEL_TARGET_RPM = 2900;
    // To achieve the wheel RPM, the motor must spin faster by the gear ratio.
    private static final double MOTOR_TARGET_RPM = SHOOTER_WHEEL_TARGET_RPM * SHOOTER_GEAR_RATIO;
    // Counts Per Revolution for the motor encoder
    private static final double SHOOTER_CPR = 28;
    // The target velocity for the motor in "ticks per second" -> (Motor RPM / 60) * Ticks Per Revolution
    private static final double SHOOTER_TARGET_VELOCITY = (MOTOR_TARGET_RPM / 60.0) * SHOOTER_CPR;
    // Allow feeding when RPM is at 96% of target
    private static final double SHOOTER_VELOCITY_TOLERANCE = 0.96;
    private double SHOOTER_MAX_SPINUP_TIME = 3;
    private boolean shotComanded = false;



    public void init(HardwareMap hwMap) {
        shooter = hwMap.get(DcMotorEx.class, "Shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // TODO: These PIDF values are a starting point. You will need to tune them for your specific shooter.
        //  - Start by finding the correct F value, which is F = 32767 / max_ticks_per_second.
        //  - Then, increase P until you get oscillations, and then back it off.
        //  - D can then be used to dampen oscillations. I is usually not needed for velocity control.
        shooter.setVelocityPIDFCoefficients(435.0060, 0.0, 3.0, 15.1970);
        shooterState = shooterState.IDLE;
        intake = hwMap.get(DcMotorSimple.class,"Intake");
        conveyor = hwMap.get(DcMotorSimple.class,"Conveyor");
        prelaunch = hwMap.get(DcMotorSimple.class,"Prelaunch");
    }




    public void update() {
        switch (shooterState) {
            case IDLE:
                if (shotComanded) {
                    shooterState = shooterState.SPIN_UP;
                    stateTimer.reset();
                }
                break;
            case SPIN_UP:
                double currentShooterVelocity = shooter.getVelocity();
                double currentShooterRPM = (((shooter.getVelocity() / 60) / SHOOTER_CPR)/1.4);
                boolean shooterAtSpeed = currentShooterVelocity >= (SHOOTER_TARGET_VELOCITY * SHOOTER_VELOCITY_TOLERANCE);

                if (shooterAtSpeed || stateTimer.seconds() > SHOOTER_MAX_SPINUP_TIME) {
                    shooterState = shooterState.SHOOT;
                } else {
                    shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
                }
                break;
            case SHOOT:
                intake.setPower(1.0);
                conveyor.setPower(-1.0);
                prelaunch.setPower(1.0);

                if (stateTimer.seconds() > 4.5) {
                    shooterState = shooterState.RESET;
                }
                break;
            case RESET:
                intake.setPower(0.0);
                conveyor.setPower(0.0);
                prelaunch.setPower(0.0);
                shooter.setVelocity(0.0);
                shooterState = shooterState.IDLE;
                shotComanded = false;

        }
    }

    public void Shoot(boolean shoot) {
        if (shooterState.equals(shooterState.IDLE) && shoot) {
            shotComanded = true;
        } else {
            shotComanded = false;
        }

    }

    public boolean isBusy() {
        return shooterState != shooterState.IDLE;
    }

}
