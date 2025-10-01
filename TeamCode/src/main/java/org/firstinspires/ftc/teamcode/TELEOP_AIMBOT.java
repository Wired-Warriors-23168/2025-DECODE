package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp
public class TELEOP_AIMBOT extends LinearOpMode {
//TODO ************* THIS IS BASED ON THE TELEOP FROM THE 6TH GRADE BOT, UPDATE IT FOR THE 23168 BOT!!!!

    // Declare OpMode members.
    private DcMotor flywheel;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private Limelight3A limelight;
    private CRServo launcherPurple;
    private CRServo launcherGreen;

    /////////////////////////////////////////////////////////////////////////
    // Declare variables
    public double txLimelight;
    public double tyLimelight;


    // Setting our velocity targets. These values are in ticks per second!
    private static final int bankVelocity = 1300;
    private static final int farVelocity = 1900;
    private static final int maxVelocity = 2200;


    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotor.class, "motor-flywheel");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left-front-drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left-back-drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right-front-drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right-back-drive");
        launcherPurple = hardwareMap.get(CRServo.class, "servo-launcher-P");
        launcherGreen = hardwareMap.get(CRServo.class, "servo-launcher-G");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");

        // Establishing the direction and mode for the motors
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //Ensures the launchers are active and ready
        launcherPurple.setPower(0);
        launcherGreen.setPower(0);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our methods while the OpMode is running
                splitStickArcadeDrive();
                setFlywheelVelocity();
                manualLauncherControl();
                aimBot();
                telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
                telemetry.addData("Flywheel Power", flywheel.getPower());
                telemetry.update();
            }
        }
    }



    /**
     * Controls for the drivetrain. The robot uses a mecanum drivetrain.
     * Forward and back is on the left stick. Strafing is on the left stick.  Turning is on the right stick.
     *Code and explanation are at: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
     */
    private void splitStickArcadeDrive() {
        double x;
        double y;
        double rx;

        x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    /**
     * Manual control for the Core Hex powered feeder and the agitator servo in the hopper
     */
    private void manualLauncherControl() {
        // Manual control for the Purple Launcher
        if (gamepad1.cross) {
            launcherPurple.setPower(0.5);
        } else if (gamepad1.triangle) {
            launcherPurple.setPower(-0.5);
        }
        // Manual control for the Green Launcher
        if (gamepad1.dpad_left) {
            launcherGreen.setPower(1);
        } else if (gamepad1.dpad_right) {
            launcherGreen.setPower(-1);
        }
    }

    /**
     * This if/else statement contains the controls for the flywheel, both manual and auto.
     * Circle and Square will spin up ONLY the flywheel to the target velocity set.
     * The bumpers will activate the flywheel, Core Hex feeder, and servo to cycle a series of balls.
     */
    private void setFlywheelVelocity() {
        if (gamepad1.options) {
            flywheel.setPower(-0.5);
        } else if (gamepad1.left_bumper) {
            farPowerAuto();
        } else if (gamepad1.right_bumper) {
            bankShotAuto();
        } else if (gamepad1.circle) {
            ((DcMotorEx) flywheel).setVelocity(bankVelocity);
        } else if (gamepad1.square) {
            ((DcMotorEx) flywheel).setVelocity(maxVelocity);
        } else {
            ((DcMotorEx) flywheel).setVelocity(0);
            launcherGreen.setPower(0);
            launcherPurple.setPower(0);
        }
    }

    /**
     * The bank shot or near velocity is intended for launching balls touching or a few inches from the goal.
     * When running this function, the flywheel will spin up and the launcher servo will wait before balls can be fed.
     */
    private void bankShotAuto() {
        ((DcMotorEx) flywheel).setVelocity(bankVelocity);
        if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 50) {
            launcherGreen.setPower(1);
            launcherPurple.setPower(1);
        } else {
            launcherGreen.setPower(0);
            launcherPurple.setPower(0);
        }
    }

    /**
     * The far power velocity is intended for launching balls a few feet from the goal. It may require adjusting the deflector.
     * When running this function, the flywheel will spin up and the launcher servo will wait before balls can be fed.
     */
    private void farPowerAuto() {
        ((DcMotorEx) flywheel).setVelocity(farVelocity);
        if (((DcMotorEx) flywheel).getVelocity() >= farVelocity - 100) {
            launcherGreen.setPower(1);
            launcherPurple.setPower(1);
        } else {
            launcherGreen.setPower(0);
            launcherPurple.setPower(0);
        }
    }

    /**
     * The aimbot function reads the txLimelight from the limelight and uses that to turn the bot to aim at the correct target
     * txLimelight and tyLimelight (if needed) should be read from the correct pipeline from the Limelight3A
     * This function should be triggered manually by holding a button and it should stop moving when the bot it aimed or the button is released.
     */

    private void aimBot() {
    //run if a button is held
    }


}
