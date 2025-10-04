package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp
public class TELEOP_MAIN extends LinearOpMode {
//TODO ************* THIS IS THE TELEOP FROM THE 6TH GRADE BOT, UPDATE IT FOR THE 23168 BOT!!!!
    private DcMotor flywheel;
    private DcMotor feeder;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private CRServo agitator;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private RevBlinkinLedDriver lightsLED;

    // Setting our velocity targets. These values are in ticks per second!
    private static final int bankVelocity = 1300;
    private static final int farVelocity = 1900;
    private static final int maxVelocity = 2200;
    private static final double limitDrivePower = 0.5;  //Mutliplier to limit the drive wheel power for training
    public static final String ALLIANCE_KEY = "Alliance";

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotor.class, "motor-flywheel");
        feeder = hardwareMap.get(DcMotor.class, "motor-feeder");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left-front-drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left-back-drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right-front-drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right-back-drive");
        agitator = hardwareMap.get(CRServo.class, "servo-agitator");
        lightsLED = hardwareMap.get(RevBlinkinLedDriver.class,"pwm-LED");

        // Establishing the direction and mode for the motors
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //Initialize actuators, sensors, and variables
        agitator.setPower(0);
        blackboard.get(ALLIANCE_KEY); //Get the Alliance from the blackboard (it was stored in the AUTO mode), either BLUE or RED

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our methods while the OpMode is running
                splitStickArcadeDrive();
                setFlywheelVelocity();
                manualFeederAndagitatorControl();
                telemetry.addData("Alliance", blackboard.get(ALLIANCE_KEY));
                telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
                telemetry.addData("Flywheel Power", flywheel.getPower());
                telemetry.update();
            }
        }
    }

    /**
     * Controls for the drivetrain. The robot uses a mecanum drivetrain.
     * Forward and back is on the left stick. Strafing is on the left stick.  Turning is on the right stick.
     * Code and explanation are at: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
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
        double frontLeftPower = limitDrivePower * (y + x + rx) / denominator;
        double backLeftPower = limitDrivePower * (y - x + rx) / denominator;
        double frontRightPower = limitDrivePower * (y - x - rx) / denominator;
        double backRightPower = limitDrivePower * (y + x - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    /**
     * Manual control for the Core Hex powered feeder and the agitator servo in the hopper
     */
    private void manualFeederAndagitatorControl() {
        // Manual control for the Core Hex agitator
        if (gamepad1.cross) {
            feeder.setPower(0.5);
        } else if (gamepad1.triangle) {
            feeder.setPower(-0.5);
        }
        // Manual control for the hopper's servo
        if (gamepad1.dpad_left) {
            agitator.setPower(1);
        } else if (gamepad1.dpad_right) {
            agitator.setPower(-1);
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
            feeder.setPower(0);
            // The check below is in place to prevent stuttering with the agitator. It checks if the agitator is under manual control!
            if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
                agitator.setPower(0);
            }
        }
    }

    /**
     * The bank shot or near velocity is intended for launching balls touching or a few inches from the goal.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The agitator will spin until the bumper is released.
     */
    private void bankShotAuto() {
        ((DcMotorEx) flywheel).setVelocity(bankVelocity);
        agitator.setPower(-1);
        if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 50) {
            feeder.setPower(1);
        } else {
            feeder.setPower(0);
        }
    }

    /**
     * The far power velocity is intended for launching balls a few feet from the goal. It may require adjusting the deflector.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The agitator will spin until the bumper is released.
     */
    private void farPowerAuto() {
        ((DcMotorEx) flywheel).setVelocity(farVelocity);
        agitator.setPower(-1);
        if (((DcMotorEx) flywheel).getVelocity() >= farVelocity - 100) {
            feeder.setPower(1);
        } else {
            feeder.setPower(0);
        }
    }

}
