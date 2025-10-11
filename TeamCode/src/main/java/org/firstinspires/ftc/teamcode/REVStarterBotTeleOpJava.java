package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;



@Config //Required to be able to tune parameters in FTCDashboard
@TeleOp
public class REVStarterBotTeleOpJava extends LinearOpMode {

    private DcMotorSimple flywheel;
    private DcMotorSimple feeder;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private CRServo agitator;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private RevBlinkinLedDriver lightsLED;

    // Declare variables
    // Set as "static" and not "final" in order to be able to tune parameters in FTCDashboard
    // Setting our velocity targets. These values are in ticks per second!
    private static double bankVelocity = 0.6;
    private static double farVelocity = 0.9;
    private static float maxVelocity = 1;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorSimple.class, "motor-flywheel");
        feeder = hardwareMap.get(DcMotorSimple.class, "motor-feeder");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left-front-drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left-back-drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right-front-drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right-back-drive");
        agitator = hardwareMap.get(CRServo.class, "servo-agitator");
        lightsLED = hardwareMap.get(RevBlinkinLedDriver.class,"pwm-LED");

        // Establishing the direction and mode for the motors
        // flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //Ensures the agitator is active and ready
        agitator.setPower(0);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our methods while the OpMode is running
                splitStickArcadeDrive();
                setFlywheelVelocity();
                manualFeederAndagitatorControl();

                /////////////////////////////////////////////////////////////////////////////////
                //Set up the telemetry to the driver hub
                //telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
                telemetry.addData("Flywheel Power", flywheel.getPower());
                telemetry.update();

                /////////////////////////////////////////////////////////////////////////////////
                // Set up channels for display in FTCDashboard
                FtcDashboard dashboard = FtcDashboard.getInstance();
                TelemetryPacket packet = new TelemetryPacket();

                // Send a value to the dashboard for graphing
                dashboard.sendTelemetryPacket(packet); // Always send the packet
                packet.put("Flywheel Power", flywheel.getPower()); // Robot-specific data
                packet.put("Feeder Power", feeder.getPower()); // Robot-specific data
                packet.put("Agitator Power", agitator.getPower()); // Robot-specific data

                //Set up the Field overlay
                packet.fieldOverlay()
                        .setFill("blue")
                        .fillRect(-20, -20, 40, 40);
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
    private void manualFeederAndagitatorControl() {
        // Manual control for the Core Hex agitator
        if (gamepad1.a) {
            feeder.setPower(0.5);
        } else if (gamepad1.y) {
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
        } else if (gamepad1.x) {
            (flywheel).setPower(farVelocity);
        } /*else if (gamepad1.right_bumper) {
           flywheel.setPower(0.7);
        } */   else if (gamepad1.b) {
            flywheel.setPower(bankVelocity);
        } else if (gamepad1.left_bumper) {
            flywheel.setPower(maxVelocity);
        } else {
            (flywheel).setPower(0);
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
   /*  private void bankShotAuto() {
        (flywheel).setPower(bankVelocity);
        agitator.setPower(-1);
       if (flywheel).getPower() >= bankVelocity - 50) {
            feeder.setPower(1);
        } else {
            feeder.setPower(0);
        }
    }*/

    /**
     * The far power velocity is intended for launching balls a few feet from the goal. It may require adjusting the deflector.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The agitator will spin until the bumper is released.
     */
   /* private void farPowerAuto() {
        (flywheel).setPower(farVelocity);
        int farVelocity1 = (farVelocity);
        agitator.setPower(-1);
        if (flywheel).getVelocity() >= farVelocity1 - 100) {
            feeder.setPower(1);
        } else {
            feeder.setPower(0);
        }
    }*/

}
