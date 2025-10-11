package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@Config //Required to be able to tune parameters in FTCDashboard
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
    SparkFunOTOS poseOTOS;

    // Declare variables
    // Set as "static" and not "final" in order to be able to tune parameters in FTCDashboard
    // Setting our velocity targets. These values are in ticks per second!
    private static int bankVelocity = 1300;
    private static int farVelocity = 1900;
    private static int maxVelocity = 2200;
    private static double limitDrivePower = 0.5;  //Mutliplier to limit the drive wheel power for training
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
        poseOTOS = hardwareMap.get(SparkFunOTOS.class, "sensor-otos");

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

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Get the latest position, which includes the x and y coordinates, plus the
                // heading angle
                SparkFunOTOS.Pose2D pos = poseOTOS.getPosition();

                // Calling our methods while the OpMode is running
                splitStickArcadeDrive();
                setFlywheelVelocity();
                manualFeederAndagitatorControl();

                ////////////////////////////////////////////////////////////////////////////////
                //Set up driver hub telemetry
                telemetry.addData("Alliance", blackboard.get(ALLIANCE_KEY));
                telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
                telemetry.addData("Flywheel Power", flywheel.getPower());
                telemetry.addLine();
                // Log the position to the telemetry
                telemetry.addData("X coordinate", pos.x);
                telemetry.addData("Y coordinate", pos.y);
                telemetry.addData("Heading angle", pos.h);
                telemetry.update();

                ////////////////////////////////////////////////////////////////////////////////
                // Set up channels for display in FTCDashboard
                FtcDashboard dashboard = FtcDashboard.getInstance();
                TelemetryPacket packet = new TelemetryPacket();

                // Send a value to the dashboard for graphing
                dashboard.sendTelemetryPacket(packet); // Always send the packet
                packet.put("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity()); // Robot-specific data
                packet.put("Flywheel Power", flywheel.getPower()); // Robot-specific data


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

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // poseOTOS.setLinearUnit(DistanceUnit.METER);
        poseOTOS.setLinearUnit(DistanceUnit.INCH);
        // poseOTOS.setAngularUnit(AnguleUnit.RADIANS);
        poseOTOS.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        poseOTOS.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        poseOTOS.setLinearScalar(1.0);
        poseOTOS.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        poseOTOS.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        poseOTOS.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        poseOTOS.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        poseOTOS.getVersionInfo(hwVersion, fwVersion);

        //telemetry.addLine("OTOS configured! Press start to get position data!");
        //telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

}
