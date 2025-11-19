package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config //Required to be able to tune parameters in FTCDashboard
@TeleOp
public class TELEOP_MAIN_MAXWELLS_CHANGES extends LinearOpMode {

    private DcMotorEx flywheel;
    private DcMotor feeder;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private Servo teamLED;
    SparkFunOTOS poseOTOS;

    // Declare variables
    // Set as "static" and not "final" in order to be able to tune parameters in FTCDashboard
    // Setting our velocity targets. These values are in ticks per second!
    public static double bankVelocity = 760;
    public static double farVelocity = 1260;
    public static double maxVelocity = 1760;
    public static  double targetVelocity;
    public static PIDFCoefficients flywheelPID = new PIDFCoefficients(400,40,0,0);
//    PIDFCoefficients flywheelPID;

    public static final String ALLIANCE_KEY = "Alliance";
    public Object colorAlliance = blackboard.get(ALLIANCE_KEY);
    public double allianceLEDColor;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "motor-flywheel");
        feeder = hardwareMap.get(DcMotor.class, "motor-feeder");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left-front-drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left-back-drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right-front-drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right-back-drive");
        poseOTOS = hardwareMap.get(SparkFunOTOS.class, "sensor-otos");
        teamLED = hardwareMap.get(Servo.class, "led-light");

        // Establishing the direction and mode for the motors
        flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,flywheelPID);
//        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

//        flywheelPID = flywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

        //Set alliance-specific settings
        if(colorAlliance=="BLUE"){
            allianceLEDColor = 0.600; // blue
        }
        else{
            allianceLEDColor = 0.283; // red
        }

        teamLED.setPosition(0.400);  //green, ready to go

        // Set up channels for display in FTCDashboard
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        waitForStart();
        if (opModeIsActive()) {
            teamLED.setPosition(allianceLEDColor);  //turn on the LED to the alliance color

            while (opModeIsActive()) {

                // Get the latest position, which includes the x and y coordinates, plus the
                // heading angle
                SparkFunOTOS.Pose2D pos = poseOTOS.getPosition();

                // Calling our methods while the OpMode is running
                splitStickArcadeDrive();
                setFlywheelVelocity();
//                manualFeederControl();

                /////////////////////////////////////////////////////////////////////////////////
                //Set up the telemetry to the driver hub
                telemetry.addData("Alliance", blackboard.get(ALLIANCE_KEY));
                telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
                telemetry.addData("Flywheel Power", flywheel.getPower());
                // Log the position to the telemetry
                telemetry.addData("X coordinate", pos.x);
                telemetry.addData("Y coordinate", pos.y);
                telemetry.addData("Heading angle", pos.h);
                telemetry.addData("Flywheel P", flywheelPID.p);
                telemetry.addData("Flywheel I", flywheelPID.i);
                telemetry.addData("Flywheel D", flywheelPID.d);
                telemetry.addData("Flywheel F", flywheelPID.f);
                telemetry.update();

//                /////////////////////////////////////////////////////////////////////////////////
//                // Send a value to the dashboard for graphing
//                TelemetryPacket packet = new TelemetryPacket();
//                packet.put("Flywheel Actual Velocity", flywheel.getVelocity()); // Robot-specific data
//                packet.put("Flywheel Target Velocity", targetVelocity); // Robot-specific data
//                dashboard.sendTelemetryPacket(packet); // Always send the packet
//                telemetry.addData("Flywheel Actual Velocity", flywheel.getVelocity());
//                telemetry.addData("Flywheel Target Velocity", targetVelocity);
//                telemetry.update();
//
//                //Set up the Field overlay
//                packet.fieldOverlay()
//                        .setFill("blue")
//                        .fillRect(-20, -20, 40, 40);
            }
        }
    }

    /**
     * Controls for the drivetrain. The robot uses a mecanum drivetrain.
     * Forward and back is on the left stick. Strafing is on the left stick.  Turning is on the right stick.
     *Code and explanation are at: <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">...</a>
     */
    private void splitStickArcadeDrive() {
        double x;
        double y;
        double rx;

        x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
        y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
        rx = gamepad2.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        double frontLeftPower = (y + x + rx)*0.75 / denominator;
        double backLeftPower = (y - x + rx)*0.75 / denominator;
        double frontRightPower = (y - x - rx)*0.75 / denominator;
        double backRightPower = (y + x - rx)*0.75 / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);


    }

    /**
     * Manual control for the Core Hex powered feeder and the agitator servo in the hopper
     */
//    private void manualFeederControl() {
//        // Manual control for the Core Hex feeder
//        if (gamepad1.leftBumperWasPressed()) {
//            feeder.setPower(1.0);
//            teamLED.setPosition(1.0);
//        }
//        else if (gamepad1.rightBumperWasPressed()) {
//            feeder.setPower(-1.0);
//            teamLED.setPosition(1.0);
//        }
//        else if (gamepad1.leftBumperWasReleased() || gamepad1.rightBumperWasReleased()) {
//            feeder.setPower(0);
//            teamLED.setPosition(allianceLEDColor);
//        }
//    }

    /** NEW BUTTON PROPOSAL:
     * Y = close launch (closest to the goal from driver's perspective
     * B = medium launch
     * A = far launch (furthest from the goal from driver's perspective)
     * left_bumper = manually run feeder
     * right_bumper = manually reverse feeder
     * dpad up = manual max flywheel velocity (for testing)
     * dpad right = manual med flywheel velocity (for testing)
     * dpad down = manual bank flywheel velocity (for testing)
     */
    //    private void setFlywheelVelocity() {
//        if (gamepad1.yWasPressed()){                //Auto shoot close (bank)
//            bankShotAuto();
//            targetVelocity = bankVelocity;
//        } else if (gamepad1.bWasPressed()){          //Auto shoot mid
//            farPowerAuto();
//            targetVelocity = farVelocity;
//        } else if (gamepad1.aWasPressed()){          //Auto shoot far
//            maxShotAuto();
//            targetVelocity = maxVelocity;
//        } else if (gamepad1.yWasReleased()||gamepad1.bWasReleased()||gamepad1.aWasReleased()){          //Reset LED to alliance color
//            teamLED.setPosition(allianceLEDColor);
//        } else if (gamepad1.dpad_down) {            //Manual flywheel close (bank) for testing
//            ((DcMotorEx) flywheel).setVelocity(bankVelocity);
//            targetVelocity = bankVelocity;
//            teamLED.setPosition(1.0);
//        } else if (gamepad1.dpad_right) {           //Manual flywheel mid for testing
//            ((DcMotorEx) flywheel).setVelocity(farVelocity);
//            targetVelocity = farVelocity;
//            teamLED.setPosition(1.0);
//        } else if (gamepad1.dpad_up) {              //Manual flywheel far for testing
//            ((DcMotorEx) flywheel).setVelocity(maxVelocity);
//            targetVelocity = maxVelocity;
//            teamLED.setPosition(1.0);
//        } else {                                    //Set everything back to zero
//            (flywheel).setPower(0);
//            feeder.setPower(0);
//            teamLED.setPosition(allianceLEDColor);
//        }
//    }
    private void setFlywheelVelocity() {
        if (gamepad1.options) {
            flywheel.setPower(-0.5);
        } else if (gamepad1.x) {
            flywheel.setVelocity(farVelocity);
            targetVelocity = farVelocity;
        }else if (gamepad1.right_trigger >0.1){
            bankShotAuto();
        }else if (gamepad1.left_trigger >0.1){
            farPowerAuto();
        }else if (gamepad1.right_bumper){
            maxShotAuto();
        } else if (gamepad1.b) {
            flywheel.setVelocity(bankVelocity);
            targetVelocity = bankVelocity;
        } else if (gamepad1.left_bumper) {
            flywheel.setVelocity(maxVelocity);
            targetVelocity = maxVelocity;
        } else {
            (flywheel).setPower(0);
            feeder.setPower(0);
            teamLED.setPosition(allianceLEDColor);
        }
    }

    /**
     * The bank shot or near velocity is intended for launching balls touching or a few inches from the goal.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The agitator will spin until the bumper is released.
     */
    private void farPowerAuto() {
        ((DcMotorEx) flywheel).setVelocity(farVelocity);
        if (flywheel.getVelocity() >= farVelocity - 50) {
            feeder.setPower(1);
            teamLED.setPosition(0.500); //green
        } else {
            feeder.setPower(0);
            teamLED.setPosition(allianceLEDColor);
        }
    }
    private void bankShotAuto() {
        ((DcMotorEx) flywheel).setVelocity(bankVelocity);
        if (flywheel.getVelocity()>= bankVelocity - 50) {
            feeder.setPower(1);
            teamLED.setPosition(0.500); //green
        } else {
            feeder.setPower(0);
            teamLED.setPosition(allianceLEDColor);
        }
    }
    private void maxShotAuto() {
        ((DcMotorEx) flywheel).setVelocity(maxVelocity);
        if (flywheel.getVelocity()>= maxVelocity - 50) {
            feeder.setPower(1);
            teamLED.setPosition(0.500); //green
        } else {
            feeder.setPower(0);
            teamLED.setPosition(allianceLEDColor);
        }
    }

    /**
     * The OTOS configuration code is from the SensorSparkFunOTOS sample code.
     * There are several comments in the sample that have been removed here which
     * show how to configure the sensor.
     */


    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements.
        // poseOTOS.setLinearUnit(DistanceUnit.METER);
        poseOTOS.setLinearUnit(DistanceUnit.INCH);
        poseOTOS.setAngularUnit(AngleUnit.RADIANS);

        // Sensor position offset
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0.65625, 0);
        poseOTOS.setOffset(offset);

        // Linear and angular scalars that were tuned in OTOSLocalizer during RoadRunner tuning
        poseOTOS.setLinearScalar(0.9792);
        poseOTOS.setAngularScalar(-0.9928);

        // Calibrate the IMU when the bot is at rest during init()
        poseOTOS.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin
        poseOTOS.resetTracking();

        // The origin position.  We reset this to the end position in AUTO from the blackboard
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
