package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Config //Required to be able to tune parameters in FTCDashboard
@TeleOp
public class TELEOP_MAIN_temp extends LinearOpMode {

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
    private static double bankVelocity = 760;
    private static double farVelocity = 1260;
    private static double maxVelocity = 1760;
    private double targetVelocity;
    public static PIDFCoefficients flywheelPID = new PIDFCoefficients(400,40,0,0);
    public static final String ALLIANCE_KEY = "Alliance";
    public Object colorAlliance = blackboard.get(ALLIANCE_KEY);
    public double allianceLEDColor;
    double previousError = 0;
    double deltaTime;
    public ElapsedTime deltaTimer = new ElapsedTime();
    public double lastTime = 0.0;       // Create a variable to hold the last recorded time

    public double blueGoalY = -71;      //Y coordinate of the blue alliance goal corner
    public double redGoalY = 71;        //Y coordinate of the red alliance goal corner
    public double goalY;                //Y goal coordinate
    public double goalX = -71;          //X coordinate of the goal corner (same for both alliances)

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


        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();
        deltaTimer.reset();
        lastTime = deltaTimer.seconds();

        //Set alliance-specific settings
        if(colorAlliance=="BLUE"){
            allianceLEDColor = 0.600; // blue
            goalY = blueGoalY;
        }
        else{
            allianceLEDColor = 0.283; // red
            goalY = redGoalY;
        }

        teamLED.setPosition(0.400);  //green, ready to go

//        // Set up channels for display in FTCDashboard
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        waitForStart();
        if (opModeIsActive()) {
            double currentTime = deltaTimer.seconds();
            deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            while (opModeIsActive()) {
                //Create new FTCDashboard packet
                TelemetryPacket packet = new TelemetryPacket();

                // Get the latest position, which includes the x and y coordinates, plus the
                // heading angle
                SparkFunOTOS.Pose2D pos = poseOTOS.getPosition();


                //Set alliance-specific settings
                if(colorAlliance=="BLUE"){
                    teamLED.setPosition(0.600); //set team LED to blue
                    goalY = blueGoalY;          //blue goal corner Y coordinate
                }
                else{
                    teamLED.setPosition(0.283); //set team LED to red
                    goalY = redGoalY;           //red goal corner Y coordinate
                }

                // Calling our methods while the OpMode is running
                splitStickArcadeDrive();
                setFlywheelVelocity();
//                manualFeederControl();

                //Call the rotate function to aim the robot at the alliance goal
                //NOTE: the bot should be pointed in the general direction of the goal for this to work best
                if (gamepad2.right_trigger >=0.1 ){
                    rotate();
                }



                /////////////////////////////////////////////////////////////////////////////////
                //Set up the telemetry to the driver hub
                telemetry.addData("Alliance", blackboard.get(ALLIANCE_KEY));
                telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
                telemetry.addData("Flywheel Power", flywheel.getPower());
                // Log the position to the telemetry
                telemetry.addData("X coordinate", pos.x);
                telemetry.addData("Y coordinate", pos.y);
                telemetry.addData("Heading angle", pos.h);
                telemetry.update();

                /////////////////////////////////////////////////////////////////////////////////
                // Send a value to the dashboard for graphing

//                packet.put("Flywheel Actual Velocity", flywheel.getVelocity()); // Robot-specific data
//                packet.put("Flywheel Target Velocity", targetVelocity); // Robot-specific data
////                dashboard.sendTelemetryPacket(packet); // Always send the packet
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
     *Code and explanation are at: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
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

    /**
     * This if/else statement contains the controls for the flywheel, both manual and auto.
     * Circle and Square will spin up ONLY the flywheel to the target velocity set.
     * The bumpers will activate the flywheel, Core Hex feeder, and servo to cycle a series of balls.
     */
    private void setFlywheelVelocity() {
        if (gamepad1.options) {
            flywheel.setPower(-0.5);
        } else if (gamepad1.left_trigger >=0.1) {
            flywheel.setVelocity(farVelocity);
            targetVelocity = farVelocity;
        }else if (gamepad1.left_bumper){
            bankShotAuto();
        }else if (gamepad1.b){
            farPowerAuto();
        }else if (gamepad1.x){
            maxShotAuto();
        } else if (gamepad1.right_trigger >=0.1) {
            flywheel.setVelocity(bankVelocity);
            targetVelocity = bankVelocity;
        } else if (gamepad1.right_bumper) {
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
        flywheel.setVelocity(farVelocity);
        if (flywheel.getVelocity() >= farVelocity - 50) {
            feeder.setPower(0.5);
            teamLED.setPosition(0.500); //green
        } else {
            feeder.setPower(0);
            teamLED.setPosition(allianceLEDColor);
        }
    }
    private void bankShotAuto() {
        (flywheel).setPower(bankVelocity);
        if (flywheel.getVelocity() >= bankVelocity - 50) {
            feeder.setPower(0.5);
            teamLED.setPosition(0.500); //green
        } else {
            feeder.setPower(0);
            teamLED.setPosition(allianceLEDColor);
        }
    }
    private void maxShotAuto() {
        (flywheel).setPower(maxVelocity);
        if (flywheel.getVelocity()>= maxVelocity - 50) {
            feeder.setPower(0.5);
            teamLED.setPosition(0.500); //green
        } else {
            feeder.setPower(0);
            teamLED.setPosition(allianceLEDColor);
        }
    }

    /**
     * rotate() calculates the heading to the alliance goal using ATAN2
     * The included PD controller turns the bot by calculating the difference between the
     * goal heading "angle" and the actual field heading of the bot pos.h.
     * THIS CODE CAME FROM THE 7TH-8TH GRADE BOT PROGRAMMERS AND WAS MODIFIED FOR NOODLES
     */

    private void rotate() {
        SparkFunOTOS.Pose2D pos = poseOTOS.getPosition();
//        int blueX = -71;
//        int blueY = -71;
//        int redX = -71;
//        int redY = 71;
        double angle;
        double kP = (0.8);
//        double kP = (1.0/36.0);  //Stiffles' gain
        double kD = 0;
        // spin drive with p controller
//        double x = blueX - pos.x;
//        double y = blueY - pos.y;
        double x = goalX - pos.x;
        double y = goalY - pos.y;
        angle = Math.atan2(y,x);
        double error = angle - pos.h;
        double derivativeError = (error - previousError) / deltaTime;
        double wheelpower = ((error * kP) + (kD * derivativeError));
        previousError = error;
        leftFrontDrive.setPower(-wheelpower);   //changed to (-) for Noodle's wiring
        leftBackDrive.setPower(-wheelpower);    //changed to (-) for Noodle's wiring
        rightFrontDrive.setPower(wheelpower);
        rightBackDrive.setPower(wheelpower);
        telemetry.addData("wheel power", wheelpower);
        telemetry.addData("angle", angle);
        telemetry.addData("pos x", pos.x);
        telemetry.addData("pos y", pos.y);
        telemetry.addData("pos h", pos.h);

    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements.
        // poseOTOS.setLinearUnit(DistanceUnit.METER);
        poseOTOS.setLinearUnit(DistanceUnit.INCH);
        poseOTOS.setAngularUnit(AngleUnit.RADIANS);
        // poseOTOS.setAngularUnit(AngleUnit.DEGREES);

        // Sensor position offset from RR tuning
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
