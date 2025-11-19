package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;


@Config //Required to be able to tune parameters in FTCDashboard
@TeleOp
public class TELEOP_MAIN_ANDY_TEST extends LinearOpMode {

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
    private static double bankVelocity = 750;
    private static double farVelocity = 1250;
    private static double maxVelocity = 1750;
    private double targetVelocity;
    public static PIDFCoefficients flywheelPID = new PIDFCoefficients(10,3,0,0);
//    PIDFCoefficients flywheelPID;

    public static final String ALLIANCE_KEY = "Alliance";
    public Object colorAlliance = blackboard.get(ALLIANCE_KEY);
    public double allianceLEDColor;

    public double blueGoalY = -71;      //Y coordinate of the blue alliance goal corner
    public double redGoalY = 71;        //Y coordinate of the red alliance goal corner
    public double goalY;                //Y goal coordinate
    public double goalX = -71;          //X coordinate of the goal corner (same for both alliances)
    public double goalHeading = Math.toRadians(180);    //the Heading to the goal corner
    private List<Action> runningActions = new ArrayList<>();    //List of RoadRunner Actions that we want to run in TELEOP


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
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);


        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

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

            while (opModeIsActive()) {
                //Create new FTCDashboard packet
                TelemetryPacket packet = new TelemetryPacket();

                // Get the latest position, which includes the x and y coordinates, plus the
                // heading angle
                SparkFunOTOS.Pose2D pos = poseOTOS.getPosition();

//                //Set up the RoadRunner drive
//                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(pos.x, -pos.y, pos.h));
//                drive.localizer.setPose(new Pose2d(pos.x, -pos.y, pos.h));  //may have to do this for the new RR version per https://community.sparkfun.com/t/sparkfun-otos-with-ftc-inital-pose-always-0/67256


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
                manualFeederControl();
                //goalHeading();


                goalHeading = Math.atan2((goalY - pos.y),(goalX - pos.x));  //use ATAN2 function to calculate heading to goal corner

//                //Update running RoadRunner Actions
//                List<Action> newActions = new ArrayList<>();
//                for (Action action : runningActions) {
//                    action.preview(packet.fieldOverlay());
//                    if (action.run(packet)) {
//                        newActions.add(action);
//                    }
//                }
//                runningActions = newActions;
//
//                // Create trajectory for turning to the goal
//                TrajectoryActionBuilder turn1 = drive.actionBuilder(new Pose2d(pos.x,pos.y,pos.h))
//                        .turnTo(goalHeading);
//                Action turnGoal = turn1.build();
//
//                if (gamepad2.right_trigger >= 0.1) {
//                    runningActions.add(new SequentialAction(
//                            turnGoal
//                    ));
//                }

                //Option using simple proportional gain rather than Roadrunner
                double headingError = (goalHeading-pos.h);
                double wheelPower = headingError*0.5;
                if (gamepad2.right_trigger >=0.1 && Math.abs(headingError)>0.05){
                    leftBackDrive.setPower(wheelPower);
                    leftFrontDrive.setPower(wheelPower);
                    rightBackDrive.setPower(-wheelPower);
                    rightFrontDrive.setPower(-wheelPower);
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
                telemetry.addData("Heading to Goal", goalHeading);
                telemetry.addData("Heading Error", headingError);
                telemetry.addData("Wheel Power", wheelPower);
                telemetry.addLine();
                telemetry.addData("Flywheel P", flywheelPID.p);
                telemetry.addData("Flywheel I", flywheelPID.i);
                telemetry.addData("Flywheel D", flywheelPID.d);
                telemetry.addData("Flywheel F", flywheelPID.f);
                telemetry.update();

                /////////////////////////////////////////////////////////////////////////////////
                // Send a value to the dashboard for graphing

//                packet.put("Flywheel Actual Velocity", flywheel.getVelocity()); // Robot-specific data
//                packet.put("Flywheel Target Velocity", targetVelocity); // Robot-specific data
////                dashboard.sendTelemetryPacket(packet); // Always send the packet
//                telemetry.addData("Flywheel Actual Velocity", flywheel.getVelocity());
//                telemetry.addData("Flywheel Target Velocity", targetVelocity);
//                telemetry.update();

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
     * Function to calculate the heading and distance to the alliance goal.
     */
//    private void goalHeading(){
//        //Instantiate the roadrunner Mecanum drive (via the OTOS localizer)
//        //SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-0, -0, Math.toRadians(0)));
//        drive.localizer.setPose(new Pose2d(-0, -0, Math.toRadians(0)));  //may have to do this for the new RR version per https://community.sparkfun.com/t/sparkfun-otos-with-ftc-inital-pose-always-0/67256
//
//        SparkFunOTOS.Pose2D pos = poseOTOS.getPosition(); //Read OTOS Pose for telemetry
//
//        if (gamepad2.right_trigger>0.1){
//            goalHeading = Math.atan2((goalY - pos.y),(goalX - pos.x));  //use ATAN2 function to calculate heading to goal corner
//            Actions.runBlocking(                                        //turn bot to heading using Roadrunner
//                    drive.actionBuilder(new Pose2d(pos.x,pos.y,pos.h))
//                            .turnTo(goalHeading)
//                    .build());
//        }
//    }

    /**
     * Manual control for the Core Hex powered feeder and the agitator servo in the hopper
     */
    private void manualFeederControl() {
        // Manual control for the Core Hex feeder
        if (gamepad1.leftBumperWasPressed()) {
            feeder.setPower(1.0);
            teamLED.setPosition(1.0);
        }
        else if (gamepad1.rightBumperWasPressed()) {
            feeder.setPower(-1.0);
            teamLED.setPosition(1.0);
        }
        else if (gamepad1.leftBumperWasReleased() || gamepad1.rightBumperWasReleased()) {
            feeder.setPower(0);
            teamLED.setPosition(allianceLEDColor);
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
        flywheel.setVelocity(farVelocity);
        if (flywheel.getVelocity() >= farVelocity - 75) {
            feeder.setPower(0.5);
            teamLED.setPosition(0.500); //green
        } else {
            feeder.setPower(0);
            teamLED.setPosition(allianceLEDColor);
        }
    }
    private void bankShotAuto() {
        (flywheel).setPower(bankVelocity);
        if (flywheel.getVelocity() >= bankVelocity - 75) {
            feeder.setPower(0.5);
            teamLED.setPosition(0.500); //green
        } else {
            feeder.setPower(0);
            teamLED.setPosition(allianceLEDColor);
        }
    }
    private void maxShotAuto() {
        (flywheel).setPower(maxVelocity);
        if (flywheel.getVelocity()>= maxVelocity - 75) {
            feeder.setPower(0.5);
            teamLED.setPosition(0.500); //green
        } else {
            feeder.setPower(0);
            teamLED.setPosition(allianceLEDColor);
        }
    }

    /**
     * The far power velocity is intended for launching balls a few feet from the goal. It may require adjusting the deflector.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The agitator will spin until the bumper is released.
     */

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
        poseOTOS.setAngularUnit(AngleUnit.RADIANS);
        // poseOTOS.setAngularUnit(AngleUnit.DEGREES);

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
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0.65625, 0);
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
        poseOTOS.setLinearScalar(0.9792);
        poseOTOS.setAngularScalar(-0.9928);

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
