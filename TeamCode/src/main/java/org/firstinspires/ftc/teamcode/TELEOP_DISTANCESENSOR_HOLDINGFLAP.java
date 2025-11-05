package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

//
@Config //Required to be able to tune parameters in FTCDashboard
@TeleOp(name = "Distance Sensor")
public class TELEOP_DISTANCESENSOR_HOLDINGFLAP extends LinearOpMode {

    private DcMotorEx flywheel;
    private Limelight3A limelight;
    double previousError = 0;

    private DistanceSensor purpleDistanceSensor;
    private DistanceSensor greenDistanceSensor;

    private Servo purpleServo;
    private Servo greenServo;
    private ShooterState shooterState = ShooterState.IDLE;
    private ElapsedTime servoTime = new ElapsedTime();
    private double startServoTime = 0;
    private boolean seenobelisk = false;

    private int teamPipeline = 0; // blue auton
    private int patternID = 0;
    private int ballnumber = 0;
    private double servoShootPosGreen = 0.1667;  //was 0.2467
    private double servoShootPosPurple = 0.17;  //was 0.0933
    private double servoShootPosGreenDown = 0.32;
    private double servoShootPosPurpleDown = 0.02;

    private double farVelocity = 1360;
    private double closeVelocity = 1200;
    private double idleVelocity = 600;
    private double targetVelocity = 600;
    private boolean shooterOn = false;
    public double txLimelight;
    public double tyLimelight;
    double tx = 0;
    double ty = 0;
    int tagID;
    double deltaTime;

    private DcMotor intake;
    private Servo selector;
    private ColorSensor colorSensorA;
    private ColorSensor colorSensorB;
    private CRServo conveyorG;
    private CRServo conveyorP;
    private double sortOffset = 55/300;
    private DcMotor lift;



    private ElapsedTime runtime = new ElapsedTime();
    final int extensionposition = 0;
    final int packagedposition = 0;
    public ElapsedTime deltaTimer = new ElapsedTime();

    // Create a variable to hold the last recorded time
    public double lastTime = 0.0;

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    SparkFunOTOS poseOTOS;

    private static double limitDrivePower = 0.5;  //Mutliplier to limit the drive wheel power for training
    public static final String ALLIANCE_KEY = "Alliance";

    @Override
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, "motor-flywheel");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        greenServo = hardwareMap.get(Servo.class, "greenServo");
        purpleServo = hardwareMap.get(Servo.class, "purpleServo");
        poseOTOS = hardwareMap.get(SparkFunOTOS.class, "sensor-otos");
        purpleDistanceSensor = hardwareMap.get(DistanceSensor.class, "purpleDistanceSensor");
        greenDistanceSensor = hardwareMap.get(DistanceSensor.class, "greenDistanceSensor");

 //       flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterpid);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        greenServo.setDirection(Servo.Direction.FORWARD);
        purpleServo.setDirection(Servo.Direction.FORWARD);

        limelight.start();
        limelight.pipelineSwitch(2);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left-front-drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left-back-drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right-front-drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right-back-drive");

        // Establishing the direction and mode for the motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "motor-intake");
        selector = hardwareMap.get(Servo.class, "servo-selector");
        colorSensorA = hardwareMap.get(ColorSensor.class, "sensor-color-a");
        colorSensorB = hardwareMap.get(ColorSensor.class, "sensor-color-b");
        conveyorG = hardwareMap.get(CRServo.class, "servo-conveyor-green");
        conveyorP = hardwareMap.get(CRServo.class, "servo-conveyor-purple");


        // Establishing the direction and mode for the motors
        intake.setDirection(DcMotor.Direction.REVERSE);
        selector.setDirection(Servo.Direction.FORWARD);
        conveyorG.setDirection(CRServo.Direction.FORWARD);
        conveyorP.setDirection(CRServo.Direction.FORWARD);
        selector.setDirection(Servo.Direction.FORWARD);

        lift = hardwareMap.get(DcMotor.class, "motor-lift");
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setTargetPosition(packagedposition);
        lift.setPower(0.5);

        telemetry.setMsTransmissionInterval(11);

        configureOtos();
        deltaTimer.reset();
        lastTime = deltaTimer.seconds();

        waitForStart();
        if (opModeIsActive()) {
            double currentTime = deltaTimer.seconds();
            deltaTime = currentTime - lastTime;
            lastTime = currentTime;
            flywheel.setVelocity(targetVelocity);
            greenServo.setPosition(servoShootPosGreenDown);
            purpleServo.setPosition(servoShootPosPurpleDown);

            conveyorG.setPower(1);
            conveyorP.setPower(1);

            runtime.reset();
            while (opModeIsActive()) {
                aimBot();
                intakeSort(false);
                drivetrain();
                lift();

                FtcDashboard dashboard = FtcDashboard.getInstance();
                TelemetryPacket packet = new TelemetryPacket();
                dashboard.sendTelemetryPacket(packet); // Always send the packet
                packet.fieldOverlay()
                        .setFill("blue")
                        .fillRect(-20, -20, 40, 40);

                telemetry.addData("Alliance", blackboard.get(ALLIANCE_KEY));
                telemetry.addData("time", runtime.time());
                telemetry.update();
            }
        }
    }

    private boolean purpleBallDetected() {
        return purpleDistanceSensor.getDistance(DistanceUnit.INCH) < 5;
        if (purpleDistanceSensor.getDistance(DistanceUnit.INCH) < 5) {
            telemetry.addLine("⚠️ PURPLE ARTIFACT IN ROBOT ⚠️");
            telemetry.addLine("PLEASE PURPLE SPEED I NEED THIS MY MOM IS KIND OF HOMELESS");
        }
    }
    private void aimBot() {
        switch (shooterState) {
            case IDLE:
                flywheel.setVelocity(idleVelocity);
                break;
            case WAITING_FOR_FLYWHEEL:
                flywheel.setVelocity(targetVelocity);
                break;
            case WAITING_FOR_SERVO:
                if (servoTime.milliseconds() - startServoTime > 250) {
                  //  startServoTime = 0;
                    shooterState = shooterState.IDLE_WITH_FLYWHEEL;
                }
            case IDLE_WITH_FLYWHEEL:

                break;
        }
//        if (shooterOn) {
//            flywheel.setVelocity(targetVelocity);
//        } else {
//            flywheel.setVelocity(0);
//        }


        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial != null) {
                tagID = fiducial.getFiducialId();
            }
        }

        if (tagID != 0  && !seenobelisk) {
            seenobelisk = true;
            patternID = tagID; // save pattern
            sleep(50);
            tagID = 0;
            limelight.pipelineSwitch(teamPipeline);
        }

        if (gamepad2.yWasPressed()) {  //&& result.isValid()+

            if (patternID == 22 && tagID == 20) {
                rotate();
                if (Math.abs(tx) < 5) {
                    Pattern22(); // Purple green purple
                }
            } else if (patternID == 21 && tagID == 20) {
                rotate();
                if (Math.abs(tx) < 5) {
                    Pattern21(); // Green purple purple
                }
            } else if (patternID == 23 && tagID == 20) {
                rotate();
                if (Math.abs(tx) < 5) {
                    Pattern23(); // Purple purple green
                }
            }
        } else if (gamepad2.dpadRightWasPressed()) {             // Forced shooting: Purple
            purpleServo.setPosition(servoShootPosPurple);
            sleep(250);
            purpleServo.setPosition(servoShootPosPurpleDown);
        } else if (gamepad2.dpadLeftWasPressed()) {      // Forced shooting: Green
            greenServo.setPosition(servoShootPosGreen);
            sleep(250);
            greenServo.setPosition(servoShootPosGreenDown);
        }
        if (gamepad2.x) {
            ballnumber = 1;
        }
        if (gamepad1.aWasPressed()) {
            shooterOn = !shooterOn;
        }

        telemetry.addData("timer start", startServoTime);
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Pattern ID", patternID);
        telemetry.addData("Ball Number", ballnumber);
        telemetry.addData("Seen obelisk", seenobelisk);
        telemetry.addData("Tag ID", tagID);
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
        telemetry.addData("Flywheel Power", flywheel.getPower());
        telemetry.addData("Target X", tx);
        telemetry.addData("Target Y", ty);
    }
    private void Pattern22() {
        if (ballnumber == 1 && flywheel.getVelocity() > targetVelocity - 60) {
            if(!(startServoTime > 1)) {
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
                purpleServo.setPosition(servoShootPosPurple);
        }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(servoShootPosPurpleDown);
                ++ballnumber;
            }
        } else if (ballnumber == 2 && flywheel.getVelocity() > targetVelocity - 60) {
            if((startServoTime > 1)) {  //REMOVED THE ! TO TEST
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
                greenServo.setPosition(servoShootPosGreen);
           }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                greenServo.setPosition(servoShootPosGreenDown);
                ++ballnumber;
            }
        } else if (ballnumber == 3 && flywheel.getVelocity() > targetVelocity - 60) {
           if((startServoTime > 1)) {  //REMOVED THE ! TO TEST
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
                purpleServo.setPosition(servoShootPosPurple);
           }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(servoShootPosPurpleDown);
                ballnumber = 0;
            }
        }
    }

    private void Pattern21() {
        if (ballnumber == 1 && flywheel.getVelocity() > targetVelocity - 60) {
            if(!(startServoTime > 1)) {
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
                greenServo.setPosition(servoShootPosGreen);
            }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                greenServo.setPosition(servoShootPosGreenDown);
                ++ballnumber;
            }
        } else if (ballnumber == 2 && flywheel.getVelocity() > targetVelocity - 60) {
            if(!(startServoTime > 1)) {
                purpleServo.setPosition(servoShootPosPurple);
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
            }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(servoShootPosPurpleDown);
                ++ballnumber;
            }
        } else if (ballnumber == 3 && flywheel.getVelocity() > targetVelocity - 60) {
            if(!(startServoTime > 1)) {
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
            }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(servoShootPosPurple);
                ballnumber = 0;
            }
        }
        else if (ballnumber == 4 && flywheel.getVelocity() > targetVelocity - 60) {
            if (!(startServoTime > 1)) {
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
            }
            if (shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(servoShootPosPurpleDown);
                ballnumber = 0;
            }
        }
    }
    private void Pattern23() {
        if (ballnumber == 1 && flywheel.getVelocity() > targetVelocity - 60) {
            if(!(startServoTime > 1)) {
                purpleServo.setPosition(servoShootPosPurple);
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
            }
            if (shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(servoShootPosPurpleDown);
                ++ballnumber;
            }
        } else if (ballnumber == 2 && flywheel.getVelocity() > targetVelocity - 60) {
            if(!(startServoTime > 1)) {
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
            }
            if (shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(servoShootPosPurple);
                ++ballnumber;
            }
            else if (ballnumber == 3 && flywheel.getVelocity() > targetVelocity - 60) {
                if(!(startServoTime > 1)) {
                    startServoTime = servoTime.milliseconds();
                    shooterState = shooterState.WAITING_FOR_SERVO;
                }
                if (shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                    purpleServo.setPosition(servoShootPosPurpleDown);
                    ballnumber = 0;
                }
            }
        } else if (ballnumber == 4 && flywheel.getVelocity() > targetVelocity - 60) {
            if(!(startServoTime > 1)) {
                greenServo.setPosition(servoShootPosGreen);
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
                greenServo.setPosition(servoShootPosGreenDown);
                ballnumber = 0;
            }
        }
    }
    private void rotate() {
        double kP = (1/24);
        double kD = 0;
        // spin drive with p controller
         double error = -ty;
         double derivativeError = (error - previousError) / deltaTime;
        double wheelpower = (error * kP + kD * derivativeError);
        previousError = error;
//        leftFrontDrive.setPower(wheelpower);
//        leftBackDrive.setPower(wheelpower);
//        rightFrontDrive.setPower(wheelpower);
//        rightBackDrive.setPower(wheelpower);

    }
    public void intakeSort(boolean auto) {

        sortArtifact();

        if(gamepad2.right_bumper){
            intake.setPower(-1);
            selector.setPosition(0.4);
        }
        if(gamepad2.right_trigger > 0.2 || auto){
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }

    public void sortArtifact() {
        float sumGreenPurpleness = greenPurplenessA() + greenPurplenessB();
        telemetry.addData("sumGreenPurpleness",sumGreenPurpleness);
        if(sumGreenPurpleness > 120){
            //selector.setPosition(0.4 + sortOffset); //calculating the offset here isn't working - data type?
            selector.setPosition(0.22);
        }
        else if (sumGreenPurpleness < -70 ) {
           //selector.setPosition(0.4 - sortOffset);  //calculating the offset here isn't working - data type?
            selector.setPosition(0.58);
        }
        else {
            selector.setPosition(0.4);
        }
        telemetry.addData("selector pos",selector.getPosition());
    }

    public float greenPurplenessA(){
        int  greenA = colorSensorA.green();
        int purpleA = (colorSensorA.red() + colorSensorA.blue())/2;
        return greenA - purpleA;
    }
    private float greenPurplenessB(){
        int  greenB = colorSensorB.green();
        int purpleB = (colorSensorB.red() + colorSensorB.blue())/2;
        return greenB - purpleB;
    }

    private void lift() {
        if (runtime.time() > 100) {

            telemetry.addData("expected lift position", lift.getTargetPosition());
            telemetry.addData("lift position", lift.getCurrentPosition());

            if (gamepad1.start) {
                lift.setTargetPosition(extensionposition);
            } else if (gamepad1.back) {
                lift.setTargetPosition(packagedposition);
            }
        }
    }
    private void drivetrain() {
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

        SparkFunOTOS.Pose2D pos = poseOTOS.getPosition();

        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);
        telemetry.addLine("PLEASE SPEED I NEED THIS MY MOM IS KIND OF HOMELESS");
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
