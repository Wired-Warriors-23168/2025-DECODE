package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;


@Config //Required to be able to tune parameters in FTCDashboard
@Autonomous(name="AUTO_Test_Pathing", group="AUTO", preselectTeleOp = "TELEOP_MAIN")
public class AUTO_Test_Pathing extends LinearOpMode {
    /////////////////////////////////////////////////////////////////////////
    // Declare variables
    /////////////////////////////////////////////////////////////////////////

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
    private double greenShootPos = 0.1667;  //was 0.2467
    private double purpleShootPos = 0.17;  //was 0.0933
    private double greenDownPos = 0.32;
    private double purpleDownPos = 0.02;
    private double greenHoldPos = 0.27;
    private double purpleHoldPos = 0.07;


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

    private double sortOffset = 55.0/300.0;
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
    SparkFunOTOS otos;

    private static double limitDrivePower = 0.5;  //Mutliplier to limit the drive wheel power for training
    public static final String ALLIANCE_KEY = "Alliance";

    @Override
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, "motor-flywheel");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        greenServo = hardwareMap.get(Servo.class, "greenServo");
        purpleServo = hardwareMap.get(Servo.class, "purpleServo");
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor-otos");
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

        //configureOtos();
        deltaTimer.reset();
        lastTime = deltaTimer.seconds();

        //TODO *********** Set the starting pose for the robot based on the alliance start position,
        // X and Y in INCHES from the center of the field, heading in RADIANS (or convert DEGREES to
        // RADIANS by multiplying the value in DEGREES by Math.PI/180
        Pose2d beginPose = new Pose2d(-62.5, -35, Math.toRadians(-90));

        //Instantiate the roadrunner Mecanum drive (via the OTOS localizer)
        //SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.localizer.setPose(beginPose);  //may have to do this for the new RR version per https://community.sparkfun.com/t/sparkfun-otos-with-ftc-inital-pose-always-0/67256

        //Set all field positions for RoadRunner
        //Pose2d waypointBank = new Pose2d(-30,-30,Math.toRadians(-135));  //Waypoint for spline, bankshot launch position
        //Pose2d waypointMid = new Pose2d(-30,-30,Math.toRadians(-135));    //Waypoint for spline, middle launch position
        //Pose2d waypointFar = new Pose2d(-30,-30,Math.toRadians(-135));    //Waypoint for spline, far launch position
        //Pose2d waypointPushStart = new Pose2d(-30,-30,Math.toRadians(-135));   //Waypoint to start pushing artifacts
        //Pose2d waypointPushEndSpline = new Pose2d(-30,-30,Math.toRadians(-135));       //Waypoint to end pushing artifacts with a spline
        //Vector2d waypointPushEndLine = new Vector2d(-51,-39);                 //Waypoint to end pushing artifacts with a LineTo




        ////////////////////////////////////////////////////////////////////////////////////
        // Wait for the game to start (driver presses START)
        ////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        if (opModeIsActive()) {
            double currentTime = deltaTimer.seconds();
            deltaTime = currentTime - lastTime;
            lastTime = currentTime;
            flywheel.setVelocity(targetVelocity);
            greenServo.setPosition(greenDownPos);
            purpleServo.setPosition(purpleDownPos);

            conveyorG.setPower(1);
            conveyorP.setPower(1);

            runtime.reset();
            while (opModeIsActive()) {
                SparkFunOTOS.Pose2D pos = otos.getPosition(); //Read OTOS Pose for telemetry



                //Roadrunner - drive first path
                Actions.runBlocking(
                        drive.actionBuilder(beginPose)

                                // Move to bankshot firing position
                                .setReversed(false)
                                .setTangent(Math.toRadians(45))  //the heading the bot will take when leaving this position
                                .splineToLinearHeading(new Pose2d(-27,-27,Math.toRadians(-135)),Math.toRadians(45))  //the target X,Y position, the target heading where the bot stops, and the heading the bot will approach that target heading from

                                .build());

                //Launch the pattern - this needs to run outside RoadRunner because it uses an FSM
                //need a method to command the launch since no gamepad
                aimBot();



                //Roadrunner - drive second path to the first ball to intake, run intakeSort, move forward to intake, then path back to launch position
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(pos.x,pos.y,pos.h))  //start from current pose

                                // Move to ball intake position
//                                .setReversed(false)
//                                .setTangent(Math.toRadians(45))  //the heading the bot will take when leaving this position
//                                .splineToLinearHeading(new Pose2d(-27,-27,Math.toRadians(-135)),Math.toRadians(45))  //the target X,Y position, the target heading where the bot stops, and the heading the bot will approach that target heading from

                                //path back to launch position
//                                .setTangent(Math.toRadians(45))  //the heading the bot will take when leaving this position
//                                .splineToLinearHeading(new Pose2d(-27,-27,Math.toRadians(-135)),Math.toRadians(45))  //the target X,Y position, the target heading where the bot stops, and the heading the bot will approach that target heading from

                                .build());

                //Launch the pattern
                //need a method to command the launch since no gamepad
                aimBot();

                //Roadrunner - drive to park position
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(pos.x,pos.y,pos.h))  //start from current pose

                                // Move to park position
//                                .setReversed(false)
//                                .setTangent(Math.toRadians(45))  //the heading the bot will take when leaving this position
//                                .splineToLinearHeading(new Pose2d(-27,-27,Math.toRadians(-135)),Math.toRadians(45))  //the target X,Y position, the target heading where the bot stops, and the heading the bot will approach that target heading from

                                .build());



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

    ///////////////////////////////////////////////////
    //PUBLIC CLASSES FOR ROADRUNNER ACTION DEFINITIONS
    //////////////////////////////////////////////////


    public class runIntakeSort implements Action {
        double intakeTime;
        ElapsedTime actionTimer;

        public runIntakeSort(double intakeTime) {
            this.intakeTime = intakeTime;
            this.actionTimer = actionTimer;
            actionTimer = new ElapsedTime();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (actionTimer == null) {
                actionTimer = new ElapsedTime();
            }

            intakeSort(false);

            return actionTimer.seconds()<intakeTime; //runs the action for maximum intakeTime seconds
        }
    }



    ///////////////////////////////////////////////////
    //Functionality from TELEOP
    //////////////////////////////////////////////////
    private boolean purpleBallDetected() {
        if (purpleDistanceSensor.getDistance(DistanceUnit.INCH) < 5) {
            telemetry.addLine("⚠️ PURPLE ARTIFACT IN ROBOT ⚠️");
            telemetry.addLine("PLEASE PURPLE SPEED I NEED THIS MY MOM IS KIND OF HOMELESS");
        }
        return purpleDistanceSensor.getDistance(DistanceUnit.INCH) < 5;

    }
    private boolean greenBallDetected() {
        if (greenDistanceSensor.getDistance(DistanceUnit.INCH) < 5) {
            telemetry.addLine("⚠️ GREEN ARTIFACT IN ROBOT ⚠️");
            telemetry.addLine("PLEASE GREEN SPEED I NEED THIS MY MOM IS KIND OF HOMELESS");
        }
        return greenDistanceSensor.getDistance(DistanceUnit.INCH) < 5;

    }
    private void aimBot() {
        //State machine important!!!
        switch (shooterState) {
            case IDLE:
                flywheel.setVelocity(idleVelocity);
                if(greenBallDetected()){
                    greenServo.setPosition(greenHoldPos);
                }
                else{
                    greenServo.setPosition(greenDownPos);
                }
                if(purpleBallDetected()){
                    purpleServo.setPosition(purpleHoldPos);
                }
                else{
                    purpleServo.setPosition(purpleDownPos);
                }
                break;
            case WAITING_FOR_FLYWHEEL:
                flywheel.setVelocity(targetVelocity);
                break;
            case WAITING_FOR_SERVO:
                if (servoTime.milliseconds() - startServoTime > 250) {
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

        if (gamepad2.y) {  //&& result.isValid()+

            if (patternID == 22 && tagID == 20) {
                rotate();
                if (Math.abs(ty) < 5) {
                    Pattern22(); // Purple green purple
                }
            } else if (patternID == 21 && tagID == 20) {
                rotate();
                if (Math.abs(ty) < 5) {
                    Pattern21(); // Green purple purple
                }
            } else if (patternID == 23 && tagID == 20) {
                rotate();
                if (Math.abs(ty) < 5) {
                    Pattern23(); // Purple purple green
                }
            }
        } else if (gamepad2.dpadRightWasPressed()) {             // Forced shooting: Purple
            purpleServo.setPosition(purpleShootPos);
            sleep(250);
            purpleServo.setPosition(purpleDownPos);
        } else if (gamepad2.dpadLeftWasPressed()) {      // Forced shooting: Green
            greenServo.setPosition(greenShootPos);
            sleep(250);
            greenServo.setPosition(greenDownPos);
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
                purpleServo.setPosition(purpleShootPos);
        }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(purpleDownPos);
                ++ballnumber;
            }
        } else if (ballnumber == 2 && flywheel.getVelocity() > targetVelocity - 60) {
            if(!(startServoTime > 1)) {
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
                greenServo.setPosition(greenShootPos);
           }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                greenServo.setPosition(greenDownPos);
                ++ballnumber;
            }
        } else if (ballnumber == 3 && flywheel.getVelocity() > targetVelocity - 60) {
           if(!(startServoTime > 1)) {
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
                purpleServo.setPosition(purpleShootPos);
           }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(purpleDownPos);
                shooterState = shooterState.IDLE;
                ballnumber = 0;
            }
        }
    }

    private void Pattern21() {
        if (ballnumber == 1 && flywheel.getVelocity() > targetVelocity - 60) {
            if(!(startServoTime > 1)) {
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
                greenServo.setPosition(greenShootPos);
            }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                greenServo.setPosition(greenDownPos);
                ++ballnumber;
            }
        } else if (ballnumber == 2 && flywheel.getVelocity() > targetVelocity - 60) {
            if(!(startServoTime > 1)) {
                purpleServo.setPosition(purpleShootPos);
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
            }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(purpleDownPos);
                if(purpleBallDetected()) {
                    ++ballnumber;
                }
            }
        }
        else if (ballnumber == 3 && flywheel.getVelocity() > targetVelocity - 60) {
            if (!(startServoTime > 1)) {
                purpleServo.setPosition(purpleShootPos);
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
            }
            if (shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(purpleDownPos);
                shooterState = shooterState.IDLE;
                ballnumber = 0;
            }
        }
    }
    private void Pattern23() {
        if (ballnumber == 1 && flywheel.getVelocity() > targetVelocity - 60) {
            if(!(startServoTime > 1)) {
                purpleServo.setPosition(purpleShootPos);
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
            }
            if (shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(purpleDownPos);
                if(purpleBallDetected()) {
                    purpleServo.setPosition(purpleShootPos);
                    ++ballnumber;
                }
            }
        } else if (ballnumber == 2 && flywheel.getVelocity() > targetVelocity - 60) {
                if(!(startServoTime > 1)) {
                    startServoTime = servoTime.milliseconds();
                    shooterState = shooterState.WAITING_FOR_SERVO;
                }
                if (shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                    purpleServo.setPosition(purpleDownPos);
                    ++ballnumber;
                }
        } else if (ballnumber == 3 && flywheel.getVelocity() > targetVelocity - 60) {
            if(!(startServoTime > 1)) {
                greenServo.setPosition(greenShootPos);
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
                greenServo.setPosition(greenDownPos);
                shooterState = shooterState.IDLE;
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
            selector.setPosition(0.4 + sortOffset); //calculating the offset here isn't working - data type?
            //selector.setPosition(0.22);
        }
        else if (sumGreenPurpleness < -70 ) {
           selector.setPosition(0.4 - sortOffset);  //calculating the offset here isn't working - data type?
           //Should work now
            //selector.setPosition(0.58);
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

        SparkFunOTOS.Pose2D pos = otos.getPosition();

        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);
    }





}
