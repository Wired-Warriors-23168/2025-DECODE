package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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

import java.util.List;


@Config //Required to be able to tune parameters in FTCDashboard
@Autonomous(name="AUTO_BLUE_1", group="AUTO", preselectTeleOp = "TELEOP_MAIN")
public class AUTO_BLUE_1 extends LinearOpMode {
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
    public static final String ALLIANCE_KEY = "BLUE";

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
        Pose2d beginPose = new Pose2d(-62.5, -35, Math.toRadians(180));

        //Instantiate the roadrunner Mecanum drive (via the OTOS localizer)
        //SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.localizer.setPose(beginPose);  //may have to do this for the new RR version per https://community.sparkfun.com/t/sparkfun-otos-with-ftc-inital-pose-always-0/67256

        //Set all field positions for RoadRunner
        //Pose2d waypointBank = new Pose2d(-30,-30,Math.toRadians(-135));               //Waypoint for spline, bankshot launch position
        //Pose2d waypointMid = new Pose2d(-30,-30,Math.toRadians(-135));                //Waypoint for spline, middle launch position
        //Pose2d waypointFar = new Pose2d(-30,-30,Math.toRadians(-135));                //Waypoint for spline, far launch position
        //Pose2d waypointPushStart = new Pose2d(-30,-30,Math.toRadians(-135));          //Waypoint to start pushing artifacts
        //Pose2d waypointPushEndSpline = new Pose2d(-30,-30,Math.toRadians(-135));      //Waypoint to end pushing artifacts with a spline
        //Vector2d waypointPushEndLine = new Vector2d(-51,-39);                         //Waypoint to end pushing artifacts with a LineTo




        ////////////////////////////////////////////////////////////////////////////////////
        // Wait for the game to start (driver presses START)
        ////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        if (opModeIsActive()) {
            double currentTime = deltaTimer.seconds();
            deltaTime = currentTime - lastTime;
            lastTime = currentTime;

           //set artifact holding positions
            flywheel.setVelocity(0);
            greenServo.setPosition(greenHoldPos);
            purpleServo.setPosition(purpleHoldPos);


//            conveyorG.setPower(1);
//            conveyorP.setPower(1);

            //X and Y positions for AUTO pathing
            double shootX = -63;
            double shootY = 25;
            double endX = 58.5;
            double endY = -35.5;

            runtime.reset();
            while (opModeIsActive()) {
                SparkFunOTOS.Pose2D pos = otos.getPosition(); //Read OTOS Pose for telemetry

                //start the flywheel
                flywheel.setVelocity(closeVelocity);

                //drive to the obelisk read location
                Actions.runBlocking(
                        drive.actionBuilder(beginPose)

                                // Move to close firing position
                                .setReversed(false)
                                .setTangent(Math.toRadians(0))  //the heading the bot will take when leaving this position
                                .splineToLinearHeading(new Pose2d(-27,-27,Math.toRadians(135)),Math.toRadians(-45))  //the target X,Y position, the target heading where the bot stops, and the heading the bot will approach that target heading from
                                .build());

                //Read the limelight and determine pattern
                LLResult result = limelight.getLatestResult();
//                if (result != null && result.isValid()) {
//                    tx = result.getTx();
//                    ty = result.getTy();
//                }
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


                //Roadrunner - turn to goal, shoot, then drive to park position
                Actions.runBlocking(
                        drive.actionBuilder(beginPose)

                                // Turn to fact goal
                                .turnTo(Math.toRadians(-135))
                                //shoot the pattern
                                .stopAndAdd(new patternLaunchAction(patternID, purpleServo,purpleShootPos,purpleDownPos,greenServo,greenShootPos,greenDownPos,2000))
                                //Move to the park position
                                .setTangent(Math.toRadians(45))  //the heading the bot will take when leaving this position
                                .splineToLinearHeading(new Pose2d(12, -18, Math.toRadians(-90)),Math.toRadians(0))
//
//                                .lineToY(56)
//                                .lineToY(25)
//                                .strafeTo(new Vector2d(shootX, shootY))
                                .build());


                flywheel.setVelocity(0); //stop the flywheel



                FtcDashboard dashboard = FtcDashboard.getInstance();
                TelemetryPacket packet = new TelemetryPacket();
                dashboard.sendTelemetryPacket(packet); // Always send the packet
                packet.fieldOverlay()
                        .setFill("blue")
                        .fillRect(-20, -20, 40, 40);

                telemetry.addData("Alliance", blackboard.get(ALLIANCE_KEY));
                telemetry.addData("time", runtime.time());
                telemetry.addData("Pattern ID", patternID);
                telemetry.addData("Seen obelisk", seenobelisk);
                telemetry.addData("Tag ID", tagID);
                telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
                telemetry.addData("Flywheel Power", flywheel.getPower());
                telemetry.update();
            }
        }
    }

    ///////////////////////////////////////////////////
    //PUBLIC CLASSES FOR ROADRUNNER ACTION DEFINITIONS
    //////////////////////////////////////////////////

    public class patternLaunchAction implements Action {
        Servo purpleServo;
        double purpleShootPos;
        double purpleDownPos;
        double servoLaunchTime;
        Servo greenServo;
        double greenShootPos;
        double greenDownPos;
        int patternID; // 21 = GPP, 22 = PPG, 23 = PGG
        ElapsedTime actionTimer;


        public patternLaunchAction(int patternID,Servo purpleServo, double purpleShootPos, double purpleDownPos,Servo greenServo,double greenShootPos,double greenDownPos,double servoLaunchTime) {
            this.purpleServo = purpleServo;
            this.patternID = patternID;
            this.servoLaunchTime = servoLaunchTime;
            this.purpleShootPos = purpleShootPos;
            this.purpleDownPos = purpleDownPos;
            actionTimer = new ElapsedTime();
        }
        //
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (actionTimer == null) {
                actionTimer = new ElapsedTime();
            }

            if (patternID == 21) {
                greenServo.setPosition(greenShootPos);
                sleep(250);
                greenServo.setPosition(greenDownPos);
                sleep(2000);
                purpleServo.setPosition(purpleShootPos);
                sleep(250);
                purpleServo.setPosition(purpleDownPos);
                sleep(2000);
                purpleServo.setPosition(purpleShootPos);
                sleep(250);
                purpleServo.setPosition(purpleDownPos);
            }
                else if (patternID == 22) {
                purpleServo.setPosition(purpleShootPos);
                sleep(250);
                purpleServo.setPosition(purpleDownPos);
                sleep(2000);
                greenServo.setPosition(greenShootPos);
                sleep(250);
                greenServo.setPosition(greenDownPos);
                sleep(2000);
                purpleServo.setPosition(purpleShootPos);
                sleep(250);
                purpleServo.setPosition(purpleDownPos);
            }
                else if (patternID == 23) {
                purpleServo.setPosition(purpleShootPos);
                sleep(250);
                purpleServo.setPosition(purpleDownPos);
                sleep(2000);
                purpleServo.setPosition(purpleShootPos);
                sleep(250);
                purpleServo.setPosition(purpleDownPos);
                sleep(2000);
                greenServo.setPosition(greenShootPos);
                sleep(250);
                greenServo.setPosition(greenDownPos);
            }
            return true;
            //return actionTimer.seconds()<servoLaunchTime; //runs the action for maximum intakeTime seconds
        }
    }

//    public class launchArtifactP implements Action {
//        Servo purpleServo;
//        double purpleShootPos;
//        double purpleDownPos;
//        double servoLaunchTime;
//        double launchPosition;
//        ElapsedTime actionTimer;
//
//
//        public launchArtifactP(Servo purpleServo, double launchPosition, double purpleShootPos, double purpleDownPos,double servoLaunchTime) {
//            this.purpleServo = purpleServo;
//            this.servoLaunchTime = servoLaunchTime;
//            this.purpleShootPos = purpleShootPos;
//            this.purpleDownPos = purpleDownPos;
//            this.launchPosition = launchPosition;
//            actionTimer = new ElapsedTime();
//        }
//        //
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (actionTimer == null) {
//                actionTimer = new ElapsedTime();
//            }
//
//            purpleServo.setPosition(purpleShootPos);
//            sleep(250);
//            purpleServo.setPosition(purpleDownPos);
//            return true;
//            //return actionTimer.seconds()<servoLaunchTime; //runs the action for maximum intakeTime seconds
//        }
//    }
//
//    public class launchArtifactG implements Action {
//        Servo greenServo;
//        double greenShootPos;
//        double greenDownPos;
//        double servoLaunchTime;
//        double launchPosition;
//        ElapsedTime actionTimer;
//
//        public launchArtifactG(Servo greenServo, double launchPosition, double greenShootPos, double greenDownPos,double servoLaunchTime) {
//            this.greenServo = greenServo;
//            this.servoLaunchTime = servoLaunchTime;
//            this.greenShootPos = greenShootPos;
//            this.greenDownPos = greenDownPos;
//            this.launchPosition = launchPosition;
//            actionTimer = new ElapsedTime();
//        }
//        //
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (actionTimer == null) {
//                actionTimer = new ElapsedTime();
//            }
//
//            greenServo.setPosition(greenShootPos);
//            sleep(250);
//            greenServo.setPosition(greenDownPos);
//            return true;
//            //return actionTimer.seconds()<servoLaunchTime; //runs the action for maximum intakeTime seconds
//        }
//    }

    ///////////////////////////////////////////////////
    //Functionality from TELEOP
    //////////////////////////////////////////////////
   
//    public void intakeSort(boolean auto) {
//
//        sortArtifact();
//
//        if(gamepad2.right_bumper){
//            intake.setPower(-1);
//            selector.setPosition(0.4);
//        }
//        if(gamepad2.right_trigger > 0.2 || auto){
//            intake.setPower(1);
//        } else {
//            intake.setPower(0);
//        }
//    }
//
//    public void sortArtifact() {
//        float sumGreenPurpleness = greenPurplenessA() + greenPurplenessB();
//        telemetry.addData("sumGreenPurpleness",sumGreenPurpleness);
//        if(sumGreenPurpleness > 120){
//            selector.setPosition(0.4 + sortOffset); //calculating the offset here isn't working - data type?
//            //selector.setPosition(0.22);
//        }
//        else if (sumGreenPurpleness < -70 ) {
//           selector.setPosition(0.4 - sortOffset);  //calculating the offset here isn't working - data type?
//           //Should work now
//            //selector.setPosition(0.58);
//        }
//        else {
//            selector.setPosition(0.4);
//        }
//        telemetry.addData("selector pos",selector.getPosition());
//    }
//
//    public float greenPurplenessA(){
//        int  greenA = colorSensorA.green();
//        int purpleA = (colorSensorA.red() + colorSensorA.blue())/2;
//        return greenA - purpleA;
//    }
//    private float greenPurplenessB(){
//        int  greenB = colorSensorB.green();
//        int purpleB = (colorSensorB.red() + colorSensorB.blue())/2;
//        return greenB - purpleB;
//    }


}
