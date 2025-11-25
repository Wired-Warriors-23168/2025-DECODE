/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


/*
 * This is our development OpMode for AUTO, the first one we're experimenting with in java for 23168
 * We will integrate to following in this mode:
 *    - Limelight3a (limelight)
 *    - Sparkfun OTOS (sensor_otos)
 *    - Mecanum drive (motors left_front, left_back, right_front, right_back)
 *    - Road Runner using a localizer for the OTOS developed by @j5155 on the FTC Discord https://github.com/jdhs-ftc
 *
 * All of the drive configuration is done via MecanumDrive.java, you do not have to manage that here.
 *
 * THIS MODE IS CONFIGURED FOR WAFFLES, NOT PANCAKE
 *
 */
//@Disabled
@Config
@Autonomous(name="AUTO_BLUE_1_NEWTEST_TRJ", group="AUTO", preselectTeleOp = "TELEOP_MAIN")
//@Disabled
public class AUTO_BLUE_1_NEWTEST_TRJ extends LinearOpMode {

    // Declare OpMode members.
    private SparkFunOTOS otos;

    private DcMotorEx flywheel;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private Servo teamLED;
    private Servo purpleServo;
    private Servo greenServo;
    private Limelight3A limelight;


    /////////////////////////////////////////////////////////////////////////
    // Declare variables
    //TODO ********** Set the Alliance Color  **************
    double feederLaunchTime;
    double launchWaitTime;
    private boolean seenobelisk = false;

    private int teamPipeline = 0; // blue auton
    private int patternID = 0;
    long servoLaunchTime = 1500; //ms
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
    private double sortOffset = 55.0/300.0;
    private double neutralPos = 140.0/300.0;
    int tagID;
    double deltaTime;
    public static final String ALLIANCE_KEY = "Alliance";
    public String colorAlliance = "BLUE";


    private ElapsedTime runtime = new ElapsedTime();
    public double lastTime = 0.0;


    ///////////////////////////////////////////////////
    //PUBLIC CLASSES FOR ROADRUNNER ACTION DEFINITIONS
    //////////////////////////////////////////////////


    /**
     * Create all the actions for AUTO in RoadRunner
     */

    /**
     * Actions for the green shooter platform
     * Cycle the Green Shooter platform from shoot to down
     * Place the green shooter platform in the Hold position
     */
    public class GreenShooter {
        private Servo greenServo;

        public GreenShooter(HardwareMap hardwareMap) {
            greenServo = hardwareMap.get(Servo.class, "greenServo");
            greenServo.setDirection(Servo.Direction.FORWARD);
        }

        public class ShootGreen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    greenServo.setPosition(greenShootPos);
                    sleep(servoLaunchTime);
                    greenServo.setPosition(greenDownPos);
                    return false;
            }
        }
        public Action shootGreen(){
            return new ShootGreen();
        }

        public class HoldGreen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                greenServo.setPosition(greenHoldPos);
                return false;
            }
        }
        public Action holdGreen(){
            return new HoldGreen();
        }
    }

    /**
     * Actions for the LED on the purple side
     * Set Alliance Color
     * Set Purple when artifact present
     * Set White when obelisk pattern is read
     */
    public class PurpleSideLED {
        private Servo teamLED;

        public PurpleSideLED(HardwareMap hardwareMap) {
            teamLED = hardwareMap.get(Servo.class, "led-light");
        }

        public class ColorAlliance implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(colorAlliance=="BLUE"){
                  teamLED.setPosition(0.600); //blue
             }
             else{
                  teamLED.setPosition(0.283);//red
             }
                return false;
            }
        }
        public Action colorAlliance(){
            return new ColorAlliance();
        }

        public class ColorPatternRead implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                teamLED.setPosition(1.0); //white
                return false;
            }
        public Action colorPatternRead(){ return new ColorPatternRead(); }

        public class ColorPurple implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                teamLED.setPosition(0.715); //purple
                return false;
            }
        }
        public Action colorPurple(){ return new ColorPurple(); }
        }

    }


    /**
     * Actions for the shooters
     * Shoot the pattern read by the limelight (from a separate action)
     * Set the shooters to the hold positions
     */
    public class Shooters {
        private Servo greenServo;
        private Servo purpleServo;


        public Shooters(HardwareMap hardwareMap) {
            purpleServo = hardwareMap.get(Servo.class, "purpleServo");
            purpleServo.setDirection(Servo.Direction.FORWARD);
            greenServo = hardwareMap.get(Servo.class, "greenServo");
            greenServo.setDirection(Servo.Direction.FORWARD);
        }

        public class ShootPattern implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //Shoot the correct pattern
                if (patternID==21){ //GPP
                    greenServo.setPosition(greenShootPos);
                    sleep(servoLaunchTime);
                    greenServo.setPosition(greenDownPos);
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);

                } else if (patternID==22){  //PGP
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);
                    greenServo.setPosition(greenShootPos);
                    sleep(servoLaunchTime);
                    greenServo.setPosition(greenDownPos);
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);
                } else if (patternID==23){  //PPG
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);
                    greenServo.setPosition(greenShootPos);
                    sleep(servoLaunchTime);
                    greenServo.setPosition(greenDownPos);
                }

                return false;
            }
        }
        public Action shootPattern(){
            return new ShootPattern();
        }

        public class HoldArtifacts implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                purpleServo.setPosition(purpleHoldPos);
                greenServo.setPosition(greenHoldPos);
                return false;
            }
        }
        public Action holdArtifacts(){
            return new HoldArtifacts();
        }
    }

    /**
     * Actions for the limelight
     * Read the pattern from the limelight
     */

    public class Limelight {
        private Limelight3A limelight;
        ElapsedTime readTime;

        public Limelight(HardwareMap hardwareMap){
            limelight = hardwareMap.get(Limelight3A.class,"limelight");
        }

        public class ReadPattern implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                limelight.start();
                limelight.pipelineSwitch(2); //the obelisk pipeline (we don't need anything else in AUTO)

                //Read the limelight and determine the pattern, then stop the limelight
                LLResult result = limelight.getLatestResult();

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial != null) {
                        tagID = fiducial.getFiducialId();
                    }
                }
                if (tagID != 0  && !seenobelisk) {
                    seenobelisk = true;
                    patternID = tagID; // save pattern
//                    sleep(50);
                    tagID = 0;
//                    limelight.pipelineSwitch(teamPipeline);
                }
                //Update telemetry to show obelisk was read
                telemetry.addData("Pattern ID", patternID);
                telemetry.addData("Seen obelisk", seenobelisk);
                telemetry.addData("Tag ID", tagID);
                telemetry.update();

                if (patternID == 0){    //keep actively looking for the pattern until it's seen
                    return true;
                } else {
                    limelight.stop();
                    return false;
                }
            }
        }
        public Action readPattern(){
            return new ReadPattern();
        }

    }


    /**
     * Actions for the intake
     * Power on
     * Power off
     */
     public class Intake {
        private DcMotor intake;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "motor-intake");
            intake.setDirection(DcMotor.Direction.REVERSE);
        }

        public class IntakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(1);
                return false;
            }
        }

        public Action intakeOn() {
            return new IntakeOn();
        }

        public class IntakeOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(0);
                return false;
            }
        }

        public Action intakeOff() {
            return new IntakeOff();
        }
    }


    /**
     * Actions for selector
     * Includes color sensor calls
     */

    public class Sort {
        private Servo selector;
        private ColorSensor colorSensorA;
        private ColorSensor colorSensorB;

        public Sort(HardwareMap hardwareMap) {
            selector = hardwareMap.get(Servo.class, "servo-selector");
            colorSensorA = hardwareMap.get(ColorSensor.class, "sensor-color-a");
            colorSensorB = hardwareMap.get(ColorSensor.class, "sensor-color-b");
            selector.setDirection(Servo.Direction.FORWARD);
        }

        public class SortArtifact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                float sumGreenPurpleness = greenPurplenessA() + greenPurplenessB();
//                telemetry.addData("sumGreenPurpleness",sumGreenPurpleness);
                if(sumGreenPurpleness > 50){
                    selector.setPosition(neutralPos - sortOffset); //updated zero position for new print (shaft was turned...)
                    //selector.setPosition(0.22);
                }
                else if (sumGreenPurpleness < -50 ) {
                    selector.setPosition(neutralPos + sortOffset);  //updated zero position for new print (shaft was turned...)
                    //Should work now
                    //selector.setPosition(0.58);
                }
                else {
                    selector.setPosition(neutralPos);
                }
                telemetry.addData("selector pos",selector.getPosition());
                telemetry.addData("sumGreenPurpleness",sumGreenPurpleness);

                return false; // run to 8.82s for 25 max speed
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
        }

        public Action sortArtifact() {
            return new SortArtifact();
        }
    }


    /**
     * Actions for flywheel
     * Set far velocity
     * Set close velocity
     * Power off
     */
    public class Flywheel {
        private DcMotorEx flywheel;

        public Flywheel(HardwareMap hardwareMap) {
            flywheel = hardwareMap.get(DcMotorEx.class, "motor-flywheel");
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        }

        public class SetFlywheelFar implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flywheel.setVelocity(farVelocity);
                return false;
            }
        }
        public Action setFlywheelFar() {
            return new SetFlywheelFar();
        }

        public class SetFlywheelClose implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flywheel.setVelocity(closeVelocity);
                return false;
            }
        }
        public Action setFlywheelClose() {
            return new SetFlywheelClose();
        }

        public class SetFlywheelStop implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flywheel.setPower(0.0);
                return false;
            }
        }
        public Action setFlywheelStop() {
            return new SetFlywheelStop();
        }
    }

//    /**
//     * Actions for the OTOS
//     * Read the OTOS for use in setting new start position
//     */
//    public class FieldPosition{
//        private SparkFunOTOS otos;
//        public SparkFunOTOS.Pose2D pos;
//
//        public FieldPosition(HardwareMap hardwareMap){
//            otos = hardwareMap.get(SparkFunOTOS.class, "sensor-otos");
//        }
//        public class ReadPosition implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                pos = otos.getPosition(); //Read OTOS Pose for next move
//                return false;
//            }
//        }
//        public Action readPosition(){
//            return new ReadPosition();
//        }
//    }



    @Override
    public void runOpMode() {

        //Set Hardware Map
        flywheel = hardwareMap.get(DcMotorEx.class, "motor-flywheel");
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor-otos");
        greenServo = hardwareMap.get(Servo.class, "greenServo");
        purpleServo = hardwareMap.get(Servo.class, "purpleServo");
        teamLED = hardwareMap.get(Servo.class, "led-light");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");

        //initDevices(); // Initialize all motors, servos, sensors

        // Establishing the direction and mode for the motors
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
//        greenServo.setDirection(Servo.Direction.FORWARD);
//        purpleServo.setDirection(Servo.Direction.FORWARD);

//        limelight.start();
//        limelight.pipelineSwitch(2);

        //set artifact holding positions
        flywheel.setVelocity(0);
        greenServo.setPosition(greenHoldPos);
        purpleServo.setPosition(purpleHoldPos);

        //Set the LED to green in INIT mode
        teamLED.setPosition(0.4);//green

        telemetry.setMsTransmissionInterval(11);

        //Initialize the mechanism Actions
//        GreenShooter greenServo = new GreenShooter(hardwareMap);
//        PurpleShooter purpleServo = new PurpleShooter(hardwareMap);
        Limelight limelight = new Limelight(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Sort sort = new Sort(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Shooters shooters = new Shooters(hardwareMap);
        PurpleSideLED purpleSideLED = new PurpleSideLED(hardwareMap);
//        FieldPosition fieldPosition = new FieldPosition(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //TODO *********** Set the starting pose for the robot based on the alliance start position,
        // X and Y in INCHES from the center of the field, heading in RADIANS (or convert DEGREES to
        // RADIANS by multiplying the value in DEGREES by Math.PI/180
        Pose2d beginPose = new Pose2d(-60.75, -38.75, Math.toRadians(180)); //NEW STARTING POSITION

        //Set AUTO waypoints
        Pose2d obeliskPose = new Pose2d(-30,-30,Math.toRadians(145));  //pose to read the obelisk
        double shootHeading = -135;
        Pose2d shootPose = new Pose2d(-20,-20,Math.toRadians(shootHeading));    //pose to shoot the pattern
        Pose2d intakePose1 = new Pose2d(-30,-30,Math.toRadians(-135));  //pose to intake artifacts from first row
//        Pose2d intakePose2 = new Pose2d(-30,-30,Math.toRadians(-135));  //pose to intake artifacts from second row
        Pose2d endPose = new Pose2d(12,-18,Math.toRadians(90));      //pose at end of AUTO
        double firstArtifact = -36;     //Y-position of the first artifact in the row
        double secondArtifact = -41;    //Y-position of the second artifact in the row
        double thirdArtifact = -46;     //Y-position of the third artifact in the row

        //Instantiate the roadrunner Mecanum drive (via the OTOS localizer)
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.localizer.setPose(beginPose);  //may have to do this for the new RR version per https://community.sparkfun.com/t/sparkfun-otos-with-ftc-inital-pose-always-0/67256

        /**
         * BUILD TRAJECTORIES
         * 1. move to obelisk position
         * 2. Turn to goal
         * 3. Drive to intake position
         * 4. Move forward for first artifact
         * 5. Move forward for second artifact
         * 6. Move forward for third artifact
         * 7. Move to shoot pose
         * 8. Move to end pose
         */
        //https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/

        //TODO learn how to use the .fresh() modifier so the trajectory starts where the last one ended
        // https://github.com/acmerobotics/road-runner-quickstart/issues/376

        TrajectoryActionBuilder trjObelisk = drive.actionBuilder(beginPose)
                //Spline to the obelisk-reading pose, then transition to the shooting pose (hopefully we read the obelisk in this time)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-30,-30,Math.toRadians(145)),Math.toRadians(0))
//                .waitSeconds(2)
//                .turnTo(Math.toRadians(-135))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-20,-20,Math.toRadians(-135)),Math.toRadians(45))
                .waitSeconds(2);

//        Action trjShoot = trjObelisk.endTrajectory().fresh()
//                .turnTo(shootHeading)
//                .build();

        TrajectoryActionBuilder trjShoot = drive.actionBuilder(obeliskPose)
                //Turn to the shooting pose
                .turnTo(Math.toRadians(-135))
                .waitSeconds(2);

        TrajectoryActionBuilder trjIntakeAndShoot = drive.actionBuilder(shootPose)
                //Spline to the first artifact row
//                .fresh()
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-18.25,-24.25,Math.toRadians(-90)),Math.toRadians(-90))
                .waitSeconds(2)
                //Move forward slowly to intake the first artifact and wait for sorting
//                .fresh()
                .lineToY(-36,
                        // override velocity constraint - slow down the move
                        new TranslationalVelConstraint(10))
                .waitSeconds(2)

                //Move forward slowly to intake the second artifact and wait for sorting
//                .fresh()
                .lineToY(-41,
                        // override velocity constraint - slow down the move
                        new TranslationalVelConstraint(10))
                .waitSeconds(2)

                //Move forward slowly to intake the third artifact and wait for sorting
//                .fresh()
                .lineToY(-46,
                        // override velocity constraint - slow down the move
                        new TranslationalVelConstraint(10))
                .waitSeconds(2)

                //Spline to the shooting pose, back up to full speed
//                .fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-30,-30,Math.toRadians(-135)),Math.toRadians(45),
                        // override velocity constraint - set back to full
                        new TranslationalVelConstraint(50.0))
                .waitSeconds(2)
                ;

//        TrajectoryActionBuilder trjMove1 = drive.actionBuilder(intakePose1).fresh()
//                .lineToY(firstArtifact,
//                        // override velocity constraint - slow down the move
//                new TranslationalVelConstraint(20.0)
//                );
//
//        TrajectoryActionBuilder trjMove2 = drive.actionBuilder(intakePose1).fresh()
//                .lineToY(secondArtifact,
//                        // override velocity constraint - slow down the move
//                        new TranslationalVelConstraint(20.0)
//                );
//
//        TrajectoryActionBuilder trjMove3 = drive.actionBuilder(intakePose1).fresh()
//                .lineToY(thirdArtifact,
//                        // override velocity constraint - slow down the move
//                        new TranslationalVelConstraint(20.0)
//                );
//
//        TrajectoryActionBuilder trjShootPose = drive.actionBuilder(intakePose1).fresh()
//                .splineToLinearHeading(shootPose,Math.toRadians(0),
//                        // override velocity constraint - set back to full
//                        new TranslationalVelConstraint(50.0)
//                );
        TrajectoryActionBuilder trjEndPose = drive.actionBuilder(shootPose)
                //Spline to the end pose
                .fresh()
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(12,-18,Math.toRadians(90)),Math.toRadians(0),
                        // only override velocity constraint - set back to full
                        new TranslationalVelConstraint(50.0)
                );


        ////////////////////////////////////////////////////////////////////////////////////
        // Wait for the game to start (driver presses START)
        ////////////////////////////////////////////////////////////////////////////////////
        waitForStart();

        if(isStopRequested()) return;

//        //TODO add actions for the LEDs
//        if(colorAlliance=="BLUE"){
//            teamLED.setPosition(0.600); //blue
//        }
//        else{
//            teamLED.setPosition(0.283);//red
//        }

        //flywheel.setVelocity(closeVelocity);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                purpleSideLED.colorAlliance(),     //set purple-side LED to alliance color
//                                greenSideLED.colorAlliance(),     //set green-side LED to alliance color
                                flywheel.setFlywheelClose()         //turn on flywheel
                        ),
                        new ParallelAction(                 //move to obelisk position and read the pattern
                                trjObelisk.build(),
                                limelight.readPattern()
                        ),
//                        trjShoot.build(),                   //turn to the goal (currently part of the obelisk trajectory)
//                        trjObelisk.build(),

                        shooters.shootPattern(),            //shoot the pattern
                        new ParallelAction(
                                intake.intakeOn(),          //turn on the intake
                                sort.sortArtifact(),        //sort artifacts in parallel
                                trjIntakeAndShoot.build()   //move to the start of the first row of artifacts, then slowly move forward one-by-one
                        ),
                        trjIntakeAndShoot.build(),
//                        intake.intakeOff(),                 //turn off intake
//                        shooters.shootPattern(),            //shoot the pattern
//                        flywheel.setFlywheelStop(),         //stop the flywheel
                        trjEndPose.build()                  //drive to end pose
                )
        );
    }

}