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
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;


/**
 * EXAMPLE ROADRUNNER AUTO OPMODE
 * Developed based on the 2024 and 2025 seasons, designed to be a basis for future AUTO development
 * Copy this example and modify from there, but please do not overwrite this example!
 *
 * There are three main sections that are specific to Roadrunner (RR) and the pathing:
 *      1. Actions - functions that we define to move actuators, read sensors, etc.
 *      2. Trajectory Definitions - trajectories are defined and then combined later.  The idea is that
 *         you can combine several trajectories and actions in series or parallel to make the bot drive and move.
 *      3. The Build - this is where you define how hte bot will step through the trajectories and actions
 *         you defined.
 *
 * Other components that are not related to RR but are covered here:
 *      - blackboard - saving information to memory for use in TELEOP
 *      - managing alliance colors
 *      - creating as few AUTO files as possible to avoid repetition and "refactoring"
 *
 * OUR ASSUMED ROBOT
 * The bot used for this example doesn't exist.  At least not yet.  you might want to make the bot and then
 * use it to learn how to code AUTOs with RR.  Here is our assumed hardware:
 *      - Mecanum drive
 *      - Sparkfun OTOS odometry
 *      - One REV ultraplanetary hex motor with encoder (for velocity or position control)
 *      - One Servo
 *      - One REV Color Sensor V3
 *      - One goBilda LED
 *  It's up to you to configure the robot and control hub correctly, but we'll assume names for the devices.
 *
 *  ORDER OF OPERATIONS
 *      1. Install Roadrunner
 *      2. Tune Roadrunner (you'll be filling out information in OTOSLocalizer and MecanumDrive)
 *      3. Install the MeepMeep path visualizer to test out your trajectories virtually before you crash the bot into stuff.
 *      4. Code the Actions and run the Build with only the actions to verify they do what you want.
 *         HINT: you can use the same code from TELEOP, but you have to create IF statements or careful FOR statements
 *         because this is a linear opmode, not a loop like TELEOP.  You could also use Actions in TELEOP, that is
 *         apparently a thing, though we have never done it.
 *      5. Code your trajectories in MeepMeep and test them.
 *      6. Code your trajectories here and run them with large wait times between steps.
 *      7. Tune the waypoints: there is always some error introduced as the bot moves, plus your tuning has to be really
 *         precise to get the same result on the actual bot as you did in MeepMeep.  You can also retune the Feedforward
 *         and Feedback controllers in Roadrunner again now if you need to.
 *      8. Combine it all together and fine-tune to squeeze out as much performance as possible.
 *      9. Advanced tuning:
 *          a. Action timers - reduce the time to the absolute minimum
 *          b. Velocity constraints - set different velocities for different trajectories to speed up simple movements
 *          c. Acceleration constraints - set different accelerations and decelerations to smooth out control
 *          HINT: there is a saying, "Slow is smooth, smooth is fast."  Sometimes slower, controlled motion
 *          will give you a better AUTO in the end.
 *
 **/
@Disabled
@Config
@Autonomous(name="AUTO_BLUE_CLOSE", group="AUTO", preselectTeleOp = "TELEOP_MAIN")
//@Disabled
public class AUTO_EXAMPLE extends LinearOpMode {

    // Declare OpMode members.
    private SparkFunOTOS otos;

    private DcMotorEx TestMotor;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private Servo testLED;
    private Servo teamLEDGreen;
    private Servo testServo;
    private Servo greenServo;
    private CRServo purpleCRServo;
    private Limelight3A limelight;
    private DistanceSensor purpleDistanceSensor;
    private DistanceSensor greenDistanceSensor;


    /////////////////////////////////////////////////////////////////////////
    // Declare variables
    //TODO ********** Set the Alliance Color  **************
    double feederLaunchTime;
    double launchWaitTime;
    private boolean seenobelisk = false;

    private int teamPipeline = 0; // blue auton
    private int patternID = 0;
    long servoLaunchTime = 250; //ms
    long shootWaitTime = 1000; //ms
    long shootSecondWaitTime = 3000; //ms
    long sortActionTime = 10;//sec
    private double greenShootPos = 0.15;  //was 0.2467
    private double purpleShootPos = 0.22;  //was 0.0933
    private double greenDownPos = 0.32;
    private double purpleDownPos = 0.02;
    private double greenHoldPos = 0.27;
    private double purpleHoldPos = 0.09;
    private double purpleCRPower = 1.0; //the power setting for the purple continuous servo
    private double farVelocity = 1460;
    private double closeVelocity = 1340;
    private double idleVelocity = 600;
    private double targetVelocity = 600;
        private PIDFCoefficients flywheelpid = new PIDFCoefficients(550, 0.0, 0.0, 0.0);
    private double sortOffset = 55.0/300.0;
    private double neutralPos = 140.0/300.0;
    int tagID;
    double deltaTime;
    public static final String ALLIANCE_KEY = "Alliance";
//    public static final String POSE_X_KEY = 0;
//    public static final String POSE_Y_KEY = "Alliance";
//    public static final String POSE_H_KEY = "Alliance";
    public String colorAlliance = "BLUE";
    public boolean isAllianceBlue = true;
    public double ledINITStatus = 0.500;        //green
    public double ledREADYStatus= 1.0;                 //white
    public double ledGreenArtifact = 0.515;     //turquoise green
    public double ledPurpleArtifact = 0.722;    //purple
    public double ledBlueAlliance = 0.600;      //blue
    public double ledRedAlliance = 0.283;       //red




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
     * Actions for the green shooter platform
     * Cycle the Green Shooter platform from shoot to down
     * Place the green shooter platform in the Hold position
     */
    public class PurpleShooter {
        private Servo testServo;

        public PurpleShooter(HardwareMap hardwareMap) {
            testServo = hardwareMap.get(Servo.class, "Servo");
            testServo.setDirection(Servo.Direction.FORWARD);
        }

        public class ShootPurple implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                greenServo.setPosition(purpleShootPos);
                sleep(servoLaunchTime);
                testServo.setPosition(purpleDownPos);
                return false;
            }
        }
        public Action shootPurple(){
            return new ShootPurple();
        }

        public class HoldPurple implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                testServo.setPosition(purpleHoldPos);
                return false;
            }
        }
        public Action holdPurple(){
            return new HoldPurple();
        }
    }

    /**
     * Actions for the LEDs on either side
     * Set Alliance Color
     * Set Purple and Green when called (both sides, not individual)
     * Set White when obelisk pattern is read
     */
    public class TeamLEDs {
        private Servo testLED;
        private Servo teamLEDGreen;

        public TeamLEDs(HardwareMap hardwareMap) {
            testLED = hardwareMap.get(Servo.class, "led-light");
            teamLEDGreen = hardwareMap.get(Servo.class, "led-light-green");
        }

        public class ColorAlliance implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(isAllianceBlue){
                  testLED.setPosition(ledBlueAlliance); //blue
                    teamLEDGreen.setPosition(ledBlueAlliance); //blue
             }
             else{
                  testLED.setPosition(ledRedAlliance);//red
                    teamLEDGreen.setPosition(ledRedAlliance);//red
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
                testLED.setPosition(ledREADYStatus); //white
                teamLEDGreen.setPosition(ledREADYStatus); //white

                return false;
            }
        public Action colorPatternRead(){ return new ColorPatternRead(); }

        public class ColorSides implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                testLED.setPosition(ledPurpleArtifact); //purple
                teamLEDGreen.setPosition(ledGreenArtifact); //green
                return false;
            }
        }
        public Action colorSides(){ return new ColorSides(); }
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
        ElapsedTime shootTimer;

        public Shooters(HardwareMap hardwareMap) {
            purpleServo = hardwareMap.get(Servo.class, "purpleServo");
            purpleServo.setDirection(Servo.Direction.FORWARD);
            greenServo = hardwareMap.get(Servo.class, "greenServo");
            testLED = hardwareMap.get(Servo.class, "led-light");
            teamLEDGreen = hardwareMap.get(Servo.class, "led-light-green");
            greenServo.setDirection(Servo.Direction.FORWARD);
        }

        public class ShootPattern implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //Shoot the correct pattern
//                if (patternID==21){ //GPP
//                    shootGreen();
//                    shootPurple();
//                    shootPurple();
//                } else if (patternID==22){  //PGP
//                    shootPurple();
//                    shootGreen();
//                    shootPurple();
//                } else if (patternID==23){  //PPG
//                    shootPurple();
//                    sleep(shootWaitTime);
//                    shootPurple();
//                    sleep(shootWaitTime);
//                    shootGreen();
//                }
//
                if (patternID==21){ //GPP
                    testLED.setPosition(ledREADYStatus);
                    teamLEDGreen.setPosition(ledGreenArtifact);
                    greenServo.setPosition(greenShootPos);
                    sleep(servoLaunchTime);
                    greenServo.setPosition(greenDownPos);
                    teamLEDGreen.setPosition(ledREADYStatus);
                    testLED.setPosition(ledPurpleArtifact);
                    sleep(shootWaitTime);
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);
                    sleep(shootSecondWaitTime);
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);
                    testLED.setPosition(ledREADYStatus);

                } else if (patternID==22){  //PGP
                    teamLEDGreen.setPosition(ledREADYStatus);
                    testLED.setPosition(ledPurpleArtifact);
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);
                    testLED.setPosition(ledREADYStatus);
                    teamLEDGreen.setPosition(ledGreenArtifact);
                    sleep(shootWaitTime);
                    greenServo.setPosition(greenShootPos);
                    sleep(servoLaunchTime);
                    greenServo.setPosition(greenDownPos);
                    teamLEDGreen.setPosition(ledREADYStatus);
                    testLED.setPosition(ledPurpleArtifact);
                    sleep(shootWaitTime);
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);
                    testLED.setPosition(ledREADYStatus);
                } else if (patternID==23){  //PPG
                    teamLEDGreen.setPosition(ledREADYStatus);
                    testLED.setPosition(ledPurpleArtifact);
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);
                    sleep(shootSecondWaitTime);
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);
                    testLED.setPosition(ledREADYStatus);
                    teamLEDGreen.setPosition(ledGreenArtifact);
                    sleep(shootWaitTime);
                    greenServo.setPosition(greenShootPos);
                    sleep(servoLaunchTime);
                    greenServo.setPosition(greenDownPos);
                    teamLEDGreen.setPosition(ledREADYStatus);
                }

                return false;
            }
            public boolean shootGreen() {
                if (shootTimer == null) {
                    shootTimer = new ElapsedTime();
                }

                testLED.setPosition(1.0); //white
                    teamLEDGreen.setPosition(1.0); //white
                if (TestMotor.getVelocity()>= closeVelocity - 40 && TestMotor.getVelocity()< closeVelocity + 40) {
                    testLED.setPosition(ledREADYStatus); //green
                    teamLEDGreen.setPosition(ledREADYStatus); //green
                    greenServo.setPosition(greenShootPos);
                    sleep(servoLaunchTime);
                    greenServo.setPosition(greenDownPos);
                }
                testLED.setPosition(ledINITStatus); //white
                    teamLEDGreen.setPosition(ledINITStatus); //white
//                return shootGreen();
//                if(greenDistanceSensor.getDistance(DistanceUnit.INCH) < 3){
//                    return false;
//                } else{
//                    return true;
//                }
                return shootTimer.seconds()<=3;
            }
            public boolean shootPurple() {
                if (shootTimer == null) {
                    shootTimer = new ElapsedTime();
                }
                testLED.setPosition(ledINITStatus); //white
                    teamLEDGreen.setPosition(ledINITStatus); //white
                if (TestMotor.getVelocity()>= closeVelocity - 40 && TestMotor.getVelocity()< closeVelocity + 40) {
                    testLED.setPosition(ledPurpleArtifact); //purple
                    purpleServo.setPosition(purpleShootPos);
                    sleep(servoLaunchTime);
                    purpleServo.setPosition(purpleDownPos);
                }
                testLED.setPosition(ledINITStatus); //white
                    teamLEDGreen.setPosition(ledINITStatus); //white
//                return shootPurple();
//                if(purpleDistanceSensor.getDistance(DistanceUnit.INCH) < 3){
//                    return false;
//                } else{
//                    return true;
//                }
                return shootTimer.seconds()<=3;
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
                    //Set team LEDs so the status can be seen
                    testLED.setPosition(ledREADYStatus);//white
                }
                //Update telemetry to show obelisk was read
                telemetry.addData("Pattern ID", patternID);
                telemetry.addData("Seen obelisk", seenobelisk);
                telemetry.addData("Tag ID", tagID);
                telemetry.update();

                if (patternID == 0){    //keep actively looking for the pattern until it's seen
                    return true;
                } else {
                    blackboard.put("PATTERN_ID_KEY",patternID); //save the pattern to the blackboard for init in TELEOP
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
        private float sumGreenPurpleness;
        ElapsedTime sortTimer;

        public Sort(HardwareMap hardwareMap) {
            selector = hardwareMap.get(Servo.class, "servo-selector");
            colorSensorA = hardwareMap.get(ColorSensor.class, "sensor-color-a");
            colorSensorB = hardwareMap.get(ColorSensor.class, "sensor-color-b");
            greenServo = hardwareMap.get(Servo.class, "greenServo");
            testServo = hardwareMap.get(Servo.class, "purpleServo");
            purpleDistanceSensor = hardwareMap.get(DistanceSensor.class, "purpleDistanceSensor");
            greenDistanceSensor = hardwareMap.get(DistanceSensor.class, "greenDistanceSensor");
            selector.setDirection(Servo.Direction.FORWARD);
        }

        public class SortArtifact implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (sortTimer == null) {
                    sortTimer = new ElapsedTime();
                }

                if (sortTimer.seconds()<=sortActionTime) {

                    if(greenDistanceSensor.getDistance(DistanceUnit.INCH) < 5){
                        greenServo.setPosition(greenHoldPos);
                    }
                    else{
                        greenServo.setPosition(greenDownPos);
                    }
                    if(purpleDistanceSensor.getDistance(DistanceUnit.INCH) < 5){
                        testServo.setPosition(purpleHoldPos);
                    }
                    else {
                        testServo.setPosition(purpleDownPos);
                    }

                    sumGreenPurpleness = greenPurplenessA() + greenPurplenessB();
                    if (sumGreenPurpleness > 50) {
                        selector.setPosition(neutralPos - sortOffset); //updated zero position for new print (shaft was turned...)
                        //selector.setPosition(0.22);
                    } else if (sumGreenPurpleness < -50) {
                        selector.setPosition(neutralPos + sortOffset);  //updated zero position for new print (shaft was turned...)
                        //Should work now
                        //selector.setPosition(0.58);
                    } else {
                        selector.setPosition(neutralPos);
                    }
                    telemetry.addData("selector pos", selector.getPosition());
                    telemetry.addData("sumGreenPurpleness", sumGreenPurpleness);
                    telemetry.update();
                }
                if (sortTimer.seconds()<=sortActionTime){
                    return true;
                } else{
                    sortTimer.reset();
                    return false;
                }
//                return sortTimer.seconds()<=sortActionTime;  // run to 8.82s for 25 max speed
//                return false; // run to 8.82s for 25 max speed

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
     * Actions for purple side continuous rotating servo
     * Set power
     * Power off
     */

    public class PurpleCRServo {
        private CRServo purpleCRServo;

        public PurpleCRServo(HardwareMap hardwareMap) {
            purpleCRServo = hardwareMap.get(CRServo.class, "servo-conveyor-purple");
            purpleCRServo.setDirection(CRServo.Direction.FORWARD);
        }

        public class SetOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                purpleCRServo.setPower(purpleCRPower);
                return false;
            }
        }
        public Action setOn() {
                return new SetOn();
            }
        public class SetOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                purpleCRServo.setPower(0);
                return false;
            }
        }
        public Action setOff() {
                return new SetOff();
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
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelpid);
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
        TestMotor = hardwareMap.get(DcMotorEx.class, "motor-flywheel");
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor-otos");
        greenServo = hardwareMap.get(Servo.class, "greenServo");
        testServo = hardwareMap.get(Servo.class, "purpleServo");
        testLED = hardwareMap.get(Servo.class, "led-light");
        teamLEDGreen = hardwareMap.get(Servo.class, "led-light-green");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        purpleDistanceSensor = hardwareMap.get(DistanceSensor.class, "purpleDistanceSensor");
        greenDistanceSensor = hardwareMap.get(DistanceSensor.class, "greenDistanceSensor");

        //initDevices(); // Initialize all motors, servos, sensors

        //Set both LEDs to alliance color to indicate INIT has started
        testLED.setPosition(ledINITStatus);
        teamLEDGreen.setPosition(ledINITStatus);

        // Establishing the direction and mode for the motors
        TestMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TestMotor.setDirection(DcMotorEx.Direction.REVERSE);
        greenServo.setDirection(Servo.Direction.FORWARD);
        testServo.setDirection(Servo.Direction.FORWARD);

//        limelight.start();
//        limelight.pipelineSwitch(2);

        //set artifact holding positions
        TestMotor.setVelocity(0);
        greenServo.setPosition(greenHoldPos);
        testServo.setPosition(purpleHoldPos);

        telemetry.setMsTransmissionInterval(11);

        //Store the alliance color to memory for use in TELEOP
        blackboard.put(ALLIANCE_KEY, isAllianceBlue);

        //Set the green side LEDs to white in INIT mode after hold
        testLED.setPosition(ledINITStatus);
        teamLEDGreen.setPosition(ledREADYStatus);//white

        //Initialize the mechanism Actions
        GreenShooter greenShooter = new GreenShooter(hardwareMap);
        PurpleShooter purpleShooter = new PurpleShooter(hardwareMap);
        Limelight limelight = new Limelight(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Sort sort = new Sort(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Shooters shooters = new Shooters(hardwareMap);
        TeamLEDs teamLEDs = new TeamLEDs(hardwareMap);
        PurpleCRServo purpleCRServo = new PurpleCRServo(hardwareMap);
//        FieldPosition fieldPosition = new FieldPosition(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //TODO *********** Set the starting pose for the robot based on the alliance start position,
        // X and Y in INCHES from the center of the field, heading in RADIANS (or convert DEGREES to
        // RADIANS by multiplying the value in DEGREES by Math.PI/180
        Pose2d beginPose = new Pose2d(-60.75, -38.75, Math.toRadians(180)); //NEW STARTING POSITION

        //Set AUTO waypoints
        Vector2d obeliskVector = new Vector2d(-12,-12);  //pose to read the obelisk
        double obeliskHeading = Math.toRadians(155);  //heading to read the obelisk
        Pose2d obeliskPose = new Pose2d(-12,-12,obeliskHeading); //pose to read the obelisk

        Pose2d shootPose = new Pose2d(-14,-14,Math.toRadians(-135));    //pose to shoot the pattern

        Pose2d intakePose1 = new Pose2d(-12.5,-28,Math.toRadians(-90));  //pose to start intake artifacts from first row
        Pose2d intakePickup1 = new Pose2d(-12.5,-49,Math.toRadians(-90));  //pose to end intake artifacts from first row

        Pose2d intakePose2 = new Pose2d(12.5,-29,Math.toRadians(-90));  //pose to start intake artifacts from second row
        Pose2d intakePickup2 = new Pose2d(12.5,-49,Math.toRadians(-90));  //pose to end intake artifacts from second row

        Pose2d endPose = new Pose2d(11,-29,Math.toRadians(-90));      //pose at end of AUTO


        //Instantiate the roadrunner Mecanum drive (via the OTOS localizer)
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.localizer.setPose(beginPose);  //may have to do this for the new RR version per https://community.sparkfun.com/t/sparkfun-otos-with-ftc-inital-pose-always-0/67256

        /**
         * BUILD TRAJECTORIES
         * https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/
         */

        //Setting the base velocity and angular velocity constraints
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 10.0);


        //Leave starting position, drive nad turn to obelisk to read pattern and continue to aiming at goal
        TrajectoryActionBuilder trjObelisk = drive.actionBuilder(beginPose)
//                .waitSeconds(2) //Wait to spin-up flywheel

                //Spline to the obelisk-reading pose, then transition to the shooting pose (hopefully we read the obelisk in this time)
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(obeliskPose,Math.toRadians(0))
//                .splineToConstantHeading(obeliskVector,obeliskHeading)

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(shootPose,Math.toRadians(45))
                .waitSeconds(1); //Wait to spin-up flywheel


        //Spline to start of first row of artifacts then move forward slowly to intake the artifacts, then return to the shooting pose
        TrajectoryActionBuilder trjIntakeAndShoot1 = drive.actionBuilder(shootPose)
                //Spline to the first artifact row
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(intakePose1,Math.toRadians(-90))
                //Move forward slowly to intake the first artifact and wait for sorting
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(intakePickup1,Math.toRadians(-90),
                        // override velocity constraint - slow down the move
                        new TranslationalVelConstraint(2.5),
                        new ProfileAccelConstraint(-10.0, 10.0));

//                //Spline to the shooting pose, back up to full speed
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(shootPose,Math.toRadians(45));

        //Spline to start of second row of artifacts then move forward slowly to intake the artifacts, then return to the shooting pose
        TrajectoryActionBuilder trjIntakeAndShoot2 = drive.actionBuilder(shootPose)
                //Spline to the first artifact row
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(intakePose2,Math.toRadians(-90))
                //Move forward slowly to intake the first artifact and wait for sorting
                .splineToLinearHeading(intakePickup2,Math.toRadians(-90),
                        // override velocity constraint - slow down the move
                        new TranslationalVelConstraint(3),
                        new ProfileAccelConstraint(-10.0, 10.0))

                //Spline to the shooting pose, back up to full speed
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(shootPose,Math.toRadians(45));

        TrajectoryActionBuilder trjShootPose1 = drive.actionBuilder(intakePickup1)
                //Spline to the shooting pose, back up to full speed
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(shootPose,Math.toRadians(45));

        TrajectoryActionBuilder trjShootPose2 = drive.actionBuilder(intakePickup2)
                //Spline to the shooting pose, back up to full speed
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(shootPose,Math.toRadians(45));

        //Spline to start of second row of artifacts then move forward slowly to intake the artifacts, then return to the shooting pose
        TrajectoryActionBuilder trjIntakeAndEnd = drive.actionBuilder(shootPose)
                //Spline to the first artifact row
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(endPose,Math.toRadians(-90));


        //Drive to end position
        TrajectoryActionBuilder trjEndPose = drive.actionBuilder(shootPose)
                //Spline to the end pose
                .fresh()
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(endPose,Math.toRadians(-90),
                        // only override velocity constraint - set back to full
                        new TranslationalVelConstraint(50.0)
                );

        //Call end position as end of the trajectory
        Action trajectoryActionCloseout = trjEndPose.endTrajectory().fresh()
                .build();



        ////////////////////////////////////////////////////////////////////////////////////
        // Wait for the game to start (driver presses START)
        ////////////////////////////////////////////////////////////////////////////////////
        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(
                    new SequentialAction(
                            greenShooter.holdGreen(),           //set the green shooter to hold position so artifact is ready to shoot after next move
                            purpleShooter.holdPurple(),         //set the purple shooter to hold position so artifact is ready to shoot after next move
                            purpleCRServo.setOn(),              //turn on the purple side CRServo to advance the second purple ball
                            flywheel.setFlywheelClose(),        //turn on flywheel
                            new ParallelAction(                 //move to obelisk position and read the pattern
                                    trjObelisk.build(),
                                    limelight.readPattern()
                            ),
                            shooters.shootPattern(),            //shoot the pattern
                            teamLEDs.colorAlliance(),           //set team LEDs to alliance color
                            intake.intakeOn(),                  //turn on the intake
                            new ParallelAction(
                                    sort.sortArtifact(),        //sort artifacts in parallel
                                    trjIntakeAndShoot1.build()   //move to the start of the FIRST row of artifacts, then slowly move forward
                            ),
                            greenShooter.holdGreen(),           //set the green shooter to hold position so artifact is ready to shoot after next move
                            purpleShooter.holdPurple(),         //set the purple shooter to hold position so artifact is ready to shoot after next move
                            trjShootPose1.build(),              //drive to shooting pose
                            intake.intakeOff(),                 //turn off intake
                            shooters.shootPattern(),            //shoot the pattern
//                            teamLEDs.colorAlliance(),           //set team LEDs to alliance color
//                            intake.intakeOn(),
//                            new ParallelAction(
//                                    sort.sortArtifact(),        //sort artifacts in parallel
//                                    trjIntakeAndEnd.build()   //move to the start of the SECOND row of artifacts, then slowly move forward
//                            ),
//                            intake.intakeOff(),                 //turn off intake
//                            shooters.shootPattern(),            //shoot the pattern
                            teamLEDs.colorAlliance(),           //set team LEDs to alliance color
                            purpleCRServo.setOff(),             //stop the purple side CRServo
                            trjEndPose.build(),                 //drive to end pose
                            flywheel.setFlywheelStop(),         //stop the flywheel
                            trajectoryActionCloseout            //STOP
                    )
        );


        //Store the alliance color to memory for use in TELEOP
        blackboard.put(ALLIANCE_KEY, isAllianceBlue);
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        blackboard.put("POSE_X_KEY",pos.x);
        blackboard.put("POSE_Y_KEY",pos.y);
        blackboard.put("POSE_H_KEY",pos.h);

        telemetry.addLine();
        telemetry.addData("OTOS Data", "X: (%.1f), Y: (%.1f), H: (%.2f)", pos.x,pos.y,pos.h);
        telemetry.update();
    }

}