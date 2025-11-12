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

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
@Autonomous(name="AUTO_BLUE_1_NEWTEST", group="AUTO", preselectTeleOp = "TELEOP_MAIN")
//@Autonomous(name="AUTO-BLUE-1", group="AUTO", preselectTeleOp = "TELEOP-BLUE (Blocks to Java)")
//@Disabled
public class AUTO_BLUE_1_NEWTEST extends LinearOpMode {

    // Declare OpMode members.
    private SparkFunOTOS otos;

    private DcMotorEx flywheel;
    private DcMotorSimple feeder;
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
    double flywheelPowerBank;
    double flywheelPowerMid;
    double flywheelPowerFar;
    double feederPower;
    int feederRotations;
    double feederLaunchTime;
    double launchWaitTime;
    private boolean seenobelisk = false;

    private int teamPipeline = 0; // blue auton
    private int patternID = 0;
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
    int tagID;
    double deltaTime;
    public static final String ALLIANCE_KEY = "Alliance";
    public String colorAlliance = "BLUE";


    private ElapsedTime runtime = new ElapsedTime();
    public double lastTime = 0.0;

    //TODO *********** Set the starting pose for the robot based on the alliance start position,
    // X and Y in INCHES from the center of the field, heading in RADIANS (or convert DEGREES to
    // RADIANS by multiplying the value in DEGREES by Math.PI/180
    Pose2d beginPose = new Pose2d(-62.5, -34.8, Math.toRadians(180));

    //Set AUTO waypoints
    Pose2d obeliskPose = new Pose2d(-30,-30,Math.toRadians(-135));  //pose to read the obelisk
    Pose2d shootPose = new Pose2d(-30,-30,Math.toRadians(-135));    //pose to shoot the pattern
    Pose2d intakePose1 = new Pose2d(-30,-30,Math.toRadians(-135));  //pose to intake artifacts from first row
    Pose2d intakePose2 = new Pose2d(-30,-30,Math.toRadians(-135));  //pose to intake artifacts from second row
    Pose2d endPose = new Pose2d(-30,-30,Math.toRadians(-135));      //pose at end of AUTO
    double firstArtifact = -36;     //Y-position of the first artifact in the row
    double secondArtifact = -41;    //Y-position of the second artifact in the row
    double thirdArtifact = -46;     //Y-position of the third artifact in the row



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
        greenServo.setDirection(Servo.Direction.FORWARD);
        purpleServo.setDirection(Servo.Direction.FORWARD);

        limelight.start();
        limelight.pipelineSwitch(2);

        //Instantiate the roadrunner Mecanum drive (via the OTOS localizer)
        //SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.localizer.setPose(beginPose);  //may have to do this for the new RR version per https://community.sparkfun.com/t/sparkfun-otos-with-ftc-inital-pose-always-0/67256

        telemetry.setMsTransmissionInterval(11);

        //Set the LED to green in INIT mode
        teamLED.setPosition(0.5);//green

        //set artifact holding positions
        flywheel.setVelocity(0);
        greenServo.setPosition(greenHoldPos);
        purpleServo.setPosition(purpleHoldPos);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ////////////////////////////////////////////////////////////////////////////////////
        // Wait for the game to start (driver presses START)
        ////////////////////////////////////////////////////////////////////////////////////
        waitForStart();

        if(isStopRequested()) return;

        if(colorAlliance=="BLUE"){
            teamLED.setPosition(0.600); //blue
        }
        else{
            teamLED.setPosition(0.283);//red
        }

        flywheel.setVelocity(closeVelocity);

        //Build the actions for our AUTO mode
        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        // Move to close firing position
                        .setReversed(false)
                        .setTangent(Math.toRadians(0))  //the heading the bot will take when leaving this position
                        .splineToLinearHeading(new Pose2d(-18,-48,Math.toRadians(90)),Math.toRadians(0))  //the target X,Y position, the target heading where the bot stops, and the heading the bot will approach that target heading from
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

        flywheel.setPower(0);

        SparkFunOTOS.Pose2D pos = otos.getPosition(); //Read OTOS Pose for telemetry

        //Store the alliance color to memory for use in TELEOP
        blackboard.put(ALLIANCE_KEY, colorAlliance);

        telemetry.addLine();

        // run until the end of the match (driver presses STOP)
    }

    ///////////////////////////////////////////////////
    //PUBLIC CLASSES FOR ROADRUNNER ACTION DEFINITIONS
    //////////////////////////////////////////////////

    public class patternLaunchAction implements Action {
        private Servo purpleServo;
        double purpleShootPos;
        double purpleDownPos;
        double servoLaunchTime;
        private Servo greenServo;
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
            this.greenServo = greenServo;
            this.greenShootPos = greenShootPos;
            this.greenDownPos = greenDownPos;
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












    //////////////////////////////////////////////////
    // PRIVATE VOIDS REFERENCED IN THE PUBLIC VOID
    //////////////////////////////////////////////////

    // Initializes all devices: sensors and actuators, to clean up the code
    private void initDevices(){
        //INIT LIMELIGHT


        // Initialize actuators

    }



}