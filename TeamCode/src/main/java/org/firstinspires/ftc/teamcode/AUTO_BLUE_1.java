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
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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
@Autonomous(name="AUTO_BLUE_1", group="AUTO", preselectTeleOp = "TELEOP_MAIN")
//@Autonomous(name="AUTO-BLUE-1", group="AUTO", preselectTeleOp = "TELEOP-BLUE (Blocks to Java)")
//@Disabled
public class AUTO_BLUE_1 extends LinearOpMode {

    // Declare OpMode members.
    private SparkFunOTOS otos;

    private DcMotorSimple flywheel;
    private DcMotorSimple feeder;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private Servo teamLED;

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
    ElapsedTime timer;
    ElapsedTime waitTimer;
    public static final String ALLIANCE_KEY = "Alliance";
    public String colorAlliance = "BLUE";

    //TODO *********** Set the starting pose for the robot based on the alliance start position,
    // X and Y in INCHES from the center of the field, heading in RADIANS (or convert DEGREES to
    // RADIANS by multiplying the value in DEGREES by Math.PI/180
    Pose2d beginPose = new Pose2d(-62.5, -35, Math.toRadians(-90));
//    Pose2d beginPose = new Pose2d(-62.125, -38.875, Math.toRadians(-90));       //Update to sit inside the goal triangle and touch the launch line

    @Override
    public void runOpMode() {

        //Set Hardware Map
        flywheel = hardwareMap.get(DcMotorSimple.class, "motor-flywheel");
        feeder = hardwareMap.get(DcMotorSimple.class, "motor-feeder");
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor-otos");
        teamLED = hardwareMap.get(Servo.class, "led-light");

        //initDevices(); // Initialize all motors, servos, sensors

        // Establishing the direction and mode for the motors
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
//        ((DcMotorEx) flywheel).setMotorEnable();
//        ((DcMotorEx) feeder).setMotorEnable();

        //Instantiate the roadrunner Mecanum drive (via the OTOS localizer)
        //SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.localizer.setPose(beginPose);  //may have to do this for the new RR version per https://community.sparkfun.com/t/sparkfun-otos-with-ftc-inital-pose-always-0/67256

        //Set the LED to green in INIT mode
        teamLED.setPosition(0.5);//green

        //Set all actuator target positions
        flywheelPowerBank = 0.57;    //the bankshot shooting power
        flywheelPowerMid = 0.9;     //the middle shooting power
        flywheelPowerFar = 1.0;     //the far shooting power
        feederPower = 0.6;          //the feeder power when activating
        feederLaunchTime = 8.0;     //the amount of time to rotate the feeder to launch an artifact (when not using RUN_TO_POSITION)
        feederRotations = 3;        //FUTURE USE the number of feeder rotations to launch an artifact (when using RUN_TO_POSITION)
        launchWaitTime = 2.5;       //the wait time between launches so the flywheel can spin up

        //Set all field positions
        //Pose2d waypointBank = new Pose2d(-30,-30,Math.toRadians(-135));  //Waypoint for spline, bankshot launch position
        //Pose2d waypointMid = new Pose2d(-30,-30,Math.toRadians(-135));    //Waypoint for spline, middle launch position
        //Pose2d waypointFar = new Pose2d(-30,-30,Math.toRadians(-135));    //Waypoint for spline, far launch position
        //Pose2d waypointPushStart = new Pose2d(-30,-30,Math.toRadians(-135));   //Waypoint to start pushing artifacts
        //Pose2d waypointPushEndSpline = new Pose2d(-30,-30,Math.toRadians(-135));       //Waypoint to end pushing artifacts with a spline
        //Vector2d waypointPushEndLine = new Vector2d(-51,-39);                 //Waypoint to end pushing artifacts with a LineTo



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

        flywheel.setPower(flywheelPowerBank);       //Set the flywheel to max power to start it up

        //Build the actions for our AUTO mode
        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        // Move to bankshot firing position
                        .setReversed(false)
                        .setTangent(Math.toRadians(45))
                        .splineToLinearHeading(new Pose2d(-27,-27,Math.toRadians(-135)),Math.toRadians(45))

//                        //launch an artifact with the feeder
                        .stopAndAdd(new SequentialAction(
                                new launchArtifactAction(feeder, feederLaunchTime, feederPower)    //rotate the feeder for time feederLaunchTime
                                //new setFeederPowerOffAction(feeder),                                //turn off the feeder
                                //new SleepAction(launchWaitTime),
                                //new launchWait(launchWaitTime, feeder),                                     //wait for launchWaitTime seconds
                               // new launchArtifactAction(feeder, feederLaunchTime, feederPower),    //rotate the feeder for time feederLaunchTime
                                //new setFeederPowerOffAction(feeder),                                //turn off the feeder
                                //new launchWait(launchWaitTime, feeder),
                                //new SleepAction(launchWaitTime),
                                //new launchArtifactAction(feeder, feederLaunchTime, feederPower)    //rotate the feeder for time feederLaunchTime
                                //new setFeederPowerOffAction(feeder)                                //turn off the feeder
                                //new launchWait(launchWaitTime,feeder)                                      //wait for launchWaitTime seconds

                        ))
                        .setReversed(false)
                        .setTangent(Math.toRadians(45))
                        .splineToLinearHeading(new Pose2d(10,-14,Math.toRadians(-90)),Math.toRadians(0))

                        .build());

        flywheel.setPower(0);       //Set the flywheel to max power to start it up

        SparkFunOTOS.Pose2D pos = otos.getPosition(); //Read OTOS Pose for telemetry

        //Store the alliance color to memory for use in TELEOP
        blackboard.put(ALLIANCE_KEY, colorAlliance);

        telemetry.addLine();
        telemetry.addData("OTOS Data", "X: (%.1f), Y: (%.1f), H: (%.2f)", pos.x,pos.y,pos.h);
        telemetry.update();

        // run until the end of the match (driver presses STOP)
    }

    ///////////////////////////////////////////////////
    //PUBLIC CLASSES FOR ROADRUNNER ACTION DEFINITIONS
    //////////////////////////////////////////////////

    // Set the feeder power to zero
    public class setFeederPowerOffAction implements Action {
        DcMotorSimple feeder;

        public setFeederPowerOffAction(DcMotorSimple feeder) {

            this.feeder = feeder;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            feeder.setPower(0);
            return false;
        }
    }

    // Set the feeder power to one
    public class launchArtifactAction implements Action {
        DcMotorSimple feeder;
        double launchTime;
        double feederPower;
        ElapsedTime timer;
//TODO
        public launchArtifactAction(DcMotorSimple feeder, double launchTime, double feederPower) {
            this.feeder = feeder;
            this.launchTime = launchTime;
            this.feederPower = feederPower;
            timer = new ElapsedTime();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            feeder.setPower(feederPower);
//            telemetry.addData("timer", "t: (%.1f)", timer);
//            telemetry.update();

            return timer.seconds() < launchTime;
        }
    }
//TODO
    // Wait a set time after a launch without using wait() or sleep()
    public class launchWait implements Action {
        DcMotorSimple feeder;
        double waitTime;
        ElapsedTime waitTimer;

        public launchWait(double waitTime,DcMotorSimple feeder) {
            this.feeder = feeder;
            this.waitTime = waitTime;
            waitTimer = new ElapsedTime();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (waitTimer == null) {
                waitTimer = new ElapsedTime();
            }
            feeder.setPower(0.0);
            return waitTimer.seconds() < waitTime;
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
