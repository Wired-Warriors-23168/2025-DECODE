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
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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
//TODO ********** Make sure the displayed names match the java class name and correct TELEOP is specified!  **************
@Autonomous(name="AUTO_2024_EXAMPLE", group="AUTO", preselectTeleOp = "TELEOP-BLUE")
//@Autonomous(name="AUTO-BLUE-1", group="AUTO", preselectTeleOp = "TELEOP-BLUE (Blocks to Java)")
//@Disabled
public class AUTO_2024_EXAMPLE extends LinearOpMode {

    // Declare OpMode members.
    private Limelight3A limelight;
    private SparkFunOTOS otos;
    private DcMotor ArmLift;
    private DcMotor ArmExtender;
    private DcMotor ArmHangerLeft;
    private DcMotor ArmHangerRight;
    private CRServo Intake;
    private Servo Wrist;
    private DistanceSensor ColorSensor_DistanceSensor;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    /////////////////////////////////////////////////////////////////////////
    // Declare variables
    //TODO ********** Set the Alliance Color  **************
    public String colorAlliance = "BLUE"; //Enter either BLUE or RED alliance, this will chance all further color-related settings
    public double txLimelight;
    public double tyLimelight;
    int currentPos;
    int errorPos;
    double motorPower;
    double errorRateMAX;
    double gainP;
    double motorPowerMAX;
    public int targetPos;
    double COUNTS_PER_DEGREE;
    int COUNTS_PER_MOTOR_REV;
    int GEAR_REDUCTION;
    int COUNTS_PER_GEAR_REV;
    int currentPos_Extender;
    ElapsedTime intakeTimer;
    int targetPos_Extender;
    int targetPos_Wrist;
    int targetPos_Hanger;
    int HangerUp;
    int HangerZero;
    double WristUp;
    double WristIntake;
    double WristEject;
    double WristStore;
    int ExtenderFull;
    int ExtenderIntake;
    int ExtenderRetract;
    int ArmFull;
    int ArmStore;
    //TODO *********** Set the starting pose for the robot based on the alliance start position,
    // X and Y in INCHES from the center of the field, heading in RADIANS (or convert DEGREES to
    // RADIANS by multiplying the value in DEGREES by Math.PI/180
    Pose2d beginPose = new Pose2d(-32.25, -63.44, 180*Math.PI/180);
    @Override
    public void runOpMode() {

        //Set Hardware Map
        ArmLift = hardwareMap.get(DcMotor.class, "Arm Lift");
        ArmExtender = hardwareMap.get(DcMotor.class, "Arm Extender");
        ArmHangerLeft = hardwareMap.get(DcMotor.class, "Arm Hanger Left");
        ArmHangerRight = hardwareMap.get(DcMotor.class, "Arm Hanger Right");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        ColorSensor_DistanceSensor = hardwareMap.get(DistanceSensor.class, "Color Sensor");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        initDevices(); // Initialize all motors, servos, sensors

        //Settings for the lift arm
        COUNTS_PER_MOTOR_REV = 28;
        GEAR_REDUCTION = 144;
        COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
        COUNTS_PER_DEGREE = (double) COUNTS_PER_GEAR_REV / 360;
        currentPos_Extender = ArmExtender.getCurrentPosition();
        intakeTimer = new ElapsedTime();

        targetPos = 0;           // Target arm position, in absolute position - encoder ticks
        targetPos_Extender = 0;  // Target arm position, in absolute position - encoder ticks
        gainP = 0.0025;           // Position controller proportional gain
        errorRateMAX = 0.4;      // Maximum arm movement speed
        currentPos = ArmLift.getCurrentPosition();         // Calculate absolute position considering initial position as zero
        targetPos = currentPos;
        currentPos_Extender = ArmExtender.getCurrentPosition();
        targetPos_Extender = currentPos_Extender;
        targetPos_Hanger = 0;
        targetPos_Wrist = 0;
        ((DcMotorEx) ArmLift).setMotorDisable();
        ((DcMotorEx) ArmExtender).setMotorEnable();
        ((DcMotorEx) ArmHangerLeft).setMotorEnable();
        ((DcMotorEx) ArmHangerRight).setMotorEnable();

        // TODO: Set initial limelight pipeline for alliance color: 0=red, 1=blue, 2=yellow
        limelight.pipelineSwitch(2);

        //Instantiate the roadrunner Mecanum drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        //Set all actuator target positions
        HangerUp = 350;
        HangerZero = 0;
        WristUp = 0.275;
        WristIntake = 0.46;
        WristEject = 0.37; //0.38
        WristStore = 0.6;
        ExtenderFull = 1510; //1510
        ExtenderIntake = 360;
        ExtenderRetract = 10;
        ArmFull = 100;
        ArmStore = 0;

        //Set all field positions
        Vector2d waypointBasketInit = new Vector2d(-49.125,-63.94);
        Pose2d waypointBasket = new Pose2d(-53.0,-53.0,Math.toRadians(-135));
        Pose2d waypointSample1 = new Pose2d(-49,-36,Math.toRadians(90));
        Pose2d waypointSample2 = new Pose2d(-59.25, -36,Math.toRadians(90));
        Pose2d waypointSample3 = new Pose2d(-62.5, -15.5,Math.toRadians(-90));


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ////////////////////////////////////////////////////////////////////////////////////
        // Wait for the game to start (driver presses START)
        ////////////////////////////////////////////////////////////////////////////////////
        waitForStart();

        if(isStopRequested()) return;
        ((DcMotorEx) ArmLift).setMotorEnable();
        //targetPos = (int) (35*COUNTS_PER_DEGREE);

        //Build the actions for our AUTO mode
        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        // Raise arm and extend simultaneously to top basket
                        // Raising and extending at the same time is risky, but fast if there is space
                        .stopAndAdd(new SequentialAction(
                                new ParallelAction(
                                        new proportionalController(ArmLift, COUNTS_PER_DEGREE * ArmFull,1.5, gainP,errorRateMAX),
                                        new setArmExtensionAction(ArmExtender,0.75,ExtenderFull)
                                        //new setWristPositionAction(Wrist, WristEject)
                                ),
                                new setWristPositionAction(Wrist, WristEject)
                        ))
                        // Drive forward to the basket and eject sample
                        .splineTo(waypointBasketInit, Math.toRadians(-180))
                        .stopAndAdd(new SequentialAction(
                                new ejectSampleAction(Intake,0.5),
                                new setIntakePowerAction(Intake, 0)
                        ))
                        //Retract to rest position and hold
                        .stopAndAdd(new SequentialAction(
                                new ParallelAction(
                                        new proportionalController(ArmLift, COUNTS_PER_DEGREE * ArmFull,0.75, gainP,errorRateMAX),
                                        new setArmExtensionAction(ArmExtender,0.75,ExtenderRetract)
                                ),
                                new proportionalController(ArmLift, COUNTS_PER_DEGREE * ArmStore,1.5, gainP,errorRateMAX),
                                new setArmPowerOffAction(ArmLift),
                                new setIntakePowerAction(Intake, 1),
                                new setWristPositionAction(Wrist, WristIntake)
                        ))

                        // Drive to the first sample intake position (the sample closest to the submersible)
                        .setTangent(0)
                        .splineToLinearHeading(waypointSample1,90*Math.PI/180)

                        //Extend arm to pick up sample from floor
                        .stopAndAdd(new SequentialAction(
                                new ParallelAction(
                                        new setArmExtensionAction(ArmExtender,0.5,ExtenderIntake),
                                        new intakeSampleAction(Intake,2)
                                ),
                                //Retract to rest position and hold
                                new ParallelAction(
                                        new setIntakePowerAction(Intake,0),
                                        new setArmExtensionAction(ArmExtender,0.5,ExtenderRetract),
                                        new setWristPositionAction(Wrist,WristEject)
                                )
                        ))

                        // Drive to the basket
                        .setTangent(0)
                        .splineToLinearHeading(waypointBasket, Math.toRadians(-135))

                        // Raise to top basket and eject sample
                        .stopAndAdd(new SequentialAction(
                                new ParallelAction(
                                        new proportionalController(ArmLift, COUNTS_PER_DEGREE * ArmFull,1.65, gainP,errorRateMAX)
                                ),
                                new ParallelAction(
                                        new proportionalController(ArmLift, COUNTS_PER_DEGREE * ArmFull,1.1, gainP,errorRateMAX),
                                        new setArmExtensionAction(ArmExtender,0.75,ExtenderFull)
                                ),
                                new ejectSampleAction(Intake,1),
                                new setIntakePowerAction(Intake, 0)
                        ))

                        //Retract to rest position and hold
                        .stopAndAdd(new SequentialAction(
                                new ParallelAction(
                                        new proportionalController(ArmLift, COUNTS_PER_DEGREE * ArmFull,0.75, gainP,errorRateMAX),
                                        new setArmExtensionAction(ArmExtender,0.75,ExtenderRetract)
                                ),
                                new proportionalController(ArmLift, COUNTS_PER_DEGREE * ArmStore,1.5, gainP,errorRateMAX),
                                new setArmPowerOffAction(ArmLift),
                                new setIntakePowerAction(Intake, 1),
                                new setWristPositionAction(Wrist, WristIntake)
                        ))

                        // Drive to the second sample intake position (the middle of the three yellow samples)
                        .setTangent(0)
                        .splineToLinearHeading(waypointSample2, 90*Math.PI/180)

                        //Extend arm to pick up sample from floor
                        .stopAndAdd(new SequentialAction(
                                new ParallelAction(
                                        new setArmExtensionAction(ArmExtender,0.5,ExtenderIntake),
                                        new intakeSampleAction(Intake,2)
                                ),
                                //Retract to rest position and hold
                                new ParallelAction(
                                        new setIntakePowerAction(Intake,0),
                                        new setArmExtensionAction(ArmExtender,0.5,ExtenderRetract),
                                        new setWristPositionAction(Wrist,WristEject)
                                )

                        ))
                        // Drive to the basket
                        .setTangent(0)
                        .splineToLinearHeading(waypointBasket, Math.toRadians(-135))

                        // Raise to top basket and eject sample
                        .stopAndAdd(new SequentialAction(
                                new proportionalController(ArmLift, COUNTS_PER_DEGREE * ArmFull,1.75, gainP,errorRateMAX),
                                new ParallelAction(
                                        new proportionalController(ArmLift, COUNTS_PER_DEGREE * ArmFull,1.0, gainP,errorRateMAX),
                                        new setArmExtensionAction(ArmExtender,0.75,ExtenderFull)
                                ),
                                new ejectSampleAction(Intake,1),
                                new setIntakePowerAction(Intake, 0)
                        ))

                        //Retract to storage position
                        .stopAndAdd(new SequentialAction(
                                new ParallelAction(
                                        new proportionalController(ArmLift, COUNTS_PER_DEGREE * ArmFull,0.75, gainP,errorRateMAX),
                                        new setArmExtensionAction(ArmExtender,0.75,ExtenderRetract)
                                ),
                                new proportionalController(ArmLift, COUNTS_PER_DEGREE * ArmStore,1.5, gainP,errorRateMAX),
                                new setArmPowerOffAction(ArmLift),
                                new setWristPositionAction(Wrist, WristStore)
                        ))

                        // Drive in reverse around the last sample (closest to the wall) and position to push it into the net zone
                        .setReversed(true)
                        .splineToLinearHeading(waypointSample3,Math.toRadians(-90))
                        .setReversed(false)

                        // Push the sample into the net zone
                        .lineToY(-44)

                        .build());



        SparkFunOTOS.Pose2D pos = otos.getPosition(); //Read OTOS Pose for telemetry

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.addLine();
        telemetry.addData("OTOS Data", "X: (%.1f), Y: (%.1f), H: (%.2f)", pos.x,pos.y,pos.h);
        telemetry.update();

        // run until the end of the match (driver presses STOP)
    }

    ///////////////////////////////////////////////////
    //PUBLIC CLASSES FOR ROADRUNNER ACTION DEFINITIONS
    //////////////////////////////////////////////////

    // Extend the arm to a given position
    public class setArmExtensionAction implements Action {
        DcMotor ArmExtender;
        double targetPos_Extender;
        double maxTime;
        ElapsedTime timer;

        public setArmExtensionAction(DcMotor ArmExtender, double maxTime, double targetPos_Extender) {
            this.ArmExtender = ArmExtender;
            this.maxTime = maxTime;
            this.targetPos_Extender = targetPos_Extender;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            ArmExtender.setTargetPosition((int) targetPos_Extender);
            ArmExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmExtender.setPower(1);

            if(timer.seconds() >= maxTime){
                return false;
            } else if (ArmExtender.isBusy()){
                return true;
            } else {
                return false;
            }
        }
    }

    // Set the hangers to a given position
    public class setHangerPositionAction implements Action {
        DcMotor ArmHangerLeft;
        DcMotor ArmHangerRight;
        double targetPos_Hanger;
        double hangTime;
        ElapsedTime timer;

        public setHangerPositionAction(DcMotor ArmHangerLeft, DcMotor ArmHangerRight, double targetPos_Hanger, double hangTime) {
            this.ArmHangerLeft = ArmHangerLeft;
            this.ArmHangerRight = ArmHangerRight;
            this.targetPos_Hanger = targetPos_Hanger;
            this.hangTime = hangTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            ArmHangerLeft.setTargetPosition((int) targetPos_Hanger);
            ArmHangerRight.setTargetPosition((int) targetPos_Hanger);
            ArmHangerLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmHangerRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmHangerLeft.setPower(1);
            ArmHangerRight.setPower(1);

            if (timer.seconds() < hangTime) {
                return true;
            } else {
                return false;
            }
        }
    }


    // Set the Arm power to zero
    public class setArmPowerOffAction implements Action {
        DcMotor ArmLift;

        public setArmPowerOffAction(DcMotor ArmLift) {
            this.ArmLift=ArmLift;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ArmLift.setPower(0);
            return false;
        }
    }

    // Set the wrist to a given position
    public class setWristPositionAction implements Action {
        Servo Wrist;
        double targetPos_Wrist;

        public setWristPositionAction(Servo Wrist, double targetPos_Wrist) {
            this.Wrist = Wrist;
            this.targetPos_Wrist = targetPos_Wrist;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Wrist.setPosition(targetPos_Wrist);

            //return Wrist.getPosition() != targetPos_Wrist;
            return false;
        }
    }

    // Set the intake power
    public class setIntakePowerAction implements Action {
        CRServo Intake;
        double powerIntake;

        public setIntakePowerAction(CRServo Intake, double powerIntake) {
            this.Intake = Intake;
            this.powerIntake = powerIntake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Intake.setPower(powerIntake);
            return false;
        }
    }

    // Intake a sample into the intake
    public class intakeSampleAction implements Action {
        CRServo Intake;
        double maxTime;
        ElapsedTime timer;

        public intakeSampleAction(CRServo Intake, double maxTime) {
            this.Intake = Intake;
            this.maxTime = maxTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            Intake.setPower(1);
            telemetry.addData("SampleDistance: ", ColorSensor_DistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();

            if (timer.seconds() >= 1) {
                return false;
            } else if (ColorSensor_DistanceSensor.getDistance(DistanceUnit.INCH) > 1.75) {
                return true;
            } else
                return false;
        }
    }

    // Eject a sample from the intake
    public class ejectSampleAction implements Action {
        CRServo Intake;
        double maxTime;
        ElapsedTime timer;

        public ejectSampleAction(CRServo Intake, double maxTime) {
            this.Intake = Intake;
            this.maxTime = maxTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            Intake.setPower(-1);
            telemetry.addData("SampleDistance: ", ColorSensor_DistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();

            if (ColorSensor_DistanceSensor.getDistance(DistanceUnit.INCH) < 2.00 ) {
                return true;
            } else {
                return false;
            }
            //return false;
        }
    }

    //Proportional Controller implemented as an action for parallel actions
    public class proportionalController implements Action {
        DcMotor ArmLift;
        double target;
        double P;
        double RateMax;
        double holdTime;
        ElapsedTime timer;

        public proportionalController(DcMotor ArmLift,double target,double holdTime, double P,double RateMax){
            this.ArmLift = ArmLift;
            this.target = target;
            this.P = P;
            this.RateMax = RateMax;
            this.holdTime   = holdTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            // Proportional Arm Position Controller
            // Update current position
            currentPos = ArmLift.getCurrentPosition();
            // Calculate the error that the controller will work to minimize.  Means that it will try to make currentPos equal to targetPos.
            errorPos = (int) (target - currentPos);
            // This is the actual proportional control, with the rate of change limited to errorRateMAX.
            // Note that it can be negative or positive depending on which direction the motor needs to turn (if the current position is larger than target or less than target).
            motorPower = Math.min(Math.max(errorPos * gainP, -errorRateMAX), errorRateMAX);
            if (Math.abs(motorPower) > 1) {
                // Normalize motor power to be less than 1.0
                motorPowerMAX = Math.abs(motorPower);
                motorPower = motorPower / motorPowerMAX;
            }
            // Move the motor / arm in the proper direction, then do it all again in the loop.
            ArmLift.setPower(motorPower);


            if (timer.seconds() < holdTime) {
                return true;
            } else {
                return false;
            }
            //return true;
        }
    }




    //////////////////////////////////////////////////
    // PRIVATE VOIDS REFERENCED IN THE PUBLIC VOID
    //////////////////////////////////////////////////

    // Initializes all devices: sensors and actuators, to clean up the code
    private void initDevices(){
        //INIT LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0); // Set a default defined pipeline
        limelight.start(); // This tells Limelight to start looking!

        // Initialize actuators
        ArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmLift.setDirection(DcMotor.Direction.REVERSE);
        ((DcMotorEx) ArmLift).setVelocity(10);
        ArmExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmExtender.setDirection(DcMotor.Direction.REVERSE);
        ArmHangerLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmHangerLeft.setDirection(DcMotor.Direction.REVERSE);
        ArmHangerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmHangerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmHangerRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmHangerRight.setDirection(DcMotor.Direction.REVERSE);
        ArmHangerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmHangerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setDirection(CRServo.Direction.REVERSE);
        Wrist.setDirection(Servo.Direction.FORWARD);
    }


    private void ProportionalController(double targetPos, double P, double RateMAX) {
        // Proportional Arm Position Controller
        // Update current position
        currentPos = ArmLift.getCurrentPosition();
        // Calculate the error that the controller will work to minimize.  Means that it will try to make currentPos equal to targetPos.
        errorPos = (int) (targetPos - currentPos);
        // This is the actual proportional control, with the rate of change limited to errorRateMAX.
        // Note that it can be negative or positive depending on which direction the motor needs to turn (if the current position is larger than target or less than target).
        motorPower = Math.min(Math.max(errorPos * gainP, -errorRateMAX), errorRateMAX);
        if (Math.abs(motorPower) > 1) {
            // Normalize motor power to be less than 1.0
            motorPowerMAX = Math.abs(motorPower);
            motorPower = motorPower / motorPowerMAX;
        }
        // Move the motor / arm in the proper direction, then do it all again in the loop.
        ArmLift.setPower(motorPower);
    }
}
