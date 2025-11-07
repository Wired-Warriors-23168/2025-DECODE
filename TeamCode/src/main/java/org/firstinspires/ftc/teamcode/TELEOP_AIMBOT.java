package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
enum ShooterState {
    WAITING_FOR_FLYWHEEL,
    WAITING_FOR_SERVO,
    IDLE,
    IDLE_WITH_FLYWHEEL
};
@Disabled
@TeleOp
public class TELEOP_AIMBOT extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx flywheel;
    private ElapsedTime servoTime = new ElapsedTime();

    private Limelight3A limelight;

    private Servo purpleServo;
    private Servo greenServo;

    private ShooterState shooterState = ShooterState.IDLE;
    private PIDFCoefficients shooterpid = new PIDFCoefficients(5, 0, 0, 0);
    private double startServoTime = 0;
    private boolean seenobelisk = false;

    private int teamPipeline = 0; // blue auton
    private int patternID = 0;

    private int ballnumber = 0;
    private double servoShootPosGreen = 0.2467;
    private double servoShootPosPurple = 0.0933;
    private double servoShootPosGreenDown = 0.32;
    private double servoShootPosPurpleDown = 0.02;
    private double farVelocity = 1360;
    private double closeVelocity = 1200;
    private double idleVelocity = 600;
    private double targetVelocity;
    public double txLimelight;
    public double tyLimelight;
    double tx = 0;
    double ty = 0;
    int tagID;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "motor-flywheel");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        greenServo = hardwareMap.get(Servo.class, "greenServo");
        purpleServo = hardwareMap.get(Servo.class, "purpleServo");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterpid);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        greenServo.setDirection(Servo.Direction.FORWARD);
        purpleServo.setDirection(Servo.Direction.FORWARD);

        telemetry.setMsTransmissionInterval(11);

        limelight.start();

        limelight.pipelineSwitch(2);


        waitForStart();
        if (opModeIsActive()) {
            targetVelocity = 2500;
            greenServo.setPosition(servoShootPosGreenDown);  // 0.3 down 0.2267 up
            purpleServo.setPosition(servoShootPosPurpleDown);      //0.02 down 0.0933 up
            while (opModeIsActive()) {

                aimBot();
                telemetry.update();
            }
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
                    shooterState = shooterState.IDLE_WITH_FLYWHEEL;
                }
            case IDLE_WITH_FLYWHEEL:
                break;
        }
            if (gamepad2.aWasPressed()) {
                if (flywheel.getVelocity() < 2000) {
                    flywheel.setVelocity(targetVelocity);
                    shooterState = shooterState.IDLE_WITH_FLYWHEEL;
                } else {
                    shooterState = shooterState.IDLE;
                    flywheel.setVelocity(0);
                }
            }

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

            if (tagID != 0 && !seenobelisk) {
                seenobelisk = true;
                patternID = tagID; // save pattern
                sleep(50);
                tagID = 0;
                limelight.pipelineSwitch(teamPipeline);
            }

            if (gamepad2.right_bumper) {  //&& result.isValid()+

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
                // flywheel.setVelocity(2500);
            }


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
            if (ballnumber == 1 && flywheel.getVelocity() > targetVelocity - 20) {
                if(!(startServoTime > 1)) {
                    startServoTime = servoTime.milliseconds();
                    shooterState = shooterState.WAITING_FOR_SERVO;
                    purpleServo.setPosition(servoShootPosPurple);
                }
                if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                    purpleServo.setPosition(servoShootPosPurpleDown);
                    ++ballnumber;
                }
            } else if (ballnumber == 2 && flywheel.getVelocity() > targetVelocity - 20) {
                if(!(startServoTime > 1)) {
                    startServoTime = servoTime.milliseconds();
                    shooterState = shooterState.WAITING_FOR_SERVO;
                    greenServo.setPosition(servoShootPosGreen);
                }
                if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                greenServo.setPosition(servoShootPosGreenDown);
                ++ballnumber;
                }
            } else if (ballnumber == 3 && flywheel.getVelocity() > targetVelocity - 20) {
                if(!(startServoTime > 1)) {
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
        if (ballnumber == 1 && flywheel.getVelocity() > targetVelocity - 20) {
            if(!(startServoTime > 1)) {
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
                greenServo.setPosition(servoShootPosGreen);
            }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                greenServo.setPosition(servoShootPosGreenDown);
                ++ballnumber;
            }
        } else if (ballnumber == 2 && flywheel.getVelocity() > targetVelocity - 20) {
            if(!(startServoTime > 1)) {
                purpleServo.setPosition(servoShootPosPurple);
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
            }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(servoShootPosPurpleDown);
                ++ballnumber;
            }
        } else if (ballnumber == 3 && flywheel.getVelocity() > targetVelocity - 20) {
            if(!(startServoTime > 1)) {
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
            }
            if(shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(servoShootPosPurple);
                ++ballnumber;
            }
        }
        else if (ballnumber == 4 && flywheel.getVelocity() > targetVelocity - 20) {
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
        if (ballnumber == 1 && flywheel.getVelocity() > targetVelocity - 20) {
            if(!(startServoTime > 1)) {
                purpleServo.setPosition(servoShootPosPurple);
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
            }
            if (shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(servoShootPosPurpleDown);
                ++ballnumber;
            }
        } else if (ballnumber == 2 && flywheel.getVelocity() > targetVelocity - 20) {
            if(!(startServoTime > 1)) {
                startServoTime = servoTime.milliseconds();
                shooterState = shooterState.WAITING_FOR_SERVO;
            }
            if (shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                purpleServo.setPosition(servoShootPosPurple);
                ++ballnumber;
            }
            else if (ballnumber == 3 && flywheel.getVelocity() > targetVelocity - 20) {
                if(!(startServoTime > 1)) {
                    startServoTime = servoTime.milliseconds();
                    shooterState = shooterState.WAITING_FOR_SERVO;
                }
                if (shooterState == shooterState.IDLE_WITH_FLYWHEEL) {
                    purpleServo.setPosition(servoShootPosPurpleDown);
                    ++ballnumber;
                }
            }
        } else if (ballnumber == 4 && flywheel.getVelocity() > targetVelocity - 20) {
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
        // spin drive with p controller
        double wheelpower = (tx/Math.abs(tx)) * 0.5; // TODO make p controller
        //leftFrontDrive.setPower(wheelpower);
    }

}
