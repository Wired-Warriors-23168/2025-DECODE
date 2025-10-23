package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@TeleOp
public class TELEOP_AIMBOT extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx flywheel;
    private Limelight3A limelight;

    private Servo purpleServo;
    private Servo greenServo;

    private boolean seenobelisk = false;

    private int teamPipeline = 0; // blue auton
    private int patternID = 0;

    private double farVelocity = 1360;
    private double closeVelocity = 1200;
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
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        greenServo.setDirection(Servo.Direction.FORWARD);
        purpleServo.setDirection(Servo.Direction.FORWARD);

        telemetry.setMsTransmissionInterval(11);

        limelight.start();

        limelight.pipelineSwitch(2);

        waitForStart();
        if (opModeIsActive()) {
            flywheel.setVelocity(farVelocity);
            greenServo.setPosition(0);
            purpleServo.setPosition(0);
            while (opModeIsActive()) {

                aimBot();
                telemetry.update();
            }
        }
    }



    private void aimBot() {

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
        if (gamepad2.left_trigger > 0.5) {  //&& result.isValid()+

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
            purpleServo.setPosition(0.2);
            sleep(250);
            purpleServo.setPosition(0);
        } else if (gamepad2.dpadLeftWasPressed()) {      // Forced shooting: Green
            greenServo.setPosition(0.2);
            sleep(250);
            greenServo.setPosition(0);
        }

        telemetry.addData("Pattern ID", patternID);
        telemetry.addData("Seen obelisk", seenobelisk);
        telemetry.addData("Tag ID", tagID);
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
        telemetry.addData("Flywheel Power", flywheel.getPower());
        telemetry.addData("Target X", tx);
        telemetry.addData("Target Y", ty);
    }
    private void Pattern22() {
        purpleServo.setPosition(0.2);
        sleep(250);
        purpleServo.setPosition(0);
        greenServo.setPosition(0.2);
        sleep(250);
        greenServo.setPosition(0);
        purpleServo.setPosition(0.2);
        sleep(250);
        purpleServo.setPosition(0);
    }
    private void Pattern21() {
        greenServo.setPosition(0.2);
        sleep(250);
        greenServo.setPosition(0);
        purpleServo.setPosition(0.2);
        sleep(250);
        purpleServo.setPosition(0);
        sleep(500);
        purpleServo.setPosition(0.2);
        sleep(250);
        purpleServo.setPosition(0);
    }
    private void Pattern23() {
        purpleServo.setPosition(0.2);
        sleep(250);
        purpleServo.setPosition(0);
        sleep(250);
        purpleServo.setPosition(0.2);
        sleep(250);
        purpleServo.setPosition(0);
        greenServo.setPosition(0.2);
        sleep(250);
        greenServo.setPosition(0);
    }
    private void rotate() {
        // spin drive with p controller
        double wheelpower = (tx/Math.abs(tx)) * 0.5; // TODO make p controller
        //leftFrontDrive.setPower(wheelpower);
    }

}
