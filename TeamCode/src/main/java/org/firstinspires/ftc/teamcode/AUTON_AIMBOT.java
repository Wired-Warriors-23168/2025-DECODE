package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@Autonomous
public class AUTON_AIMBOT extends LinearOpMode {
//TODO ************* THIS IS BASED ON THE TELEOP FROM THE 6TH GRADE BOT, UPDATE IT FOR THE 23168 BOT!!!!

    // Declare OpMode members.
    private DcMotor flywheel;
    private Limelight3A limelight;

    private CRServo purpleServo;
    private CRServo greenServo;

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;


    private double shootposition = 10;

    private boolean seenobelisk = false;

    private int teamPipeline = 0; // blue auton
    private int patternID = 0;

    private long shootTimeMS = 1500;

    /////////////////////////////////////////////////////////////////////////
    // Declare variables
    public double txLimelight;
    public double tyLimelight;
    double tx = 0;
    double ty = 0;
    int tagID;

    @Override
    public void runOpMode() {
       // flywheel = hardwareMap.get(DcMotor.class, "motor-flywheel");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        greenServo = hardwareMap.get(CRServo.class, "greenServo");
        purpleServo = hardwareMap.get(CRServo.class, "purpleServo");
        // Establishing the direction and mode for the motors
      //  flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  flywheel.setDirection(DcMotor.Direction.REVERSE);
        purpleServo.setDirection(DcMotorSimple.Direction.FORWARD);
        greenServo.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left-front-drive");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "left-back-drive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "right-front-drive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "right-back-drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(11);

        limelight.start();

        limelight.pipelineSwitch(2);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                aimBot();
                telemetry.update();
            }
        }
    }



    private void aimBot() {
    //run if a button is held
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
        if (gamepad2.left_trigger > 0.5 && result.isValid()) {
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
        }

        if (gamepad1.dpad_right) {             // Forced shooting: Purple
            purpleServo.setPower(1);
            sleep(shootTimeMS);
            purpleServo.setPower(0);
        } else if (gamepad1.dpad_left) {      // Forced shooting: Green
            greenServo.setPower(1);
            sleep(shootTimeMS);
            greenServo.setPower(0);
        }

        telemetry.addData("Pattern ID", patternID);
        telemetry.addData("Seen obelisk", seenobelisk);
        telemetry.addData("Tag ID", tagID);
      //  telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
      //  telemetry.addData("Flywheel Power", flywheel.getPower());
        telemetry.addData("Target X", tx);
        telemetry.addData("Target Y", ty);
    }
    private void Pattern22() {
        purpleServo.setPower(1);
        sleep(shootTimeMS);
        purpleServo.setPower(0);
        greenServo.setPower(1);
        sleep(shootTimeMS);
        greenServo.setPower(0);
        purpleServo.setPower(1);
        sleep(shootTimeMS);
        purpleServo.setPower(0);
    }
    private void Pattern21() {
        greenServo.setPower(1);
        sleep(shootTimeMS);
        greenServo.setPower(0);
        purpleServo.setPower(1);
        sleep(2 * shootTimeMS);
        purpleServo.setPower(0);
    }
    private void Pattern23() {
        purpleServo.setPower(1);
        sleep(2 * shootTimeMS);
        purpleServo.setPower(0);
        greenServo.setPower(1);
        sleep(shootTimeMS);
        greenServo.setPower(0);
    }
    private void rotate() {
        // spin drive with p controller
        double wheelpower = (tx/Math.abs(tx)) * 0.5; // TODO make p controller
        leftFrontDrive.setPower(wheelpower);
    }

}
