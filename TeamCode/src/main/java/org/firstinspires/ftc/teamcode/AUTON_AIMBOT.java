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


    private double shootposition = 10;

    private boolean seenobelisk = false;

    private int teamPipeline = 0; // blue auton
    private int patternID = 0;


    /////////////////////////////////////////////////////////////////////////
    // Declare variables
    public double txLimelight;
    public double tyLimelight;
    double tx = 0;
    double ty = 0;
    int tagID;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotor.class, "motor-flywheel");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        greenServo = hardwareMap.get(CRServo.class, "greenServo");
        purpleServo = hardwareMap.get(CRServo.class, "purpleServo");
        // Establishing the direction and mode for the motors
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        purpleServo.setDirection(DcMotorSimple.Direction.FORWARD);
        greenServo.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.setMsTransmissionInterval(11);

        limelight.start();

        limelight.pipelineSwitch(2);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                aimBot();

            }
        }
    }

    private void Pattern22() {
        purpleServo.setPower(1);
        sleep(1000);
        purpleServo.setPower(0);
        greenServo.setPower(1);
        sleep(1000);
        greenServo.setPower(0);
        purpleServo.setPower(1);
        sleep(1000);
        purpleServo.setPower(0);
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
            sleep(1000);
            tagID = 0;
            limelight.pipelineSwitch(teamPipeline);
        }

        if (patternID == 22 && tagID == 20) {
            if (tx < 5 && ty < 5) {
                Pattern22(); // Purple green purple
            }
        }

        telemetry.addData("Pattern ID", patternID);
        telemetry.addData("Seen obelisk", seenobelisk);
        telemetry.addData("Tag ID", tagID);
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
        telemetry.addData("Flywheel Power", flywheel.getPower());
        telemetry.addData("Target X", tx);
        telemetry.addData("Target Y", ty);
        telemetry.update();
    }


}
