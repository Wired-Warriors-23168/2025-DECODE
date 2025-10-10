package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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


    /////////////////////////////////////////////////////////////////////////
    // Declare variables
    public double txLimelight;
    public double tyLimelight;
    double tx = 0;
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

        limelight.pipelineSwitch(2 );

        limelight.start();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                aimBot();

            }
        }
    }




    private void aimBot() {
    //run if a button is held
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx();
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial != null) {
               tagID = fiducial.getFiducialId();
            }
        }

        if (tagID == 22) {
            purpleServo.setPower(1);
            sleep(500);
            purpleServo.setPower(0);
            greenServo.setPower(1);
            sleep(500);
            greenServo.setPower(0);
            purpleServo.setPower(1);
            sleep(500);
            purpleServo.setPower(0);

        } else if (tagID == 23) {
            purpleServo.setPower(1);
            sleep(1000);
            purpleServo.setPower(0);
            greenServo.setPower(1);
            sleep(500);
            greenServo.setPower(0);
        } else if (tagID == 21) {
            greenServo.setPower(1);
            sleep(500);
            greenServo.setPower(0);
            purpleServo.setPower(1);
            sleep(10000);
            purpleServo.setPower(0);
        }


        telemetry.addData("Tag ID", tagID);
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
        telemetry.addData("Flywheel Power", flywheel.getPower());
        telemetry.addData("Target X", tx);
        telemetry.update();
    }


}
