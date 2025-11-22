package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TELEOP_INTAKE extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor intake;
    private Servo selector;
    private ColorSensor colorSensorA;
    private ColorSensor colorSensorB;
    private double startPos;
    private double sortOffset = 55.0/300.0;
    private double neutralPos = 140.0/300.0;
    private boolean onewaysort = false;
    private int sortToggle = 0;
    private double revolutions;
    private double intakeRevs = 2.5;
    private double sortTime = 0;
    public ElapsedTime deltaTimer = new ElapsedTime();


    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "motor-intake");
        selector = hardwareMap.get(Servo.class, "servo-selector");
        colorSensorA = hardwareMap.get(ColorSensor.class, "sensor-color-a");
        colorSensorB = hardwareMap.get(ColorSensor.class, "sensor-color-b");


        // Establishing the direction and mode for the motors
        intake.setDirection(DcMotor.Direction.REVERSE);
        selector.setDirection(Servo.Direction.FORWARD);
        selector.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        if (opModeIsActive()) {

            deltaTimer.reset();

            while (opModeIsActive()) {

                intakeSort(false);
                telemetry.update();
            }
        }
    }
    public void intakeSort(boolean auto) {
        sortArtifact();

        revolutions = intake.getCurrentPosition()/288.0;
        intake.getCurrentPosition();

        if(gamepad2.right_trigger > 0.2){
            intake.setPower(-1);
            selector.setPosition(neutralPos);
        } else if (intake.getPower() <= -1 ) {
            intake.setPower(0);
        }
        if (auto) {
            intake.setPower(1);
        }
        else if(gamepad2.rightBumperWasPressed()){
            startPos = intake.getCurrentPosition();
            intake.setPower(1);
        } else if (intake.getCurrentPosition() > startPos + (intakeRevs * 288.0)) {
            intake.setPower(0);
        }
        telemetry.addData("sort time", sortTime);
        telemetry.addData("delta timer", deltaTimer.milliseconds());
        telemetry.addData("Start Pos", startPos);
    }

    public void sortArtifact() {
        float sumGreenPurpleness = greenPurplenessA() + greenPurplenessB();
        telemetry.addData("sumGreenPurpleness",sumGreenPurpleness);
        if (gamepad2.backWasPressed()) {
            onewaysort = !onewaysort; // flip
        }
        if (gamepad2.left_bumper) {
            selector.setPosition(neutralPos - sortOffset);
            sortToggle = -1;
        }
        else if(gamepad2.left_trigger > 0.1){
            selector.setPosition(neutralPos+sortOffset);
            sortToggle = 1;
        }
        else if(Math.abs(sumGreenPurpleness)>50 && onewaysort){
            selector.setPosition((sortToggle*sortOffset)+neutralPos);
        }
        else if(sumGreenPurpleness > 50){
            selector.setPosition(neutralPos - sortOffset);
            sortTime = 1000 + deltaTimer.milliseconds();
        }
        else if (sumGreenPurpleness < -50 ) {
            selector.setPosition(neutralPos + sortOffset);
            sortTime = 1000 + deltaTimer.milliseconds();
        }
        else if (sortTime <= deltaTimer.milliseconds()) {
            selector.setPosition(neutralPos);
        }
        telemetry.addData("selector pos",selector.getPosition());
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
