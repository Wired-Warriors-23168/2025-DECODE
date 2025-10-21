package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class TELEOP_INTAKE extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor intake;
    private Servo selector;
    private ColorSensor colorSensorA;
    private ColorSensor colorSensorB;
    private CRServo conveyorG;
    private CRServo conveyorP;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "motor-intake");
        selector = hardwareMap.get(Servo.class, "servo-selector");
        colorSensorA = hardwareMap.get(ColorSensor.class, "sensor-color-A");
        colorSensorB = hardwareMap.get(ColorSensor.class, "sensor-color-B");
        conveyorG = hardwareMap.get(CRServo.class, "green-conveyor");
        conveyorP = hardwareMap.get(CRServo.class, "purple-conveyor");


        // Establishing the direction and mode for the motors
        intake.setDirection(DcMotor.Direction.REVERSE);
        selector.setDirection(Servo.Direction.FORWARD);
        conveyorG.setDirection(CRServo.Direction.FORWARD);
        conveyorP.setDirection(CRServo.Direction.FORWARD);
        selector.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        if (opModeIsActive()) {
            conveyorG.setPower(1);
            conveyorP.setPower(1);
            while (opModeIsActive()) {

                intakeSort(false);
                telemetry.update();
            }
        }
    }

    public void intakeSort(boolean auto) {

        sortArtifact();

        if(gamepad2.right_bumper){
            intake.setPower(-1);
            selector.setPosition(0.75);
        }
        if(gamepad2.right_trigger > 0.2 || auto){
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }

    public void sortArtifact() {
        float sumGreenPurpleness = greenPurplenessA() + greenPurplenessB();
        telemetry.addData("sumGreenPurpleness",sumGreenPurpleness);
        if(sumGreenPurpleness > 120){
            selector.setPosition(1);
        }
        else if (sumGreenPurpleness < -70 ) {
            selector.setPosition(0.5);
        }
        else {
            selector.setPosition(0.75);
        }
        telemetry.addData("selector_pos",selector.getPosition());
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
