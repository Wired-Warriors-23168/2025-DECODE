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
//TODO ************* THIS IS BASED ON THE TELEOP FROM THE 6TH GRADE BOT, UPDATE IT FOR THE 23168 BOT!!!!

    // Declare OpMode members.
    private DcMotor intake;
    private Servo selector;
    private ColorSensor colorSensorA;
    private ColorSensor colorSensorB;

    /////////////////////////////////////////////////////////////////////////
    // Declare variables
    public double colorA;
    public double colorB;


    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "motor-intake");
        selector = hardwareMap.get(Servo.class, "servo-selector");
        colorSensorA = hardwareMap.get(ColorSensor.class, "sensor-color-A");
        colorSensorB = hardwareMap.get(ColorSensor.class, "sensor-color-B");

        // Establishing the direction and mode for the motors
        intake.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our methods while the OpMode is running
                manualIntakeSort();
                //telemetry.addData("ARTIFACT Color", ();
                telemetry.update();
            }
        }
    }


    /**
     * Manual control to run the intake and sort
     */
    private void manualIntakeSort() {
        // Manual control for the intake functions
        if (gamepad1.square) {
            runIntake();
            sortArtifact();
          }
    }


    /**
     * The runIntake function is designed to run the intake until an ARTIFACT is ready to sort.
     * When running this function, the intake will run until the distance sensors indicate there is an ARTIFACT in the intake, ready to sort
     */
    private void runIntake() {

    }

    /**
     * The sortArtifact function will utilize the selector to sort the green and purple ARTIFACTS into the correct launcher.
     * When running this function, the color sensors will determine the color of the ARTIFACT.  There are two sensors because the holes in the
     * ARTIFACT may cause the sensor to not read the color.  If one of the two sensors reads a color, the function should use that information to
     * move the selector to the correct side of the launcher.
     */
    private void sortArtifact() {

    }



}
