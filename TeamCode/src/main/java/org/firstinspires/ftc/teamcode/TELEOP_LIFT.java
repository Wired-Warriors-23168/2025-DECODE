package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TELEOP_LIFT extends LinearOpMode {
//TODO ************* THIS IS BASED ON THE TELEOP FROM THE 6TH GRADE BOT, UPDATE IT FOR THE 23168 BOT!!!!

    // Declare OpMode members.
    private DcMotor lift;

    /////////////////////////////////////////////////////////////////////////
    // Declare variables



    @Override
    public void runOpMode() {
        lift = hardwareMap.get(DcMotor.class, "motor-lift");

        // Establishing the direction and mode for the motors
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our methods while the OpMode is running
                runLift();
                //telemetry.addData("Lift Position", ();
                telemetry.update();
            }
        }
    }


    /**
     * The runLift function is designed to run the lift to lift the robot in endgame.
     * When running this function, the lift motor will run to a defined position and then hold there, even if the driver releases the button.
     * This function will be activated by pressing two buttons at the same time for safety.
     */
    private void runLift() {

    }





}
