package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp
public class TELEOP_LIFT extends LinearOpMode {
//TODO ************* THIS IS BASED ON THE TELEOP FROM THE 6TH GRADE BOT, UPDATE IT FOR THE 23168 BOT!!!!

    // Declare OpMode members.
    private DcMotor lift;

    private ElapsedTime runtime = new ElapsedTime();
    final int extensionposition = 0;
    final int packagedposition = 0;
    /////////////////////////////////////////////////////////////////////////
    // Declare variables



    @Override
    public void runOpMode() {
        lift = hardwareMap.get(DcMotor.class, "motor-lift");

        // Establishing the direction and mode for the motors
        lift.setTargetPosition(0);
        lift.setPower(0.5);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                lift();
            }
        }
    }


    /**
     * The lift function is designed to run the lift to lift the robot in endgame.
     * When running this function, the lift motor will run to a defined position and adjust for ally height with manual player input.
     * This function will be eventually activated only after a timer for 180 seconds goes off and a button is pressed for safety.
     */


private void lift() {
    if (runtime.time() > 100) {
        if (gamepad1.start) {
            lift.setTargetPosition(extensionposition);
        } else if (gamepad1.back) {
            lift.setTargetPosition(packagedposition);
        }
    }
    //telemetry.addData("Lift Position", ();
    telemetry.addData("controller start", gamepad1.start);
    telemetry.addData("controller back", gamepad1.back);
    telemetry.addData("expected lift position", lift.getTargetPosition());
    telemetry.addData("lift position", lift.getCurrentPosition());
    telemetry.addData("time", runtime.time());
    telemetry.update();
}



}
