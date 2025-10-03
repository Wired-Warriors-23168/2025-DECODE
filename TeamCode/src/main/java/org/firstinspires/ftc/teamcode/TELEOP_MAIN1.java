package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class TEST_LINEAR_ACTUATOR extends LinearOpMode {
//TODO ************* THIS IS THE TELEOP FROM THE 6TH GRADE BOT, UPDATE IT FOR THE 23168 BOT!!!!
    public Servo myServo;

    // Setting our velocity targets. These values are in ticks per second!


    @Override
    public void runOpMode() {
        myServo = hardwareMap.get(Servo.class, "myServo");


        // Check if the servo supports PwmControl before casting
        if (myServo instanceof PwmControl) {
            PwmControl pwmControl = (PwmControl) myServo;
            // Set a custom PWM range (optional, default is usually fine)
            PwmControl.PwmRange range = new PwmControl.PwmRange(500, 2500);
            pwmControl.setPwmRange(range);
            // Enable PWM output
            pwmControl.setPwmEnable();
        }

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our methods while the OpMode is running
                manualLinearActuator();

            }
        }
    }private void manualLinearActuator() {
        // Manual control for the linear actuator
        if (gamepad1.dpad_left) {
            myServo.setPosition(0.2);
        } else if (gamepad1.dpad_right) {
            myServo.setPosition(0.8);
        }
    }


}
