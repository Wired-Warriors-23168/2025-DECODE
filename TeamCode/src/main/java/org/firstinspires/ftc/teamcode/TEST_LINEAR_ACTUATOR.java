package org.firstinspires.ftc.teamcode;

import static java.lang.Math.max;
import static java.lang.Math.min;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;


@TeleOp
public class TEST_LINEAR_ACTUATOR extends LinearOpMode {
//TODO ************* THIS IS THE TELEOP FROM THE 6TH GRADE BOT, UPDATE IT FOR THE 23168 BOT!!!!
    public Servo myServo;
    public double setPos;

    // Setting our velocity targets. These values are in ticks per second!


    @Override
    public void runOpMode() {
        myServo = hardwareMap.get(Servo.class, "myServo");


        // Check if the servo supports PwmControl before casting
        if (myServo instanceof PwmControl) {
            PwmControl pwmControl = (PwmControl) myServo;
            // Set a custom PWM range (optional, default is usually fine)
            PwmControl.PwmRange range = new PwmControl.PwmRange(1000, 2000);
            pwmControl.setPwmRange(range);
            // Enable PWM output
            pwmControl.setPwmEnable();
            setPos=0.5;
            myServo.setPosition(setPos);
        }

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our methods while the OpMode is running
                manualLinearActuator();
                telemetry.addData("Set Position", setPos);
                telemetry.update();

            }
        }
    }private void manualLinearActuator() {
        // Manual control for the linear actuator

        if (gamepad1.dpadLeftWasPressed()) {
            setPos=setPos-0.05;
            setPos=max(setPos,0);
            myServo.setPosition(setPos);
        } else if (gamepad1.dpadRightWasPressed()) {
            setPos=setPos+0.05;
            setPos=min(setPos,1);
            myServo.setPosition(setPos);
        }
    }


}
