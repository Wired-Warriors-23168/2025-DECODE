package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public abstract class TEST_Linear_Actuator extends LinearOpMode {

    public Servo myServo;

    public void init(HardwareMap hardwareMap) {
        myServo = hardwareMap.get(Servo.class, "myServo");

        // Check if the servo supports PwmControl before casting
        if (myServo instanceof PwmControl) {
            PwmControl pwmControl = (PwmControl) myServo;
            // Set a custom PWM range (optional, default is usually fine)
            //pwmControl.setPwmRange(500, 2500, );
            // Enable PWM output
            pwmControl.setPwmEnable();
        }

        // Set servo position (this will generate the corresponding PWM signal)
        // myServo.setPosition(0.5);

        if (gamepad1.dpad_left) {
            myServo.setPosition(0.2);
        } else if (gamepad1.dpad_right) {
            myServo.setPosition(0.8);
        }
    }
}