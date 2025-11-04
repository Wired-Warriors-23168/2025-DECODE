package org.firstinspires.ftc.teamcode;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

//@Config //Required to be able to tune parameters in FTCDashboard
@Disabled
@TeleOp
public class TEST_LINEAR_ACTUATOR extends LinearOpMode {
    //TODO ************* THIS IS THE TELEOP FROM THE 6TH GRADE BOT, UPDATE IT FOR THE 23168 BOT!!!!
    public Servo myServo;
    private DcMotor flywheel;
    //public double setPos;

    // Declare variables
    // Set as "static" and not "final" in order to be able to tune parameters in FTCDashboard
    // Setting our velocity targets. These values are in ticks per second!
    private static final int bankVelocity = 1300;
    //private static final int farVelocity = 1900;
    private static final int maxVelocity = 2200;
    public static double setPos = 0.5;

    @Override
    public void runOpMode() {
        myServo = hardwareMap.get(Servo.class, "myServo");
        flywheel = hardwareMap.get(DcMotor.class, "motor-flywheel");

        // Establishing the direction and mode for the motors
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        // Check if the servo supports PwmControl before casting
        if (myServo instanceof PwmControl) {
            PwmControl pwmControl = (PwmControl) myServo;
            // Set a custom PWM range (optional, default is usually fine)
            PwmControl.PwmRange range = new PwmControl.PwmRange(1000, 2000);
            pwmControl.setPwmRange(range);
            // Enable PWM output
            pwmControl.setPwmEnable();
            setPos = 0.5;
            myServo.setPosition(setPos);
        }

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our methods while the OpMode is running
                manualLinearActuator();
                setFlywheelVelocity();
//                myServo.setPosition(setPos);

                //Set up the telemetry to the driver hub
                telemetry.addData("Set Position", setPos);
                telemetry.update();

//                // Set up channels for display in FTCDashboard
//                FtcDashboard dashboard = FtcDashboard.getInstance();
//                TelemetryPacket packet = new TelemetryPacket();
//
//                // Send a value to the dashboard for graphing
//                dashboard.sendTelemetryPacket(packet); // Always send the packet
//                packet.put("Set Position", setPos); // Robot-specific data
//
//                //Set up the Field overlay
//                packet.fieldOverlay()
//                        .setFill("blue")
//                        .fillRect(-20, -20, 40, 40);

            }
        }
    }


    /**
     * This if/else statement contains the controls for the flywheel, both manual and auto.
     * Circle and Square will spin up ONLY the flywheel to the target velocity set.
     * The bumpers will activate the flywheel, Core Hex feeder, and servo to cycle a series of balls.
     */
    private void setFlywheelVelocity() {
        if (gamepad1.options) {
            flywheel.setPower(-0.5);
        } else if (gamepad1.a) {
            ((DcMotorEx) flywheel).setVelocity(bankVelocity);
        } else if (gamepad1.b) {
            ((DcMotorEx) flywheel).setVelocity(maxVelocity);
        } else {
            ((DcMotorEx) flywheel).setVelocity(0);

        }
    }

    private void manualLinearActuator() {
        // Manual control for the linear actuator

        if (gamepad1.dpadLeftWasPressed()) {
            setPos = setPos - 0.05;
            setPos = max(setPos, 0);
            myServo.setPosition(setPos);
        } else if (gamepad1.dpadRightWasPressed()) {
            setPos = setPos + 0.05;
            setPos = min(setPos, 1);
            myServo.setPosition(setPos);
        }
    }

}

