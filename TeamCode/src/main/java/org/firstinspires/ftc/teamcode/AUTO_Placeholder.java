/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
 * This is our development OpMode for AUTO, the first one we're experimenting with in java for 23168
 * We will integrate to following in this mode:
 *    - Limelight3a (limelight)
 *    - Sparkfun OTOS (sensor_otos)
 *    - Mecanum drive (motors left_front, left_back, right_front, right_back)
 *    - Road Runner using a localizer for the OTOS developed by @j5155 on the FTC Discord https://github.com/jdhs-ftc
 *
 * All of the drive configuration is done via MecanumDrive.java, you do not have to manage that here.
 *
 * THIS MODE IS CONFIGURED FOR WAFFLES, NOT PANCAKE
 *
 */
@Autonomous(name="AUTO-Placeholder", group="AUTO", preselectTeleOp = "REVStarterBotTeleOpJava")
//@Autonomous(name="AUTO-BLUE-1", group="AUTO", preselectTeleOp = "TELEOP-BLUE (Blocks to Java)")
//@Disabled
public class AUTO_Placeholder extends LinearOpMode {

    // Declare OpMode members.
    private Limelight3A limelight;
    private SparkFunOTOS otos;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    /////////////////////////////////////////////////////////////////////////
    // Declare variables
    //TODO ********** Set the Alliance Color  **************


    //TODO *********** Set the starting pose for the robot based on the alliance start position,
    // X and Y in INCHES from the center of the field, heading in RADIANS (or convert DEGREES to
    // RADIANS by multiplying the value in DEGREES by Math.PI/180
    Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
    @Override
    public void runOpMode() {

        //Set Hardware Map

        otos = hardwareMap.get(SparkFunOTOS.class, "sensor-otos");

        //initDevices(); // Initialize all motors, servos, sensors



        //Instantiate the roadrunner Mecanum drive (via the OTOS localizer)
        //SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        //Set all actuator target positions


        //Set all field positions



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ////////////////////////////////////////////////////////////////////////////////////
        // Wait for the game to start (driver presses START)
        ////////////////////////////////////////////////////////////////////////////////////
        waitForStart();

        if(isStopRequested()) return;

        //Build the actions for our AUTO mode
        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        // First Move
                        .lineToX(30)
                        .turn(Math.toRadians(90))
                        .lineToY(30)
                        .turn(Math.toRadians(90))
                        .lineToX(0)
                        .turn(Math.toRadians(90))
                        .lineToY(0)
                        .turn(Math.toRadians(90))

                        .build());



        SparkFunOTOS.Pose2D pos = otos.getPosition(); //Read OTOS Pose for telemetry


        telemetry.addLine();
        telemetry.addData("OTOS Data", "X: (%.1f), Y: (%.1f), H: (%.2f)", pos.x,pos.y,pos.h);
        telemetry.update();

        // run until the end of the match (driver presses STOP)
    }

    ///////////////////////////////////////////////////
    //PUBLIC CLASSES FOR ROADRUNNER ACTION DEFINITIONS
    //////////////////////////////////////////////////













    //////////////////////////////////////////////////
    // PRIVATE VOIDS REFERENCED IN THE PUBLIC VOID
    //////////////////////////////////////////////////

    // Initializes all devices: sensors and actuators, to clean up the code
    private void initDevices(){
        //INIT LIMELIGHT


        // Initialize actuators

    }



}
