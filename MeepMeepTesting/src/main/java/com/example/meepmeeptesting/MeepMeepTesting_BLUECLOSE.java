package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_BLUECLOSE {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d beginPose = new Pose2d(62, -14.5, Math.toRadians(180)); //NEW STARTING POSITION

        //Set AUTO waypoints
        Vector2d obeliskVector = new Vector2d(55,-12);  //pose to read the obelisk
        double obeliskHeading = Math.toRadians(-157);  //heading to read the obelisk
        Pose2d obeliskPose = new Pose2d(55,-12,obeliskHeading); //pose to read the obelisk

        Pose2d shootPoseLong = new Pose2d(55,-12,Math.toRadians(-157));    //pose to shoot the pattern
        Pose2d shootPoseClose = new Pose2d(-14,-14,Math.toRadians(-135));    //pose to shoot the pattern

        Pose2d intakePose1 = new Pose2d(35,-29,Math.toRadians(-90));  //pose to start intake artifacts from first row
        Pose2d intakePickup1 = new Pose2d(35,-49,Math.toRadians(-90));  //pose to end intake artifacts from first row

        Pose2d intakePose2 = new Pose2d(11,-29,Math.toRadians(-90));  //pose to start intake artifacts from second row
        Pose2d intakePickup2 = new Pose2d(11,-49,Math.toRadians(-90));  //pose to end intake artifacts from second row

        Pose2d endPose = new Pose2d(59,-40,Math.toRadians(180));      //pose at end of AUTO



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(85, 75, Math.toRadians(180), Math.toRadians(180), 17.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(beginPose)
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(obeliskPose,Math.toRadians(180))
//                .splineToConstantHeading(obeliskVector,obeliskHeading)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(shootPoseLong,Math.toRadians(180))
//                .setTangent(Math.toRadians(-135))
//                .splineToLinearHeading(intakePose1,Math.toRadians(-120))
//                        .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(intakePickup1,Math.toRadians(-90))
//                .setTangent(Math.toRadians(30))
//                .splineToLinearHeading(shootPoseLong,Math.toRadians(30))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(endPose,Math.toRadians(-90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}