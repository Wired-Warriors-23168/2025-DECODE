package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, -55.5, Math.toRadians(-90)))
                //.lineToX(30)
                //.lineToY(30)
                .setReversed(true)
                .splineTo(new Vector2d(-28, -28), Math.toRadians(45))

                .waitSeconds(8) //placeholder for shooting

                .setReversed(false)
                .setTangent(Math.toRadians(45)) //heading to leave position from
                .splineToLinearHeading(new Pose2d(24, -24, Math.toRadians(0)), Math.toRadians(0))  //final heading, heading to enter final heading from

                .strafeTo(new Vector2d(24,-46))

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(57, -55, Math.toRadians(-90)), Math.toRadians(-100))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}