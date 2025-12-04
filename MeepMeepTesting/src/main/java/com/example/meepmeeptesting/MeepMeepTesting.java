package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double shootX = -63;
        double shootY = 30;
        double shootTime = 2;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(85, 75, Math.toRadians(180), Math.toRadians(180), 17.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-58.5, -45, Math.toRadians(144)))
                .setTangent(Math.toRadians(65))
                .splineToSplineHeading(new Pose2d(-16,-16,Math.toRadians(-135)),Math.toRadians(0))
                        .waitSeconds(1)
//                .turnTo(Math.toRadians(-135))
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-20,-20,Math.toRadians(-135)),Math.toRadians(45))
//                        .waitSeconds(1)
//
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-12.25,-30.25,Math.toRadians(-90)),Math.toRadians(-90))
//                .lineToY(-36,
//                        // override velocity constraint - slow down the move
//                        new TranslationalVelConstraint(20.0))
//                .waitSeconds(1)
//                .lineToY(-41,
//                        // override velocity constraint - slow down the move
//                        new TranslationalVelConstraint(20.0))
//                .waitSeconds(1)
//                .lineToY(-46,
//                        // override velocity constraint - slow down the move
//                        new TranslationalVelConstraint(20.0))
//                .waitSeconds(1)
//
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-30,-30,Math.toRadians(-135)),Math.toRadians(45),
//                        // override velocity constraint - slow down the move
//                        new TranslationalVelConstraint(50))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}