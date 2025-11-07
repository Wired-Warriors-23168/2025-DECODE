package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AUTO4 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14.656)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62.5, -32.5, Math.toRadians(-90)))
                .setReversed(false)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-2,-12,Math.toRadians(-140)),Math.toRadians(170))

                //Corral three balls on the goal
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(1,-28,Math.toRadians(-90)),Math.toRadians(-90))
                .lineToY(-46)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-47,-47,Math.toRadians(-45)),Math.toRadians(-180))

                //Park in the center near the line to draw a foul
                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(8,-12,Math.toRadians(-90)),Math.toRadians(0))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_PAPER)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}