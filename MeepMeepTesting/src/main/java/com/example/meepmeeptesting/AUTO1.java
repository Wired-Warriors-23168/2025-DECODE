package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AUTO1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(-62.5, 35, Math.toRadians(180), Math.toRadians(-90), 14.656)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-62.5, -35.5, Math.toRadians(-90)))
                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-27,-27,Math.toRadians(-135)),Math.toRadians(45))

                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(8,-14,Math.toRadians(-90)),Math.toRadians(0))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_PAPER)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}