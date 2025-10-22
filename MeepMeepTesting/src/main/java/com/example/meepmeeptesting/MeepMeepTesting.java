package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-63, 30, Math.toRadians(90)))
                .strafeTo(new Vector2d(shootX, shootY))
                .waitSeconds(shootTime)
                .strafeTo(new Vector2d(-11.4, 30))
                .strafeTo(new Vector2d(-11.4, 56))
                .strafeTo(new Vector2d(shootX, shootY))
                .waitSeconds(shootTime)
                .strafeTo(new Vector2d(12.2, 30))
                .strafeTo(new Vector2d(12.2, 56))
                .strafeTo(new Vector2d(shootX, shootY))
                .waitSeconds(shootTime)
                .strafeTo(new Vector2d(35.8, 30))
                .strafeTo(new Vector2d(35.8, 56))
                .strafeTo(new Vector2d(shootX, shootY))
                .waitSeconds(shootTime)
                .strafeTo(new Vector2d(shootX, shootY))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}