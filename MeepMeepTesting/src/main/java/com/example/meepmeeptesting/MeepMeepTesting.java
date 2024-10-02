package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-13, -58, Math.toRadians(90)))
                .splineTo(new Vector2d(-13, -33), Math.toRadians(90))
                .setReversed(true)
                .strafeTo(new Vector2d(-48, -38))
                .strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(45))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-58, -38), Math.toRadians(90))
//                .setReversed(false)
//                .splineTo(new Vector2d(-53, -53), Math.toRadians(45))




                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}