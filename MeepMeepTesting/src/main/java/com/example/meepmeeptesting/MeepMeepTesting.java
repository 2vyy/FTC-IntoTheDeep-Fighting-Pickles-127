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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-32, -61, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225)) //move to basket
                .waitSeconds(1) //drop off 1st sample

                .strafeToLinearHeading(new Vector2d(-48, -40), Math.toRadians(90)) //move to 2nd sample
                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225)) //move to basket
//                .waitSeconds(1) //drop off 2nd sample
//
//                .strafeToLinearHeading(new Vector2d(-48-10, -40), Math.toRadians(90)) //move to 3rd sample
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225)) //move to basket
//                .waitSeconds(1) //drop off 3rd sample
//
//                .strafeToLinearHeading(new Vector2d(-48-6, -26), Math.toRadians(180)) //move to 4th sample
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225)) //move to basket
//                .waitSeconds(1) //drop off 4th sample
//
//                .strafeToLinearHeading(new Vector2d(-36, -10), Math.toRadians(0)) //move to center
//                .strafeToLinearHeading(new Vector2d(-24, -10), Math.toRadians(0)) //get closer
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}