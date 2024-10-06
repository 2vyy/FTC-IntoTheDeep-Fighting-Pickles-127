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

        RoadRunnerBotEntity myBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity teleBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-13, -58, Math.toRadians(90)))
                .splineTo(new Vector2d(-13, -33), Math.toRadians(90))
                .waitSeconds(2)
                .setReversed(true)
                .strafeTo(new Vector2d(-48, -38))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(45))
                .waitSeconds(2)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-58, -38), Math.toRadians(90))
                .waitSeconds(2)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(45))
                .waitSeconds(2)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-58, -25), Math.toRadians(180))
                .waitSeconds(2)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-50, -53), Math.toRadians(45))
                .waitSeconds(2)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-23, -10), Math.toRadians(0))
                .build());

        myBot1.runAction(myBot1.getDrive().actionBuilder(new Pose2d(12, -58, Math.toRadians(90)))
                .strafeTo(new Vector2d(12, -33))
                .waitSeconds(.75)
//
//
//                .setReversed(true)
//
//                .splineToConstantHeading(new Vector2d(18,-40), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(32,-20), Math.toRadians(50))
//                .splineToConstantHeading(new Vector2d(46,-10), Math.toRadians(0))
//                .strafeTo(new Vector2d(46,-48))
//
//                .waitSeconds(.1)
//
//                .splineToConstantHeading(new Vector2d(46,-20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(46+10,-10), Math.toRadians(0))
//                .strafeTo(new Vector2d(46+10,-47))
//
//                .waitSeconds(.1)
//
//                .splineToConstantHeading(new Vector2d(46+10,-20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(61,-10), Math.toRadians(0))
//                .strafeTo(new Vector2d(61,-48))
//
//                .setReversed(false)
//
//                .strafeToLinearHeading(new Vector2d(50, -38),Math.toRadians(135))
//                .strafeToLinearHeading(new Vector2d(48, -47),Math.toRadians(270))
//                .waitSeconds(1)
//
//                .strafeToConstantHeading(new Vector2d(9, -33))
//                .waitSeconds(.75)
//                .strafeToConstantHeading(new Vector2d(48, -47))
//                .waitSeconds(1)
//
//
//                .strafeToConstantHeading(new Vector2d(7, -33))
//                .waitSeconds(.75)
//                .strafeToConstantHeading(new Vector2d(48, -47))
//                .waitSeconds(1)
//
//                .strafeToConstantHeading(new Vector2d(5, -33))
//                .waitSeconds(.75)
//                .strafeToConstantHeading(new Vector2d(48, -47))






                .strafeToLinearHeading(new Vector2d(37, -40), Math.toRadians(55))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(37, -50), Math.toRadians(315))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(37+12, -40), Math.toRadians(55))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(37+12, -43), Math.toRadians(270))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(51, -26), Math.toRadians(0))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(48, -43), Math.toRadians(270))

                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(12, -33), Math.toRadians(90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(10, -36), Math.toRadians(90))
                .waitSeconds(.5)


                .strafeToLinearHeading(new Vector2d(48, -43), Math.toRadians(270))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(8, -33), Math.toRadians(90))
                .waitSeconds(.5)


                .strafeToLinearHeading(new Vector2d(48, -43), Math.toRadians(270))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(6, -33), Math.toRadians(90))

                .build());

        teleBot.runAction(teleBot.getDrive().actionBuilder(new Pose2d(-23, -10, Math.toRadians(0)))
                .waitSeconds(3)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-50, -53), Math.toRadians(45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-23, -10), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        //.addEntity(myBot);
        //.addEntity(myBot1);
        .addEntity(teleBot);
        meepMeep.start();
                //.start();

    }
}