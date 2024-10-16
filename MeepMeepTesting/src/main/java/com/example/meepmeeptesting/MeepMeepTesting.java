package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity leftRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity rightRedCope = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity rightRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        leftRed.runAction(leftRed.getDrive().actionBuilder(new Pose2d(-10, -58, Math.toRadians(270)))
                .strafeTo(new Vector2d(-10, -33)) //go hang specimen
                .waitSeconds(2)

                .splineToLinearHeading(new Pose2d(-48, -40,Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(2) //pick up sample

                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2) //deposit sample

                .strafeToLinearHeading(new Vector2d(-58, -40), Math.toRadians(90))
                .waitSeconds(2) //pick up sample

                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2) //deposit sample

                .strafeToLinearHeading(new Vector2d(-55, -25), Math.toRadians(180))
                .waitSeconds(2) //pick up sample

                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2) //deposit sample

                .strafeToLinearHeading(new Vector2d(-27, 12), Math.toRadians(0))
                .build());

        rightRedCope.runAction(rightRedCope.getDrive().actionBuilder(new Pose2d(12, -58, Math.toRadians(270)))
                .strafeTo(new Vector2d(12, -33))
                .waitSeconds(1)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(18,-40), Math.toRadians(0))

//
//
//
//
//
//
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

        rightRed.runAction(rightRed.getDrive().actionBuilder(new Pose2d(10, -58, Math.toRadians(270)))
                .strafeTo(new Vector2d(10, -33)) //go hang specimen
                .waitSeconds(2)

                .strafeTo(new Vector2d(20, -50))
                .splineToSplineHeading(new Pose2d(45,-10, Math.toRadians(90)), Math.toRadians(35))
                .strafeTo(new Vector2d(45,-50)) // push sample into obs. zone

//                .strafeTo(new Vector2d(45, -15))
//                .splineToConstantHeading(new Vector2d(50,-10), Math.toRadians(0))
//                .strafeTo(new Vector2d(50,-50)) // push sample into obs. zone

//
////                .strafeTo(new Vector2d(46,-48))
////
////                .waitSeconds(.1)
////
////                .splineToConstantHeading(new Vector2d(46,-20), Math.toRadians(90))
////                .splineToConstantHeading(new Vector2d(46+10,-10), Math.toRadians(0))
////                .strafeTo(new Vector2d(46+10,-47))
////
////                .waitSeconds(.1)
////
////                .splineToConstantHeading(new Vector2d(46+10,-20), Math.toRadians(90))
////                .splineToConstantHeading(new Vector2d(61,-10), Math.toRadians(0))
////                .strafeTo(new Vector2d(61,-48))
////
////                .setReversed(false)
////
////                .strafeToLinearHeading(new Vector2d(50, -38),Math.toRadians(135))
////                .strafeToLinearHeading(new Vector2d(48, -47),Math.toRadians(270))
////                .waitSeconds(1)
////
////                .strafeToConstantHeading(new Vector2d(9, -33))
////                .waitSeconds(.75)
////                .strafeToConstantHeading(new Vector2d(48, -47))
////                .waitSeconds(1)
////
////
////                .strafeToConstantHeading(new Vector2d(7, -33))
////                .waitSeconds(.75)
////                .strafeToConstantHeading(new Vector2d(48, -47))
////                .waitSeconds(1)
////
////                .strafeToConstantHeading(new Vector2d(5, -33))
////                .waitSeconds(.75)
////                .strafeToConstantHeading(new Vector2d(48, -47))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
//        .addEntity(leftRed);
//        .addEntity(rightRedCope);
        .addEntity(rightRed)
        .start();
                //.start();

    }
}