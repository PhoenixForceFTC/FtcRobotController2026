package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 10.80737288830478)
                .build();

        myBot.runAction(myBot.getDrive()
                //--- starting position
                .actionBuilder(new Pose2d(-38, -53, Math.toRadians(90)))
                //.waitSeconds(2)
                //--- align to shoot
                .strafeToSplineHeading(new Vector2d(-36, -36), Math.toRadians(235))
                .waitSeconds(1) //--- shoot (1-3)

                //--- align with balls
                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
                //.waitSeconds(2)
                //--- drive forward pick up balls (1)
                .strafeToSplineHeading(new Vector2d(-16, -46), Math.toRadians(0))
                //.waitSeconds(1)
                //--- align to shoot
                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-36, -36), Math.toRadians(235))
                .waitSeconds(1) //--- shoot (4-6)

                //--- align with balls
                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
                //.waitSeconds(2)
                //--- open the balls
                .strafeToSplineHeading(new Vector2d(-5, -46), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-5, -50), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-5, -46), Math.toRadians(0))
                //.waitSeconds(2)
                //--- drive forward pick up balls (2)
                .strafeToSplineHeading(new Vector2d(7, -46), Math.toRadians(0))
                //.waitSeconds(1)
                //--- align to shoot
                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-36, -36), Math.toRadians(235))
                .waitSeconds(1) //--- shoot (7-9)

                //--- align with balls
                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
                //.waitSeconds(2)
                //--- drive forward pick up balls (3)
                .strafeToSplineHeading(new Vector2d(30, -46), Math.toRadians(0))
                //.waitSeconds(1)
                //--- align to shoot
                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-36, -36), Math.toRadians(235))
                .waitSeconds(1) //--- shoot (10-12)

                //--- align with balls
                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
                //.waitSeconds(2)
                //--- drive forward pick up balls - in player zone (4)
                .strafeToSplineHeading(new Vector2d(50, -46), Math.toRadians(0))
                //.waitSeconds(2)
                .strafeToSplineHeading(new Vector2d(60, -50), Math.toRadians(270))
                //.waitSeconds(2)
                .strafeToSplineHeading(new Vector2d(60, -60), Math.toRadians(270))
                //.waitSeconds(1)
                //--- align with balls
                .strafeToSplineHeading(new Vector2d(50, -46), Math.toRadians(0))
                //.waitSeconds(2)
                //--- align to shoot
                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-36, -36), Math.toRadians(235))
                .waitSeconds(1) //--- shoot (13-15)

                //--- move off the line
                .strafeToSplineHeading(new Vector2d(-36, -50), Math.toRadians(235))
                .waitSeconds(1) //--- shoot (13-15)

                .waitSeconds(20)

                .build());

//        myBot.runAction(myBot.getDrive()
//                //--- starting position
//                .actionBuilder(new Pose2d(-55, -45, Math.toRadians(235)))
//                //.waitSeconds(2)
//                //--- align to shoot
//                .strafeToSplineHeading(new Vector2d(-36, -36), Math.toRadians(235))
//                .waitSeconds(1) //--- shoot (1-3)
//
//                //--- align with balls
//                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
//                //.waitSeconds(2)
//                //--- drive forward pick up balls (1)
//                .strafeToSplineHeading(new Vector2d(-16, -46), Math.toRadians(0))
//                //.waitSeconds(1)
//                //--- align to shoot
//                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
//                .strafeToSplineHeading(new Vector2d(-36, -36), Math.toRadians(235))
//                .waitSeconds(1) //--- shoot (4-6)
//
//                //--- align with balls
//                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
//                //.waitSeconds(2)
//                //--- open the balls
//                .strafeToSplineHeading(new Vector2d(-5, -46), Math.toRadians(0))
//                .strafeToSplineHeading(new Vector2d(-5, -50), Math.toRadians(0))
//                .strafeToSplineHeading(new Vector2d(-5, -46), Math.toRadians(0))
//                //.waitSeconds(2)
//                //--- drive forward pick up balls (2)
//                .strafeToSplineHeading(new Vector2d(7, -46), Math.toRadians(0))
//                //.waitSeconds(1)
//                //--- align to shoot
//                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
//                .strafeToSplineHeading(new Vector2d(-36, -36), Math.toRadians(235))
//                .waitSeconds(1) //--- shoot (7-9)
//
//                //--- align with balls
//                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
//                //.waitSeconds(2)
//                //--- drive forward pick up balls (3)
//                .strafeToSplineHeading(new Vector2d(30, -46), Math.toRadians(0))
//                //.waitSeconds(1)
//                //--- align to shoot
//                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
//                .strafeToSplineHeading(new Vector2d(-36, -36), Math.toRadians(235))
//                .waitSeconds(1) //--- shoot (10-12)
//
//                //--- align with balls
//                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
//                //.waitSeconds(2)
//                //--- drive forward pick up balls - in player zone (4)
//                .strafeToSplineHeading(new Vector2d(50, -46), Math.toRadians(0))
//                //.waitSeconds(2)
//                .strafeToSplineHeading(new Vector2d(60, -50), Math.toRadians(270))
//                //.waitSeconds(2)
//                .strafeToSplineHeading(new Vector2d(60, -60), Math.toRadians(270))
//                //.waitSeconds(1)
//                //--- align with balls
//                .strafeToSplineHeading(new Vector2d(50, -46), Math.toRadians(0))
//                //.waitSeconds(2)
//                //--- align to shoot
//                .strafeToSplineHeading(new Vector2d(-26, -46), Math.toRadians(0))
//                .strafeToSplineHeading(new Vector2d(-36, -36), Math.toRadians(235))
//                .waitSeconds(1) //--- shoot (13-15)
//
//                //--- move off the line
//                .strafeToSplineHeading(new Vector2d(-36, -50), Math.toRadians(235))
//                .waitSeconds(1) //--- shoot (13-15)
//
//                .waitSeconds(20)
//
//                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


// package com.example.meepmeeptesting;

// import com.acmerobotics.roadrunner.Pose2d;
// import com.noahbres.meepmeep.MeepMeep;
// import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
// import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// public class MeepMeepTesting {
//     public static void main(String[] args) {
//         MeepMeep meepMeep = new MeepMeep(800);

//         RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                 // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                 .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                 .build();

//         myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
//                 .lineToX(30)
//                 .turn(Math.toRadians(90))
//                 .lineToY(30)
//                 .turn(Math.toRadians(90))
//                 .lineToX(0)
//                 .turn(Math.toRadians(90))
//                 .lineToY(0)
//                 .turn(Math.toRadians(90))
//                 .build());

//         meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
//                 .setDarkMode(true)
//                 .setBackgroundAlpha(0.95f)
//                 .addEntity(myBot)
//                 .start();
//     }
// }

