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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-48, -42, Math.toRadians(231)))
                .waitSeconds(2)
                .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(225))
                .waitSeconds(2)

                .strafeToSplineHeading(new Vector2d(-12, -30), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(-12, -48), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(225))
                .waitSeconds(2)

                .strafeToSplineHeading(new Vector2d(0, -34), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(12, -48), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(225))
                .waitSeconds(2)

                .build());

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

