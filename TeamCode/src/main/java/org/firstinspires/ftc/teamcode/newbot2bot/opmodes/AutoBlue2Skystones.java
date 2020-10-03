package org.firstinspires.ftc.teamcode.newbot2bot.opmodes;

import android.graphics.PointF;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.newbot2bot.NewBot2Bot;

@Disabled
@Autonomous(name="NB2B Blue 2 Skystones", group="No")
public class AutoBlue2Skystones extends NB2BAuto {
    public NewBot2Bot bot = new NewBot2Bot(MechBot.MotorType.NeverestOrbital20, 32.75, 31.75, 31.75,
            BNO055Enhanced.AxesMap.XZY, BNO055Enhanced.AxesSign.PNP, 1);

    final OpenGLMatrix landscapePhoneLocation = OpenGLMatrix.translation(-216f, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC
            , AxesOrder.YXY, AngleUnit.DEGREES, 90, 180, -9.6f));

    public AutoBlue2Skystones() {super(AllianceColor.BLUE);}

    @Override
    public void runLoggingOpmode() {

        bot.init(hardwareMap, 180.0f, true);
        super.setBot(bot);

        //Finding SkyStone
        setTargetLocations(AllianceColor.BLUE);

        VuforiaNavigator.activate("Skystone", targetLocations, landscapePhoneLocation, VuforiaLocalizer.CameraDirection.BACK, null);
        CameraDevice.getInstance().setField("zoom", "21");
        VuforiaNavigator.setFlashTorchMode(true);

        waitForStart();
        skyStonePosition = determinePositionBlobStatistics(SkyStonePosition.LEFT);
        telemetry.addData("Sky Stone Position", "%s", skyStonePosition.toString());
        telemetry.update();

        CameraDevice.getInstance().setField("zoom", "0");

        resetOdometry(X_START, Y_START, -90);



        BetaLog.dd("AUTOBLUE2SKYSTONES - START","X = %.1f  Y = %.1f  HEADING = %.1f",
                robotXYTheta[0],robotXYTheta[1],robotXYTheta[2] * 180.0/Math.PI);
        //Drive to block line
        driveDirectionGyro(FAST_DRIVE_SPEED, -90, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] < BLOCK_LINE_Y / 3;
            }
        });
        BetaLog.dd("AUTOBLUE2SKYSTONES - DRIVE1","X = %.1f  Y = %.1f  HEADING = %.1f",
                robotXYTheta[0],robotXYTheta[1],robotXYTheta[2] * 180.0/Math.PI);
        bot.setRightFlippoDown();
        bot.setRightFlippoFingerOpen();

        turnToHeadingGyro(0, STANDARD_TOLERANCE, STANDARD_LATENCY);
        BetaLog.dd("AUTOBLUE2SKYSTONES - TURN1","X = %.1f  Y = %.1f  HEADING = %.1f",
                robotXYTheta[0],robotXYTheta[1],robotXYTheta[2] * 180.0/Math.PI);
        //Move to skystone and intake
        final float targetX = (skyStonePosition == SkyStonePosition.RIGHT?84:(skyStonePosition == SkyStonePosition.CENTER?104:124));
        final boolean toWall = robotXYTheta[0] > targetX;
        /*driveDirectionGyro(FAST_DRIVE_SPEED, toWall ? 180 : 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return toWall ? robotXYTheta[0] < targetX : robotXYTheta[0] > targetX;
            }
        });*/
        driveToPositionGyro(FAST_DRIVE_SPEED, 20, targetX, robotXYTheta[1], 0);
        BetaLog.dd("AUTOBLUE2SKYSTONES - DRIVE2","X = %.1f  Y = %.1f  HEADING = %.1f",
                robotXYTheta[0],robotXYTheta[1],robotXYTheta[2] * 180.0/Math.PI);
        driveDirectionGyro(STD_DRIVE_SPEED, -90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] < BLOCK_GRAB_Y;
            }
        });

        bot.setRightFlippoFingerClosed();
        bot.setDragServoUp();
        sleep(400);
        bot.rightFlippo.setPosition(.8);

        driveDirectionGyro(FAST_DRIVE_SPEED, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > BLOCK_LINE_Y + 50;
            }
        });

        deliverBlock(80, true);

        driveDirectionGyro(FAST_DRIVE_SPEED, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > BLOCK_GRAB_Y + 10;
            }
        });
        turnToHeadingGyro(-90, STANDARD_TOLERANCE, STANDARD_LATENCY, 150);
        driveToPositionGyro(FAST_DRIVE_SPEED, STD_DRIVE_SPEED, robotXYTheta[0], -FOUNDATION_POSITION, -90);

        bot.setRightFlippoUp();
        bot.setLeftFlippoUp();
        bot.setRightFlippoFingerClosed();
        bot.setLeftFlippoFingerClosed();

        dragFoundationArc(RotationDirection.COUNTER_CLOCK, false);

        driveToPositionGyro(FAST_DRIVE_SPEED, STD_DRIVE_SPEED, robotXYTheta[0] - 10, BLOCK_LINE_Y + 55, 0);
        bot.setDragServoDown();

        //Drive back to loading area

//        driveDirectionGyro(180, 0, new DriveSpeedControl() {
//            float distance;
//            boolean nearWall = false;
//
//            @Override
//            public boolean isTrue() {
//                if(!nearWall) {
//                    if((distance = (float)bot.backDistSensor.getDistance(DistanceUnit.CM)) <= 6 && robotXYTheta[0] < FIELD_WIDTH * .35) {
//                        nearWall = true;
//                        resetOdometry(8 + BOT_LENGTH / 2, robotXYTheta[1]);
//                    }
//                    return false;
//                }
//
//                return robotXYTheta[0] < BOT_LENGTH / 2;
//            }
//
//            @Override
//            public float speed() {
//                return (distance < 70 && robotXYTheta[0] < FIELD_WIDTH * .35) ? STD_DRIVE_SPEED : VERY_FAST_DRIVE_SPEED;
//            }
//        });

        driveDirectionGyro(VERY_FAST_DRIVE_SPEED, 180, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                if(robotXYTheta[0] <= FIELD_WIDTH * .4) {
                    bot.setRightFlippoFingerOpen();
                    bot.setRightFlippoDown();
                }

                return (bot.backDistSensor.getDistance(DistanceUnit.CM) < 70 && robotXYTheta[0] < FIELD_WIDTH * .35) || (bot.backDistSensor.getDistance(DistanceUnit.CM) > 800 && robotXYTheta[0] < FIELD_WIDTH * .15);
            }
        });

        bot.setRightFlippoFingerOpen();
        bot.setRightFlippoDown();

        sleep(300);

        turnToHeadingGyro(0, STANDARD_TOLERANCE, STANDARD_LATENCY);

        float avgXDist = 0;
        float avgYDist = 0;
        ElapsedTime et = new ElapsedTime();
        for(int i = 0; i < 5 && opModeIsActive(); i++) {
            float distX = (float)bot.backDistSensor.getDistance(DistanceUnit.CM);
            float distY = (float)bot.leftDistSensor.getDistance(DistanceUnit.CM);
            avgXDist += distX;
            avgYDist += distY;
            BetaLog.dd("Distance Sensor Loop", "%d:  X: %.1f cm Y: %.1f cm  %f millis", i + 1, distX, distY, et.milliseconds());
        }
        avgXDist /= 5;
        avgYDist /= 5;

        if(avgXDist > 800)
            avgXDist = robotXYTheta[0] - BOT_LENGTH / 2;

        if(avgYDist > 800)
            avgYDist = -robotXYTheta[1] - BOT_LENGTH / 2;

        BetaLog.dd("BACK IN LOADING AREA", "Heading Before resetOdometry: %.4f", robotXYTheta[2] * 180 / Math.PI);
        resetOdometry(avgXDist + BOT_LENGTH / 2, -avgYDist - BOT_LENGTH / 2);
        BetaLog.dd("BACK IN LOADING AREA","Current Heading: %.4f", robotXYTheta[2]*180.0/Math.PI );

        final float targetX2 = (skyStonePosition == SkyStonePosition.RIGHT?28:(skyStonePosition == SkyStonePosition.CENTER?48:68));

        //Move to x coordinate to intake 2nd skystone
//        if(skyStonePosition == SkyStonePosition.RIGHT) {
//            if (robotXYTheta[0] < targetX2) {
//                driveDirectionGyro(STD_DRIVE_SPEED, 0, 0, new Predicate() {
//                    @Override
//                    public boolean isTrue() {
//                        return robotXYTheta[0] >= targetX2;
//                    }
//                });
//            } else {
//                driveDirectionGyro(STD_DRIVE_SPEED, 180, 0, new Predicate() {
//                    @Override
//                    public boolean isTrue() {
//                        return robotXYTheta[0] <= targetX2;
//                    }
//                });
//            }
//        } else {
        driveToPositionGyro(FAST_DRIVE_SPEED, 20, targetX2, robotXYTheta[1], 0);
        //}
        turnToHeadingGyro(0, STANDARD_TOLERANCE, STANDARD_LATENCY);

        //resetOdometry(robotXYTheta[0], -(float)bot.leftDistSensor.getDistance(DistanceUnit.CM) - BOT_LENGTH / 2);

        driveDirectionGyro(STD_DRIVE_SPEED, -90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] < BLOCK_GRAB_Y;
            }
        });

        bot.setRightFlippoFingerClosed();
        sleep(500);
        bot.rightFlippo.setPosition(.8);

        driveDirectionGyro(FAST_DRIVE_SPEED, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > BLOCK_LINE_Y + 50;
            }
        });

        driveDirectionGyro(VERY_FAST_DRIVE_SPEED, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                // Second check should help with distance sensor issues
                return robotXYTheta[0] > FIELD_WIDTH * .6;
            }
        });

        bot.setRightFlippoFingerOpen();
        bot.setDragServoUp();

        driveToPositionGyro(FAST_DRIVE_SPEED, 10, robotXYTheta[0], -90, 0);
//        turnToHeadingGyro(-90, STANDARD_TOLERANCE, STANDARD_LATENCY, 150);
//        driveDirectionGyro(STD_DRIVE_SPEED, -90, -90, new Predicate() {
//            @Override
//            public boolean isTrue() {
//                return robotXYTheta[1] < -FOUNDATION_POSITION;
//            }
//        });
//
//        dragFoundationArc(RotationDirection.COUNTER_CLOCK, false);

        bot.setRightFlippoUp();
        bot.setLeftFlippoUp();
        bot.setRightFlippoFingerClosed();
        bot.setLeftFlippoFingerClosed();

        //Drive a short distance away from the back wall
//        driveDirectionGyro(FAST_DRIVE_SPEED, -175, 0, new Predicate() {
//            @Override
//            public boolean isTrue() {
//                return robotXYTheta[0] < FIELD_WIDTH - 54 * 2.54;
//            }
//        });

        //Drive into 2nd lane
//        driveDirectionGyro(FAST_DRIVE_SPEED, -90, 0, new Predicate() {
//            @Override
//            public boolean isTrue() {
//                //armControl.updateArm(ARM_FINAL_DEG, ARM_FIRST_EXTEND_CM);
//                return robotXYTheta[1] < -37 * 2.54;
//            }
//        });

        //Move to the bridge and park
//        driveDirectionGyro(FAST_DRIVE_SPEED, 180, 0, new Predicate() {
//            @Override
//            public boolean isTrue() {
//                //armControl.updateArm(ARM_FINAL_DEG, ARM_FIRST_EXTEND_CM);
//                return robotXYTheta[0] < FIELD_WIDTH / 2 + 5 * 2.54;
//            }
//        });
        driveToPositionGyro(FAST_DRIVE_SPEED, 20, FIELD_WIDTH / 2 + 5 * 2.54f, robotXYTheta[1], 0);
        VuforiaNavigator.setFlashTorchMode(false);
    }

    void deliverBlock(final float wallDist, boolean dropBlock) {
        //Drive to back wall
        driveDirectionGyro(VERY_FAST_DRIVE_SPEED, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                // Second check should help with distance sensor issues
                return bot.frontDistSensor.getDistance(DistanceUnit.CM) < wallDist && robotXYTheta[0] > FIELD_WIDTH * .65;
            }
        });

        resetOdometry(FIELD_WIDTH - (float)bot.frontDistSensor.getDistance(DistanceUnit.CM) - BOT_LENGTH / 2, robotXYTheta[1]);

        driveToPositionGyro(FAST_DRIVE_SPEED, 20, robotXYTheta[0], BLOCK_DROP_Y, 0);

        if(dropBlock) {
            bot.setRightFlippoFingerOpen();
            sleep(200);
        }
    }
}
