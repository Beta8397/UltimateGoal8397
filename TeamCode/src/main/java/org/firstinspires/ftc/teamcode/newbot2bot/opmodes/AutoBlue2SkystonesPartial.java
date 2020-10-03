package org.firstinspires.ftc.teamcode.newbot2bot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="NB2B Blue 2 Skystones Partial", group="No")
public class AutoBlue2SkystonesPartial extends NB2BAuto {
    public NewBot2Bot bot = new NewBot2Bot(MechBot.MotorType.NeverestOrbital20, 32.75, 31.75, 31.75,
            BNO055Enhanced.AxesMap.XZY, BNO055Enhanced.AxesSign.PNP, 1);

    final OpenGLMatrix landscapePhoneLocation = OpenGLMatrix.translation(-216f, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC
            , AxesOrder.YXY, AngleUnit.DEGREES, 90, 180, -9.6f));

    public AutoBlue2SkystonesPartial() {super(AllianceColor.BLUE);}

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
                return robotXYTheta[1] < BLOCK_LINE_Y / 2.2;
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
                return robotXYTheta[1] > BLOCK_LINE_Y + LONG_DRIVE_OFFSET;
            }
        });

        deliverBlock(80, true);

        driveDirectionGyro(FAST_DRIVE_SPEED, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > BLOCK_GRAB_Y + 10;
            }
        });


        // Move back to normal grabbing X and back away slightly
       // driveToPositionGyro(FAST_DRIVE_SPEED, 20f, FIELD_WIDTH - 80 - BOT_LENGTH / 2, BLOCK_GRAB_Y + 10, 0);


        turnToHeadingGyro(-90, STANDARD_TOLERANCE, STANDARD_LATENCY, 150);
        driveToPositionGyro(FAST_DRIVE_SPEED, STD_DRIVE_SPEED, robotXYTheta[0], -FOUNDATION_POSITION, -90);
        // TODO Touch sensor

        bot.setRightFlippoUp();
        bot.setLeftFlippoUp();
        bot.setRightFlippoFingerClosed();
        bot.setLeftFlippoFingerClosed();

        dragFoundationArcPartial(RotationDirection.COUNTER_CLOCK, false);

        turnToHeadingGyro(0, STANDARD_TOLERANCE, STANDARD_LATENCY);
        float leftDist = (float)bot.leftDistSensor.getDistance(DistanceUnit.CM);
        if(leftDist > 25 && leftDist < 125) {
            resetOdometry(robotXYTheta[0], -leftDist - BOT_LENGTH / 2);
        }
        // Adjust y coordinate to avoid hitting second bot
        driveToPositionGyro(FAST_DRIVE_SPEED, STD_DRIVE_SPEED, robotXYTheta[0] - 10, BLOCK_LINE_Y + 50, 0);
        bot.setDragServoDown();

        // Drive back to loading area
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
        // # of bad values we've gotten from the distance sensors
        int badValXCount = 0;
        int badValYCount = 0;
        ElapsedTime et = new ElapsedTime();
        for(int i = 0; i < 5 && opModeIsActive(); i++) {
            float distX = (float)bot.backDistSensor.getDistance(DistanceUnit.CM);
            float distY = (float)bot.leftDistSensor.getDistance(DistanceUnit.CM);
            if(distX < 800)
                avgXDist += distX;
            else
                badValXCount++;
            if(distY < 800)
                avgYDist += distY;
            else
                badValYCount++;
            BetaLog.dd("Distance Sensor Loop", "%d:  X: %.1f cm Y: %.1f cm  %f millis", i + 1, distX, distY, et.milliseconds());
        }

        BetaLog.dd("BACK IN LOADING AREA", "Y Distance Total: %.4f   Bad Y Values: %d", avgYDist, badValYCount);
        if (badValXCount < 5) avgXDist /= 5 - badValXCount;
        if (badValYCount < 5) avgYDist /= 5 - badValYCount;

        BetaLog.dd("BACK IN LOADING AREA", "Avg Y Dist before bad distance sensor correction: %.4f", avgYDist);

        if(avgXDist > 800 || badValXCount > 2)
            avgXDist = robotXYTheta[0] - BOT_LENGTH / 2;

        if(avgYDist > 800 || badValYCount > 2)
            avgYDist = -robotXYTheta[1] - BOT_LENGTH / 2;

        BetaLog.dd("BACK IN LOADING AREA", "Avg Y Dist: %.4f", avgYDist);
        BetaLog.dd("BACK IN LOADING AREA", "Heading Before resetOdometry: %.4f", robotXYTheta[2] * 180 / Math.PI);
        BetaLog.dd("BACK IN LOADING AREA", "Odometry being reset to: %.4f, %.4f", avgXDist + BOT_LENGTH / 2, -avgYDist - BOT_LENGTH / 2);
        resetOdometry(avgXDist + BOT_LENGTH / 2, -avgYDist - BOT_LENGTH / 2);
        BetaLog.dd("BACK IN LOADING AREA","Current Heading: %.4f", robotXYTheta[2]*180.0/Math.PI );

        final float targetX2 = (skyStonePosition == SkyStonePosition.RIGHT?28:(skyStonePosition == SkyStonePosition.CENTER?48:68));

        BetaLog.dd("BACK IN LOADING AREA", "Odometry before driveToPositionGyro: %.4f, %.4f", robotXYTheta[0], robotXYTheta[1]);
        driveToPositionGyro(FAST_DRIVE_SPEED, 20, targetX2, robotXYTheta[1], 0);
        BetaLog.dd("BACK IN LOADING AREA", "Odometry after driveToPositionGyro: %.4f, %.4f", robotXYTheta[0], robotXYTheta[1]);
        turnToHeadingGyro(0, STANDARD_TOLERANCE, STANDARD_LATENCY);


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
                return robotXYTheta[1] > BLOCK_LINE_Y + LONG_DRIVE_OFFSET;
            }
        });

        driveToPositionGyro(VERY_FAST_DRIVE_SPEED, STD_DRIVE_SPEED, FIELD_WIDTH / 2 + BOT_LENGTH + 20, BLOCK_LINE_Y + LONG_DRIVE_OFFSET + 8, 00);

        turnToHeadingGyro(90, STANDARD_TOLERANCE, STANDARD_LATENCY);

        driveToPositionGyro(VERY_FAST_DRIVE_SPEED, STD_DRIVE_SPEED, FIELD_WIDTH - 17 * 2.54f - BOT_LENGTH / 2, BLOCK_LINE_Y + LONG_DRIVE_OFFSET + 8, 90);

        bot.setRightFlippoDown();
        //sleep(200);
        bot.setRightFlippoFingerOpen();
        sleep(200);

        driveToPositionGyro(FAST_DRIVE_SPEED, 20, robotXYTheta[0], -90, 90);

        bot.setRightFlippoUp();
        bot.setLeftFlippoUp();
        bot.setRightFlippoFingerClosed();
        bot.setLeftFlippoFingerClosed();

        driveToPositionGyro(FAST_DRIVE_SPEED, 20, FIELD_WIDTH / 2 + 5 * 2.54f, robotXYTheta[1], 90);
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
            bot.setRightFlippoDown();
            //sleep(200);
            bot.setRightFlippoFingerOpen();
            sleep(200);
        }
    }
}
