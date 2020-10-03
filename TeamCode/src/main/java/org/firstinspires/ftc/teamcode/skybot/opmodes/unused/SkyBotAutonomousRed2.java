package org.firstinspires.ftc.teamcode.skybot.opmodes.unused;
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
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.skybot.SkyBot;
import org.firstinspires.ftc.teamcode.skybot.opmodes.SkyBotAutonomous;

/**
 * Created by FTC Team 8397 on 10/29/2019.
 */
@Disabled
@Autonomous(name = "SkyBot Autonomous Red 2", group = "Maybe")
public class SkyBotAutonomousRed2 extends SkyBotAutonomous {
    public SkyBot bot = new SkyBot(MechBot.MotorType.NeverestOrbital20, 31.5, 31.75, 31.75,
            BNO055Enhanced.AxesMap.YZX, BNO055Enhanced.AxesSign.PNN, 1);

    final OpenGLMatrix landscapePhoneLocation = OpenGLMatrix.translation(-216f, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC
            , AxesOrder.YXY, AngleUnit.DEGREES, 90, 180, -9.6f));

    public SkyBotAutonomousRed2() {super(AllianceColor.RED);}

    @Override
    public void runLoggingOpmode() {

        bot.init(hardwareMap, 0.0f, true);
        super.setBot(bot);

        //Vuforia Stuff
        setTargetLocations(AllianceColor.RED);

        VuforiaNavigator.activate("Skystone", targetLocations, landscapePhoneLocation, VuforiaLocalizer.CameraDirection.BACK, null);
        CameraDevice.getInstance().setField("zoom", "22");
        VuforiaNavigator.setFlashTorchMode(true);

        waitForStart();

        resetOdometry(X_START2, Y_START, 0);

        /* ************************************************************
        Move slightly forward so we don't drag against the wall when moving down
        *************************************************************** */
        driveDirectionGyro(30, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > Y_START + 2 * 2.54f;
            }
        });

        driveDirectionGyro(30, 180, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] < X_START;
            }
        });


        skyStonePosition = determinePosition(SkyStonePosition.RIGHT);

        driveDirectionGyro(30, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > BLOCK_LINE_Y - 13 * 2.54f;
            }
        });
        if (skyStonePosition == SkyStonePosition.LEFT){
            driveDirectionGyro(30, 0, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    return robotXYTheta[0] >  X_START + 4 * 2.54f;
                }
            });
        } else if (skyStonePosition == SkyStonePosition.CENTER){
            driveDirectionGyro(30, 0, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    return robotXYTheta[0] >  X_START + 12 * 2.54f;
                }
            });
        } else {
            driveDirectionGyro(30, 180, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    return robotXYTheta[0] <  X_START - 2.5f * 2.54f;
                }
            });
        }
        driveDirectionGyro(30, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > BLOCK_LINE_Y;
            }
        });

        bot.setIntakePower(INTAKE_POWER);
        driveDirectionGyro(30, 180, 0, new Predicate() {
            float x0 = robotXYTheta[0];
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] < x0 - 8 * 2.54f;
            }
        });
        ElapsedTime intakeET = new ElapsedTime();
        while (intakeET.milliseconds() < 1500 && opModeIsActive()){
            continue;
        }
        bot.setIntakePower(0);


        driveDirectionGyro(30, -90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] < BLOCK_LINE_Y - 15 * 2.54f;
            }
        });

        driveDirectionGyro(60, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                //return robotXYTheta[0] >  125 * 2.54f;
                return bot.frontDistSensor.getDistance(DistanceUnit.CM) < 60;
            }
        });
        deliverBlock();

        VuforiaNavigator.setFlashTorchMode(false);

    }
    void deliverBlock() {
        // Uncomment later
        //resetOdometry(FIELD_WIDTH - (float)bot.backDistSensor.getDistance(DistanceUnit.CM) - BOT_LENGTH / 2, robotXYTheta[1]);
        turnToHeadingGyro(90, 2, 0.2f);
        driveDirectionGyro(30, 90, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > FOUNDATION_POSITION;
            }
        });
        dumpBlock();
        dragFoundation();
        driveDirectionGyro(30, 180, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] < FIELD_WIDTH / 2;
            }
        });
    }

    void dragFoundation() {

    }
}
