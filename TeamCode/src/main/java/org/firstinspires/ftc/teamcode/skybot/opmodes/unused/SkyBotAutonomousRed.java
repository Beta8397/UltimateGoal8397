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
@Autonomous(name = "SkyBot Autonomous Red", group = "Maybe")
public class SkyBotAutonomousRed extends SkyBotAutonomous {
    public SkyBot bot = new SkyBot(MechBot.MotorType.NeverestOrbital20, 31.5, 31.75, 31.75,
            BNO055Enhanced.AxesMap.YZX, BNO055Enhanced.AxesSign.PNN, 1);

    final OpenGLMatrix landscapePhoneLocation = OpenGLMatrix.translation(-216f, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC
            , AxesOrder.YXY, AngleUnit.DEGREES, 90, 180, -9.6f));

    public SkyBotAutonomousRed() {super(AllianceColor.RED);}

    @Override
    public void runLoggingOpmode() {

        bot.init(hardwareMap, 0.0f, true);
        super.setBot(bot);

        //Vuforia Stuff
        setTargetLocations(AllianceColor.RED);

        VuforiaNavigator.activate("Skystone", targetLocations, landscapePhoneLocation, VuforiaLocalizer.CameraDirection.BACK, null);
        CameraDevice.getInstance().setField("zoom", "21");
        VuforiaNavigator.setFlashTorchMode(true);

        waitForStart();
        skyStonePosition = determinePosition(SkyStonePosition.RIGHT);
        telemetry.addData("Sky Stone Position", "%s", skyStonePosition.toString());
        telemetry.update();

        resetOdometry(X_START, Y_START, 0);

        driveDirectionGyro(STD_DRIVE_SPEED, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > BLOCK_LINE_Y / 2 + 5.5 * 2.54f;
            }
        });
        // Move to skystone and intake
        if (skyStonePosition == SkyStonePosition.RIGHT){
            intakeRightBlock();
            driveDirectionGyro(STD_DRIVE_SPEED, -90, -45, new Predicate() {
                @Override
                public boolean isTrue() {
                    return robotXYTheta[1] < BLOCK_LINE_Y - 15 * 2.54f;
                }
            });
            turnToHeadingGyro(0, 2, 0.3f, 150);
        } else {
            intakeBlock(skyStonePosition);
            //Move away from blocks
            driveDirectionGyro(STD_DRIVE_SPEED, -90, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    return robotXYTheta[1] < BLOCK_LINE_Y - 12 * 2.54f;
                }
            });
        }
        bot.setIntakePower(0);
        bot.setDragServoUp();

        //Drive to back wall
        driveDirectionGyro(60, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return bot.frontDistSensor.getDistance(DistanceUnit.CM) < 60;
            }
        });

        deliverBlock();
        dragFoundation();
        resetOdometry(robotXYTheta[0], 9 * 2.54f, 90);
        //Drive away from back wall to clear foundation
        driveDirectionGyro(STD_DRIVE_SPEED, 180, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] < FIELD_WIDTH / 2 + 20 * 2.54;
            }
        });
        // Drive away from the red wall to be in line with the foundation end
        driveDirectionGyro(STD_DRIVE_SPEED, 90, 90, new Predicate() {
            float y0 = robotXYTheta[1];
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > y0 + 10 * 2.54;
            }
        });

        // Bump the foundation into the back wall
        driveDirectionGyro(STD_DRIVE_SPEED, 0, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] > FIELD_WIDTH - 43 * 2.54;
            }
        });
        resetOdometry(FIELD_WIDTH - 44 * 2.54f, robotXYTheta[1]);

        //Drive away from the back wall until there is enough room to rotate
        driveDirectionGyro(STD_DRIVE_SPEED, 180, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] < FIELD_WIDTH - 54 * 2.54;
            }
        });

        //Drive into 2nd lane
        driveDirectionGyro(STD_DRIVE_SPEED, 90, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > 32 * 2.54;
            }
        });

        //Move to the bridge and park
        driveDirectionGyro(STD_DRIVE_SPEED, 180, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] < FIELD_WIDTH / 2 + 3 * 2.54;
            }
        });

        VuforiaNavigator.setFlashTorchMode(false);

    }

    void intakeRightBlock() {
        turnToHeadingGyro(-90, 2, 0.3f, 150);
        driveDirectionGyro(STD_DRIVE_SPEED, 0, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] > 52f * 2.54f;
            }
        });
        driveDirectionGyro(STD_DRIVE_SPEED, 90, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > BLOCK_LINE_Y - 13 * 2.54f;
            }
        });
        bot.setDriveSpeed(0,-6, Math.PI / 6);
        bot.setIntakePower(INTAKE_POWER);
        while(opModeIsActive()) {
            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            if(heading > -Math.PI / 4) {
                break;
            }
        }
        bot.setDriveSpeed(0,0,0);
    }

    // For CENTER and LEFT
    void intakeBlock(SkyStonePosition skyStonePosition) {
        if (skyStonePosition == SkyStonePosition.CENTER){
            driveDirectionGyro(STD_DRIVE_SPEED, 0, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    return robotXYTheta[0] >  X_START + 12.5 * 2.54f;
                }
            });
        } else {
            driveDirectionGyro(STD_DRIVE_SPEED, 0, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    return robotXYTheta[0] >  X_START + 4.5 * 2.54f;
                }
            });
        }
        driveDirectionGyro(STD_DRIVE_SPEED, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > BLOCK_LINE_Y;
            }
        });
        bot.setIntakePower(INTAKE_POWER);
        driveDirectionGyro(STD_DRIVE_SPEED, 180, 0, new Predicate() {
            float x0 = robotXYTheta[0];
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] < x0 - 8 * 2.54f;
            }
        });
        ElapsedTime intakeET = new ElapsedTime();
        while (intakeET.milliseconds() < 900 && opModeIsActive()){
            continue;
        }
    }

    void deliverBlock() {
        resetOdometry(FIELD_WIDTH - (float)bot.frontDistSensor.getDistance(DistanceUnit.CM) - BOT_LENGTH / 2, robotXYTheta[1]);
        turnToHeadingGyro(90, 2, 0.3f, 150);
        driveDirectionGyro(STD_DRIVE_SPEED, 90, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > FOUNDATION_POSITION;
            }
        });
        dumpBlock();
    }

    void dragFoundation() {
        bot.setDragServoDown();
        sleep(500);
        driveDirectionGyro(STD_DRIVE_SPEED, -90, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] <= 5 + BOT_LENGTH / 2;
            }
        });
        bot.setDragServoUp();
        sleep(500);
    }
}
