package org.firstinspires.ftc.teamcode.skybot.opmodes;

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

/**
 * Created by FTC Team 8397 on 10/29/2019.
 */
@Disabled
@Autonomous(name = "Red Skystone Only", group = "Maybe")
public class AutoRedSkystoneOnly extends SkyBotAutonomous {
    public SkyBot bot = new SkyBot(MechBot.MotorType.NeverestOrbital20, 31.5, 31.75, 31.75,
            BNO055Enhanced.AxesMap.YZX, BNO055Enhanced.AxesSign.PNN, 1);

    final OpenGLMatrix landscapePhoneLocation = OpenGLMatrix.translation(-216f, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC
            , AxesOrder.YXY, AngleUnit.DEGREES, 90, 180, -9.6f));

    public AutoRedSkystoneOnly() {super(AllianceColor.RED);}

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
        armControl.init();
        skyStonePosition = determinePositionBlob(SkyStonePosition.RIGHT);
        telemetry.addData("Sky Stone Position", "%s", skyStonePosition.toString());
        telemetry.update();

        resetOdometry(X_START, Y_START, 0);

        driveDirectionGyro(STD_DRIVE_SPEED, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArmRotation(ARM_RAISE_DEG);
                return robotXYTheta[1] > BLOCK_LINE_Y / 2 + 5.5 * 2.54f;
            }
        });
        bot.armPitchServo.setPosition(0);
        // Move to skystone and intake
        if (skyStonePosition == SkyStonePosition.RIGHT){
            intakeRightBlock();
            bot.armClawServo.setPosition(SkyBot.CLAW_SERVO_OPEN);
            driveDirectionGyro(STD_DRIVE_SPEED, -90, -45, new Predicate() {
                @Override
                public boolean isTrue() {
                    armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
                    return robotXYTheta[1] < BLOCK_LINE_Y - 15 * 2.54f;
                }
            });
            ElapsedTime et = new ElapsedTime();
            bot.intakeRight.setPower(-1);
            while(opModeIsActive() && et.milliseconds() < 65) {
                armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
            }
            bot.setIntakePower(0);
            turnToHeadingGyro(0, 3, 0.3f, 150, new Predicate() {
                @Override
                public boolean isTrue() {
                    armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
                    return false;
                }
            });
        } else {
            intakeBlock(skyStonePosition);
            bot.armClawServo.setPosition(SkyBot.CLAW_SERVO_OPEN);
            //Move away from blocks
            driveDirectionGyro(STD_DRIVE_SPEED, -90, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
                    return robotXYTheta[1] < BLOCK_LINE_Y - 12 * 2.54f;
                }
            });
        }
        bot.setIntakePower(0);
        bot.setDragServoUp();

        //Drive to bridge
        driveDirectionGyro(FAST_DRIVE_SPEED, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
                bot.setIntakePower(INTAKE_POWER);
                return robotXYTheta[0] > FIELD_WIDTH / 2 + BOT_LENGTH / 2 + 10.0;
            }
        });
        bot.setIntakePower(0);

        driveDirectionGyro(STD_DRIVE_SPEED, 180, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[0] < FIELD_WIDTH / 2 + BOT_LENGTH / 2 - 10;
            }
        });

        //Drive into 2nd lane
        driveDirectionGyro(STD_DRIVE_SPEED, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_FINAL_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] > 37 * 2.54;
            }
        });

        armControl.stopArmPower();
        VuforiaNavigator.setFlashTorchMode(false);
    }

    void intakeRightBlock() {
        turnToHeadingGyro(-90, 3, 0.3f, 150, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return false;
            }
        });
        driveDirectionGyro(STD_DRIVE_SPEED, 0, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[0] > 53f * 2.54f;
            }
        });
        driveDirectionGyro(STD_DRIVE_SPEED, 90, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] > BLOCK_LINE_Y - 14.5 * 2.54f;
            }
        });
        bot.setDriveSpeed(0,-6, Math.PI / 6);
        bot.setIntakePower(INTAKE_POWER);
        while(opModeIsActive()) {
            armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
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
                    armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                    return robotXYTheta[0] >  X_START + 12.5 * 2.54f;
                }
            });
        } else {
            driveDirectionGyro(STD_DRIVE_SPEED, 0, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                    return robotXYTheta[0] >  X_START + 4.5 * 2.54f;
                }
            });
        }
        driveDirectionGyro(STD_DRIVE_SPEED, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] > BLOCK_LINE_Y;
            }
        });
        bot.setIntakePower(INTAKE_POWER);
        driveDirectionGyro(STD_DRIVE_SPEED, 180, 0, new Predicate() {
            float x0 = robotXYTheta[0];
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[0] < x0 - 5 * 2.54f;
            }
        });
        ElapsedTime intakeET = new ElapsedTime();
        while (intakeET.milliseconds() < 900 && opModeIsActive()){
            armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
        }
    }

    void deliverBlock() {
        // 0.5 is a test value. Change later
        bot.armPitchServo.setPosition(0.5);
        resetOdometry(FIELD_WIDTH - (float)bot.frontDistSensor.getDistance(DistanceUnit.CM) - BOT_LENGTH / 2, robotXYTheta[1]);
        turnToHeadingGyro(90, 3, 0.3f, 150, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return false;
            }
        });
        driveDirectionGyro(STD_DRIVE_SPEED, 90, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] > FOUNDATION_POSITION;
            }
        });
        bot.armClawServo.setPosition(SkyBot.CLAW_SERVO_CLOSED);
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
