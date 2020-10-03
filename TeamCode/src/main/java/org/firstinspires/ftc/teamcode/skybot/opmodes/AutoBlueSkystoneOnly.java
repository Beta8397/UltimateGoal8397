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
@Autonomous(name = "Blue Skystone Only", group = "No")
public class AutoBlueSkystoneOnly extends SkyBotAutonomous {
    public SkyBot bot = new SkyBot(MechBot.MotorType.NeverestOrbital20, 31.5, 31.75, 31.75,
            BNO055Enhanced.AxesMap.YZX, BNO055Enhanced.AxesSign.PNN, 1);

    final OpenGLMatrix landscapePhoneLocation = OpenGLMatrix.translation(-216f, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC
            , AxesOrder.YXY, AngleUnit.DEGREES, 90, 180, -9.6f));

    public AutoBlueSkystoneOnly() {super(AllianceColor.BLUE);}

    @Override
    public void runLoggingOpmode() {

        bot.init(hardwareMap, 180.0f, true);
        super.setBot(bot);

        //Vuforia Stuff
        setTargetLocations(AllianceColor.BLUE);

        VuforiaNavigator.activate("Skystone", targetLocations, landscapePhoneLocation, VuforiaLocalizer.CameraDirection.BACK, null);
        CameraDevice.getInstance().setField("zoom", "21");
        VuforiaNavigator.setFlashTorchMode(true);

        waitForStart();
        armControl.init();
        skyStonePosition = determinePositionBlob(SkyStonePosition.LEFT);
        telemetry.addData("Sky Stone Position", "%s", skyStonePosition.toString());
        telemetry.update();

        resetOdometry(X_START, Y_START, 180);

        driveDirectionGyro(STD_DRIVE_SPEED, -90, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArmRotation(ARM_RAISE_DEG);
                return robotXYTheta[1] < BLOCK_LINE_Y / 2 - 5.5 * 2.54;
            }
        });

        bot.armPitchServo.setPosition(0);
        // Move to skystone and intake
        if (skyStonePosition == SkyStonePosition.LEFT){
            intakeLeftBlock();
            bot.armClawServo.setPosition(SkyBot.CLAW_SERVO_OPEN);
            driveDirectionGyro(STD_DRIVE_SPEED, 90, 45, new Predicate() {
                @Override
                public boolean isTrue() {
                    armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
                    return robotXYTheta[1] > BLOCK_LINE_Y + 15 * 2.54f;
                }
            });
            ElapsedTime et = new ElapsedTime();
            bot.intakeLeft.setPower(-1);
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
            driveDirectionGyro(STD_DRIVE_SPEED, 90, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
                    return robotXYTheta[1] > BLOCK_LINE_Y + 12 * 2.54f;
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
                return robotXYTheta[0] < FIELD_WIDTH / 2 + BOT_LENGTH / 2;
            }
        });

           //Drive into 2nd lane
        driveDirectionGyro(STD_DRIVE_SPEED, -90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_FINAL_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] < -37 * 2.54;
            }
        });

        armControl.stopArmPower();
        VuforiaNavigator.setFlashTorchMode(false);
    }

    void intakeLeftBlock() {
        turnToHeadingGyro(90, 3, 0.3f, 150, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return false;
            }
        });
        driveDirectionGyro(STD_DRIVE_SPEED, 0, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[0] > 53f * 2.54f;
            }
        });
        driveDirectionGyro(STD_DRIVE_SPEED, -90, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] < BLOCK_LINE_Y + 14 * 2.54f;
            }
        });
        bot.setDriveSpeed(0,-6, -Math.PI / 6);
        bot.setIntakePower(INTAKE_POWER);
        while(opModeIsActive()) {
            armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            if(heading < Math.PI / 4) {
                break;
            }
        }
        bot.setDriveSpeed(0,0,0);
    }

    // For CENTER and RIGHT
    void intakeBlock(SkyStonePosition skyStonePosition) {
        turnToHeadingGyro(0, 3, 0.3f, 150, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArmExtension(ARM_FIRST_EXTEND_CM);
                armControl.updateArmRotation(ARM_RAISE_DEG);
                return false;
            }
        }, RotationDirection.COUNTER_CLOCK);
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
        driveDirectionGyro(STD_DRIVE_SPEED, -90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] < BLOCK_LINE_Y;
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
        while (intakeET.milliseconds() < 900 && opModeIsActive()) {
            armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
        }
    }

    void deliverBlock() {
        // 0.5 is a test value. Change later
        bot.armPitchServo.setPosition(0.5);
        resetOdometry(FIELD_WIDTH - (float)bot.frontDistSensor.getDistance(DistanceUnit.CM) - BOT_LENGTH / 2, robotXYTheta[1]);
        turnToHeadingGyro(-90, 3, 0.3f, 150, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArmExtensionIfRotationAbove(ARM_RETRACT_CM, ARM_RAISE_DEG - 5);
                armControl.updateArmRotation(ARM_RAISE_DEG);
                return false;
            }
        });
        driveDirectionGyro(STD_DRIVE_SPEED, -90, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_GRAB_BLOCK_DEG, ARM_RETRACT_CM);
                return robotXYTheta[1] < -FOUNDATION_POSITION;
            }
        });
        armControl.stopArmPower();
        bot.armClawServo.setPosition(SkyBot.CLAW_SERVO_CLOSED);
    }

    // Overridden so we don't add arm control to our other classes
    void dragFoundationArc(RotationDirection direction) {
        bot.setDragServoDown();
        sleep(500);
        float va = STD_DRIVE_SPEED / FOUNDATION_ARC_RADIUS;
        if(direction == RotationDirection.CLOCK) va = -va;
        float vy = -STD_DRIVE_SPEED * (float)Math.cos(FOUNDATION_DRAG_ANGLE);
        float vx = STD_DRIVE_SPEED * (float)Math.sin(FOUNDATION_DRAG_ANGLE) * Math.signum(va);
        bot.setDriveSpeed(vx, vy, va);
        while(opModeIsActive()) {
            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            if(Math.signum(heading) * Math.signum(va) >= 0) {
                break;
            }
            armControl.updateArmRotation(ARM_HIGH_RAISE_DEG);
            armControl.updateArmExtensionIfRotationAbove(ARM_FOUNDATION_EXTEND_CM, ARM_HIGH_RAISE_DEG - 5);
        }

        bot.setDrivePower(0,0,0);
        driveDirectionGyro(STD_DRIVE_SPEED, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_FINAL_DEG, ARM_FOUNDATION_EXTEND_CM);
                return robotXYTheta[0] > FIELD_WIDTH - 17 * 2.54 - BOT_LENGTH / 2;
            }
        });
        bot.armClawServo.setPosition(SkyBot.CLAW_SERVO_OPEN);
        resetOdometry(FIELD_WIDTH - 18 * 2.54f - BOT_LENGTH / 2, robotXYTheta[1]);
        bot.setDragServoUp();
        ElapsedTime et = new ElapsedTime();
        while(et.milliseconds() < 500) {
            armControl.updateArm(ARM_FINAL_DEG, ARM_FOUNDATION_EXTEND_CM);
        }
    }

    void dragFoundation() {
        bot.setDragServoDown();
        sleep(500);
        driveDirectionGyro((FAST_DRIVE_SPEED + STD_DRIVE_SPEED) / 2, 90, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] >= -5 - BOT_LENGTH / 2;
            }
        });
        bot.setDragServoUp();
        sleep(500);
    }
}
