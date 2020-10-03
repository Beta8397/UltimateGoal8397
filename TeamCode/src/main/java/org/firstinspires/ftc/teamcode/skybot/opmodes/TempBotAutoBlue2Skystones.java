package org.firstinspires.ftc.teamcode.skybot.opmodes;

import android.graphics.PointF;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.firstinspires.ftc.teamcode.skybot.SkyBot;
import org.firstinspires.ftc.teamcode.skybot.TempBot;

/**
 * Created by FTC Team 8397 on 10/29/2019.
 */
@Disabled
@Autonomous(name = "Temp Bot Blue 2 Skystones", group = "No")
public class TempBotAutoBlue2Skystones extends SkyBotAutonomous {
    public TempBot bot = new TempBot(MechBot.MotorType.NeverestOrbital20, 31.5, 31.75, 31.75,
            BNO055Enhanced.AxesMap.YZX, BNO055Enhanced.AxesSign.PNN, 1);

    final OpenGLMatrix landscapePhoneLocation = OpenGLMatrix.translation(-216f, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC
            , AxesOrder.YXY, AngleUnit.DEGREES, 90, 180, -9.6f));

    private final float IMU_LATENCY = 0.2f;

    public TempBotAutoBlue2Skystones() {super(AllianceColor.BLUE);}

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
        //armControl.init();
        skyStonePosition = determinePositionBlob(SkyStonePosition.LEFT);
        telemetry.addData("Sky Stone Position", "%s", skyStonePosition.toString());
        telemetry.update();

        CameraDevice.getInstance().setField("zoom", "0");

        resetOdometry(X_START, Y_START, 180);

        //Drive to block line
        driveDirectionGyro(FAST_DRIVE_SPEED, -90, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArmRotation(ARM_RAISE_DEG);
                return robotXYTheta[1] < BLOCK_LINE_Y / 2 - 5.5 * 2.54;
            }
        });
        //bot.armPitchServo.setPosition(0);

        // Move to skystone and intake
        if (skyStonePosition == SkyStonePosition.LEFT){
            intakeLeftBlock();
            //bot.armClawServo.setPosition(SkyBot.CLAW_SERVO_OPEN);
            driveDirectionGyro(FAST_DRIVE_SPEED, 90, 45, new Predicate() {
                @Override
                public boolean isTrue() {
                    //armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
                    return robotXYTheta[1] > BLOCK_LINE_Y + 15 * 2.54f;
                }
            });
            ElapsedTime et = new ElapsedTime();
            //bot.intakeLeft.setPower(-1);
            //while(opModeIsActive() && et.milliseconds() < 65) {
                //armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
            //}
            //bot.setIntakePower(0);
            turnToHeadingGyro(0, 3, IMU_LATENCY, 150, new Predicate() {
                @Override
                public boolean isTrue() {
                    //armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
                    return false;
                }
            });
        } else {
            intakeBlock(skyStonePosition);
            //bot.armClawServo.setPosition(SkyBot.CLAW_SERVO_OPEN);
            //Move away from blocks
            driveDirectionGyro(FAST_DRIVE_SPEED, 90, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    //armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
                    return robotXYTheta[1] > BLOCK_LINE_Y + 12 * 2.54f;
                }
            });
        }
        //bot.setIntakePower(0);
        bot.setDragServoUp();

        //Drive to build area
        // Slightly off to the side to avoid hitting the bridge and stones on the way back
        driveDirectionGyro(VERY_FAST_DRIVE_SPEED, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                if(robotXYTheta[0] < FIELD_WIDTH / 2 + 20) {
                    //armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
                    //bot.setIntakePower(INTAKE_POWER);
                } else {
                    //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                }
                // Second check should help with distance sensor issues
                return bot.frontDistSensor.getDistance(DistanceUnit.CM) < 100 && robotXYTheta[0] > FIELD_WIDTH * .65;
            }
        });
        //bot.setIntakePower(0);

        //Drop block here

        //Drive back to loading area
        driveDirectionGyro(VERY_FAST_DRIVE_SPEED, 180, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return (float)bot.backDistSensor.getDistance(DistanceUnit.CM) <= 70 && robotXYTheta[0] < FIELD_WIDTH * .35;
            }
        });

        turnToHeadingGyro(90, 3, IMU_LATENCY);

        sleep(100);

        //Update robotXYTheta using Vuforia to get an initial estimate
        ElapsedTime et = new ElapsedTime();
        boolean success = false;
        while(opModeIsActive() && et.milliseconds() < 2000) {
            OpenGLMatrix robotLocationTransform = VuforiaNavigator.getRobotLocation(8);
            if(robotLocationTransform == null) continue;
            float[] tempXYTheta = VuforiaNavigator.getX_Y_Theta_FromLocationTransform(robotLocationTransform);
            resetOdometry(tempXYTheta[0], tempXYTheta[1]);
            BetaLog.dd("VUFORIA", "X: %.2f, Y: %.2f, Heading: %.2f", robotXYTheta[0], robotXYTheta[1], robotXYTheta[2] * 180 / Math.PI);
            success = true;
            break;
        }
        telemetry.addData("Success: ", success);
        telemetry.update();

        // LEFT and CENTER adjusted to be slightly closer to the wall
        final float targetX = (skyStonePosition == SkyStonePosition.RIGHT?6:(skyStonePosition == SkyStonePosition.CENTER?15:23)) * 2.54f + BOT_LENGTH / 2;
        final boolean toWall = robotXYTheta[0] > targetX;

        //Align with skystone
        driveLineGyro(STD_DRIVE_SPEED, toWall ? 180 : 0, new PointF(0, -36 * 2.54f), 90, new PoseCallBack() {
            @Override
            public float[] getXYTheta(float[] xyTheta, float heading, MechBot mechBot) {
                float[] tempXYTheta = bot.updateOdometry(xyTheta, heading);
                OpenGLMatrix robotLocationTransform = VuforiaNavigator.getRobotLocation(8);
                if(robotLocationTransform == null) return tempXYTheta;
                tempXYTheta = VuforiaNavigator.getX_Y_Theta_FromLocationTransform(robotLocationTransform);
                tempXYTheta[2] = heading;
                return tempXYTheta;
            }
        }, new Predicate() {
            @Override
            public boolean isTrue() {
                return (toWall && robotXYTheta[0] <= targetX) || (!toWall && robotXYTheta[0] >= targetX);
            }
        });

//        driveDirectionGyro(180, 0, new DriveSpeedControl() {
//            float distance;
//            boolean nearWall = false;
//
//            @Override
//            public boolean isTrue() {
//                if(!nearWall) {
//                    if((distance = (float)bot.backDistSensor.getDistance(DistanceUnit.CM)) <= 6) {
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
//
//        resetOdometry(BOT_LENGTH / 2, robotXYTheta[1]);

        BetaLog.dd("BACK IN LOADING AREA","Current Heading: %.4f", robotXYTheta[2]*180.0/Math.PI );

        /* Distance Sensor Stuff
        float backDist = (float)bot.backDistSensor.getDistance(DistanceUnit.CM);
        float leftDist = (float)bot.leftDistSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance Sensors: ", "Back: %f  Left: %f", backDist, leftDist);
        telemetry.update();
        BetaLog.dd("DISTANCE SENSORS: ", "Back: %f  Left: %f", backDist, leftDist);
        resetOdometry(backDist + BOT_LENGTH / 2, -leftDist - BOT_LENGTH / 2);

        while(opModeIsActive()) {
            telemetry.addData("Old Measurement: ", "Back: %f  Left: %f", backDist, leftDist);
            telemetry.addData("New Measurement: ", "Back: %f  Left: %f", (float)bot.backDistSensor.getDistance(DistanceUnit.CM), (float)bot.leftDistSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }*/


        //turnToHeadingGyro(90,3, IMU_LATENCY);
        //sleep(400);

        //Update robotXYTheta using Vuforia
//        ElapsedTime et = new ElapsedTime();
//        boolean success = false;
//        while(opModeIsActive() && et.milliseconds() < 2000) {
//            OpenGLMatrix robotLocationTransform = VuforiaNavigator.getRobotLocation(9);
//            if(robotLocationTransform == null) continue;
//            float[] tempXYTheta = VuforiaNavigator.getX_Y_Theta_FromLocationTransform(robotLocationTransform);
//            resetOdometry(tempXYTheta[0], tempXYTheta[1]);
//            BetaLog.dd("VUFORIA", "X: %.2f, Y: %.2f, Heading: %.2f", robotXYTheta[0], robotXYTheta[1], robotXYTheta[2] * 180 / Math.PI);
//            success = true;
//            break;
//        }
//        telemetry.addData("Success: ", success);
//        telemetry.update();

        //BetaLog.dd("CHECKED FOR VUFORIA: ", "%s", success? "SUCCESS" : "FAILED");
        //BetaLog.dd("ABOUT TO TURN TO 0. ", "Current Heading = %.4f", robotXYTheta[2]);

        //turnToHeadingGyro(0, 3, IMU_LATENCY);

        //Move to x coordinate to intake 2nd skystone
//        if(robotXYTheta[0] < targetX) {
//            driveDirectionGyro(FAST_DRIVE_SPEED, 0, 0, new Predicate() {
//                @Override
//                public boolean isTrue() {
//                    return robotXYTheta[0] >= targetX;
//                }
//            });
//        } else {
//            driveDirectionGyro(FAST_DRIVE_SPEED, 180, 0, new Predicate() {
//                @Override
//                public boolean isTrue() {
//                    return robotXYTheta[0] <= targetX;
//                }
//            });
//        }

        //resetOdometry(robotXYTheta[0], -(float)bot.leftDistSensor.getDistance(DistanceUnit.CM) - BOT_LENGTH / 2);

        turnToHeadingGyro(0, 3, IMU_LATENCY);

        //Move into line of skystones
        driveDirectionGyro(STD_DRIVE_SPEED, -90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] < BLOCK_LINE_Y;
            }
        });

        //Intake block
        //bot.setIntakePower(INTAKE_POWER);
        driveDirectionGyro(FAST_DRIVE_SPEED, 180, 0, new Predicate() {
            float x0 = robotXYTheta[0];

            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[0] < x0 - 5 * 2.54f;
            }
        });
        ElapsedTime intakeET = new ElapsedTime();
        while (intakeET.milliseconds() < 900 && opModeIsActive()) {
            //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
        }
        driveDirectionGyro(FAST_DRIVE_SPEED, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] > BLOCK_LINE_Y + 12 * 2.54f;
            }
        });
        //bot.setIntakePower(0);
        bot.setDragServoUp();

        //Drive to back wall
        driveDirectionGyro(VERY_FAST_DRIVE_SPEED, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                if(robotXYTheta[0] < FIELD_WIDTH / 2 + 20) {
                    //armControl.updateArm(ARM_LOWER_DEG, ARM_FIRST_EXTEND_CM);
                    //bot.setIntakePower(INTAKE_POWER);
                } else {
                    //armControl.updateArmExtensionIfRotationAbove(ARM_RETRACT_CM, ARM_RAISE_DEG - 5);
                    //armControl.updateArmRotation(ARM_RAISE_DEG);
                }
                // Second check should help with distance sensor issues
                return bot.frontDistSensor.getDistance(DistanceUnit.CM) < 60 && robotXYTheta[0] > FIELD_WIDTH * .75;
            }
        });
        //bot.setIntakePower(0);

        deliverBlock();
        dragFoundationArc(RotationDirection.COUNTER_CLOCK);

        //Drive a short distance away from the back wall
        driveDirectionGyro(FAST_DRIVE_SPEED, -165, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                if(robotXYTheta[0] > FIELD_WIDTH - 27 * 2.54) {
                    //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                } else {
                    //armControl.updateArm(ARM_FINAL_DEG, ARM_FIRST_EXTEND_CM);
                }
                return robotXYTheta[0] < FIELD_WIDTH - 54 * 2.54;
            }
        });

        //Drive into 2nd lane
//        driveDirectionGyro(FAST_DRIVE_SPEED, -90, 0, new Predicate() {
//            @Override
//            public boolean isTrue() {
//                //armControl.updateArm(ARM_FINAL_DEG, ARM_FIRST_EXTEND_CM);
//                return robotXYTheta[1] < -37 * 2.54;
//            }
//        });

        //Move to the bridge and park
        driveDirectionGyro(FAST_DRIVE_SPEED, 180, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_FINAL_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[0] < FIELD_WIDTH / 2 + 5 * 2.54;
            }
        });
        //armControl.stopArmPower();
        VuforiaNavigator.setFlashTorchMode(false);
    }

    void intakeLeftBlock() {
        turnToHeadingGyro(90, 3, IMU_LATENCY, 150, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return false;
            }
        });
        driveDirectionGyro(FAST_DRIVE_SPEED, 0, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[0] > 53f * 2.54f;
            }
        });
        driveDirectionGyro(FAST_DRIVE_SPEED, -90, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] < BLOCK_LINE_Y + 14 * 2.54f;
            }
        });
        bot.setDriveSpeed(0,-6, -Math.PI / 6);
        //bot.setIntakePower(INTAKE_POWER);
        while(opModeIsActive()) {
            //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
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
        turnToHeadingGyro(0, 3, IMU_LATENCY, 150, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArmExtension(ARM_FIRST_EXTEND_CM);
                //armControl.updateArmRotation(ARM_RAISE_DEG);
                return false;
            }
        }, RotationDirection.COUNTER_CLOCK);
        if (skyStonePosition == SkyStonePosition.CENTER){
            driveDirectionGyro(STD_DRIVE_SPEED, 0, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                    return robotXYTheta[0] >  X_START + 12.5 * 2.54f;
                }
            });
        } else {
            driveDirectionGyro(STD_DRIVE_SPEED, 0, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                    return robotXYTheta[0] >  X_START + 4.5 * 2.54f;
                }
            });
        }
        driveDirectionGyro(FAST_DRIVE_SPEED, -90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] < BLOCK_LINE_Y;
            }
        });
        //bot.setIntakePower(INTAKE_POWER);
        driveDirectionGyro(FAST_DRIVE_SPEED, 180, 0, new Predicate() {
            float x0 = robotXYTheta[0];

            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[0] < x0 - 4 * 2.54f;
            }
        });
        ElapsedTime intakeET = new ElapsedTime();
        while (intakeET.milliseconds() < 900 && opModeIsActive()) {
            //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
        }
    }

    void deliverBlock() {
        // 0.5 is a test value. Change later
        //bot.armPitchServo.setPosition(0.3);
        resetOdometry(FIELD_WIDTH - (float)bot.frontDistSensor.getDistance(DistanceUnit.CM) - BOT_LENGTH / 2, robotXYTheta[1]);
        turnToHeadingGyro(-90, 3, IMU_LATENCY, 150, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return false;
            }
        });
        driveDirectionGyro((FAST_DRIVE_SPEED + STD_DRIVE_SPEED) / 2, -90, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] < -FOUNDATION_POSITION;
            }
        });
        //bot.armClawServo.setPosition(SkyBot.CLAW_SERVO_CLOSED);
    }

    void dragFoundationArc(RotationDirection direction) {
        bot.setDragServoDown();
        ElapsedTime et = new ElapsedTime();
        while(opModeIsActive() && et.milliseconds() < 500) {
            //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
        }
        // Move a bit back so the foundation lands more squarely in the building site
        driveDirectionGyro(FAST_DRIVE_SPEED, 90, -90, new Predicate() {
            float y0 = robotXYTheta[1];
            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] - y0 > 2 * 2.54;
            }
        });
        float speed = (FAST_DRIVE_SPEED + STD_DRIVE_SPEED) / 2;

        float va = speed / FOUNDATION_ARC_RADIUS;
        if(direction == RotationDirection.CLOCK) va = -va;
        float vy = -speed * (float)Math.cos(FOUNDATION_DRAG_ANGLE);
        float vx = speed * (float)Math.sin(FOUNDATION_DRAG_ANGLE) * Math.signum(va);
        bot.setDriveSpeed(vx, vy, va);
        while(opModeIsActive()) {
            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            if(Math.signum(heading) * Math.signum(va) >= 0) {
                break;
            }
            //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
        }
        bot.setDrivePower(0,0,0);
        driveDirectionGyro(FAST_DRIVE_SPEED, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[0] > FIELD_WIDTH - 21 * 2.54 - BOT_LENGTH / 2;
            }
        });
        // Slow down when the robot gets close to the wall
        driveDirectionGyro(STD_DRIVE_SPEED, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[0] > FIELD_WIDTH - 17.9 * 2.54 - BOT_LENGTH / 2;
            }
        });
        resetOdometry(FIELD_WIDTH - 18 * 2.54f - BOT_LENGTH / 2, robotXYTheta[1]);
        bot.setDragServoUp();
        et.reset();
        while(opModeIsActive() && et.milliseconds() < 500) {
            //armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
        }
    }
}
