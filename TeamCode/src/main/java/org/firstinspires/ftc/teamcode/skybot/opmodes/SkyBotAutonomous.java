package org.firstinspires.ftc.teamcode.skybot.opmodes;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.cv.Blob;
import org.firstinspires.ftc.teamcode.cv.BlobHelper;
import org.firstinspires.ftc.teamcode.cv.HSV_Range;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
import org.firstinspires.ftc.teamcode.skybot.SkyBot;

import java.util.List;

/**
 * Created by FTC Team 8397 on 11/2/2019.
 */
public abstract class SkyBotAutonomous extends MechBotAutonomous {
    public SkyBot bot;
    public SkyStonePosition skyStonePosition = SkyStonePosition.RIGHT;

    public enum SkyStonePosition {
        RIGHT, CENTER, LEFT
    }

    public final float X_START = 39 * 2.54f;
    public final float Y_START;
    public final float X_START2 = 92 * 2.54f;

    public final float BLOCK_LINE_Y;

    public final float FOUNDATION_POSITION = (53 - 9) * 2.54f;
    public final float FOUNDATION_LENGTH = 34.5f * 2.54f;
    public final float FOUNDATION_WIDTH = 18.375f * 2.54f;
    public final float FOUNDATION_ARC_RADIUS = 24 * 2.54f;
    public final float FOUNDATION_DRAG_ANGLE = 30.0f * (float)Math.PI/180.0f;

    public final float FIELD_WIDTH = 12 * 12 * 2.54f;
    public final float BOT_LENGTH = 18 * 2.54f;
    public final float STD_DRIVE_SPEED = 40;
    public final float FAST_DRIVE_SPEED = 80;
    public final float VERY_FAST_DRIVE_SPEED = 130;

    public final int SKYSTONE_PICTURE_LINE_Y = 428;
    public final int SKYSTONE_PICTURE_LINE_HEIGHT = 280;
    public final HSV_Range SKYSTONE_YELLOW_HSV_RANGE = new HSV_Range(10, 55, 0.5f, 1.0f, 0.45f, 1.0f);
    public final int SKYSTONE_MIN_BLOB_PXLS = 2000;

    public final float ARM_RAISE_DEG = 30;
    public final float ARM_FIRST_EXTEND_CM = 33;
    public final float ARM_LOWER_DEG = 0;
    public final float ARM_RETRACT_CM = -3;
    public final float ARM_GRAB_BLOCK_DEG = 20;
    public final float ARM_HIGH_RAISE_DEG = 40;
    public final float ARM_FOUNDATION_EXTEND_CM = 40;
    public final float ARM_FINAL_DEG = 15;

    //value might need to be changed
    public final static float INTAKE_POWER = 1f;

    protected ArmControl armControl;

    public enum AllianceColor{
        RED, BLUE
    }

    public OpenGLMatrix[] targetLocations = new OpenGLMatrix[13];

    // Maybe this was a mistake
    public SkyBotAutonomous(AllianceColor AC) {
        // Negate Y if blue
        Y_START = (AC == AllianceColor.RED ? 1 : -1) * 10 * 2.54f;
        BLOCK_LINE_Y = (AC == AllianceColor.RED ? 1 : -1) * 51 * 2.54f;
    }

    protected void setBot(SkyBot skyBot){
        bot = skyBot;
        super.setBot(skyBot);
        bot.setGripServoGripped();
        armControl = new ArmControl();
    }

    public SkyStonePosition determinePositionBlob(SkyStonePosition fallBack) {
        BlobHelper blobHelper = new BlobHelper(1280, 720, 0, SKYSTONE_PICTURE_LINE_Y, 1280, SKYSTONE_PICTURE_LINE_HEIGHT, 4);
        ElapsedTime et = new ElapsedTime();
        boolean imageGot = false;
        while(opModeIsActive() && et.milliseconds() < 500) {
            if(imageGot = blobHelper.updateImage())
                break;
        }
        if(imageGot) {
            telemetry.addData("Image File Saved: ", blobHelper.saveRawImageFile(false));

            List<Blob> blobs = blobHelper.getBlobs(SKYSTONE_YELLOW_HSV_RANGE, new org.firstinspires.ftc.robotcore.external.Predicate<Blob>() {
                @Override
                public boolean test(Blob blob) {
                    return blob.getNumPts() > SKYSTONE_MIN_BLOB_PXLS;
                }
            });
            // Assume the most likely reason we'd have 3+ blobs is if it's in the center
            if(blobs.size() >= 2) {
                return SkyStonePosition.CENTER;
            }
            if(blobs.size() == 1) {
                if(blobs.get(0).getAvgX() > 160) {
                    return SkyStonePosition.LEFT;
                } else {
                    return SkyStonePosition.RIGHT;
                }
            }
        } else {
            telemetry.addData("No image", "");
        }
        return fallBack;
    }

    public SkyStonePosition determinePosition(SkyStonePosition fallBack){
        ElapsedTime ET = new ElapsedTime();
        OpenGLMatrix pose = null;
        while(opModeIsActive() && ET.milliseconds() <= 1000){
            pose = VuforiaNavigator.getRobotLocation(0);
            if(pose != null)
                break;
        }

        if(pose != null){
            float[] xyTheta = VuforiaNavigator.getX_Y_Theta_FromLocationTransform(pose);
            if(xyTheta[0] < -10)
                return SkyStonePosition.RIGHT;
            else if (xyTheta[0] > 10)
                return SkyStonePosition.LEFT;
            else
                return SkyStonePosition.CENTER;
        } else {
            return fallBack;
        }
    }


    protected void setTargetLocations(AllianceColor AC){
        for(int i = 0; i < targetLocations.length; i++) {
            targetLocations[i] = Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES, 90, 0, 0);
        }
        targetLocations[8] = OpenGLMatrix.translation(0, -36 * 25.4f, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 90, 0));
        targetLocations[9] = OpenGLMatrix.translation(36 * 25.4f, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 0, 0));
    }

    @Deprecated
    public void dumpBlock() {}

    void dragFoundationArc(RotationDirection direction) {
        bot.setDragServoDown();
        ElapsedTime et = new ElapsedTime();
        while(opModeIsActive() && et.milliseconds() < 500) {
            armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
        }
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
            armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
        }
        bot.setDrivePower(0,0,0);
        driveDirectionGyro(STD_DRIVE_SPEED, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[0] > FIELD_WIDTH - 17 * 2.54 - BOT_LENGTH / 2;
            }
        });
        resetOdometry(FIELD_WIDTH - 18 * 2.54f - BOT_LENGTH / 2, robotXYTheta[1]);
        bot.setDragServoUp();
        et.reset();
        while(opModeIsActive() && et.milliseconds() < 500) {
            armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
        }
    }

    void dragFoundationLine(AllianceColor color) {
        float targetHeading = color == AllianceColor.BLUE ? -90 : 90;
        bot.setDragServoDown();
        sleep(500);
        driveDirectionGyro((FAST_DRIVE_SPEED + STD_DRIVE_SPEED) / 2, -targetHeading, targetHeading, new Predicate() {
            @Override
            public boolean isTrue() {
                return Math.abs(robotXYTheta[1]) <= BOT_LENGTH / 2 - 18;
            }
        });
        bot.setDragServoUp();
        // Long pause to allow time for our partner to do things before we park
        sleep(9500);
        resetOdometry(robotXYTheta[0], (color==AllianceColor.BLUE?-1:1) * BOT_LENGTH / 2, targetHeading);
        //Drive away from back wall to clear foundation
        driveDirectionGyro(STD_DRIVE_SPEED, 180, targetHeading, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] < FIELD_WIDTH / 2 + 20 * 2.54;
            }
        });
        // Drive away from the alliance color wall to be in line with the foundation end
        driveDirectionGyro(STD_DRIVE_SPEED, targetHeading, targetHeading, new Predicate() {
            float y0 = Math.abs(robotXYTheta[1]);
            @Override
            public boolean isTrue() {
                return Math.abs(robotXYTheta[1]) > y0 + 10 * 2.54;
            }
        });

        // Bump the foundation into the back wall
        driveDirectionGyro(STD_DRIVE_SPEED, 0, targetHeading, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] > FIELD_WIDTH - 43 * 2.54;
            }
        });
        resetOdometry(FIELD_WIDTH - FOUNDATION_LENGTH - BOT_LENGTH / 2, robotXYTheta[1]);
        bot.setDragServoDown();
    }

    public class ArmControl {

        private int initArmRotationTicks = bot.armRotation.getCurrentPosition();
        private int initArmExtensionTicks = bot.armExtension.getCurrentPosition();

        public void init() {
            initArmRotationTicks = bot.armRotation.getCurrentPosition();
            initArmExtensionTicks = bot.armExtension.getCurrentPosition();
        }

        public void stopArmPower() {
            bot.armExtension.setPower(0);
            bot.armRotation.setPower(0);
        }

        private float armRotDegreesFromTicks(int ticks){
            return (ticks - initArmRotationTicks) * SkyBot.ARM_DEGREES_PER_TICK;
        }

        private int armRotTicksFromDegrees(float degrees){
            return Math.round(initArmRotationTicks + degrees / SkyBot.ARM_DEGREES_PER_TICK);
        }

        private float armExtCMfromTicks(int ticks){
            return (ticks - initArmExtensionTicks) * SkyBot.ARM_CM_PER_TICK;
        }

        private int armExtTicksFromCM(float cm){
            return Math.round(initArmExtensionTicks + cm / SkyBot.ARM_CM_PER_TICK);
        }

        public void updateArm(float degrees, float cm) {
            updateArmRotation(degrees);
            updateArmExtension(cm);
        }

        public void updateArmExtension(float cm) {
            int targetArmExtTicks = armExtTicksFromCM(cm);

            int armExtTicks = bot.armExtension.getCurrentPosition();
            float pArm = -0.0005f * (armExtTicks - targetArmExtTicks);
            pArm = Range.clip(pArm, -0.6f, 0.6f);
            bot.armExtension.setPower(pArm);
        }

        public void updateArmRotation(float degrees) {
            int targetArmRotTicks = armRotTicksFromDegrees(degrees);

            int armRotTicks = bot.armRotation.getCurrentPosition();
            float pArm = -0.003f * (armRotTicks - targetArmRotTicks);
            pArm = Range.clip(pArm, -0.6f, 0.6f);
            bot.armRotation.setPower(pArm);

        }

        public void updateArmExtensionIfRotationAbove(float cm, float minDegrees) {
            if(bot.armRotation.getCurrentPosition() <= armRotTicksFromDegrees(minDegrees)) {
                updateArmExtension(cm);
            } else {
                bot.armExtension.setPower(0);
            }
        }

    }
}
