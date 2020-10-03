package org.firstinspires.ftc.teamcode.newbot2bot.opmodes;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.cv.Blob;
import org.firstinspires.ftc.teamcode.cv.BlobHelper;
import org.firstinspires.ftc.teamcode.cv.HSV_Range;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
import org.firstinspires.ftc.teamcode.newbot2bot.NewBot2Bot;

import java.util.List;

public abstract class NB2BAuto extends MechBotAutonomous {
    public NewBot2Bot bot;
    public SkyStonePosition skyStonePosition = SkyStonePosition.RIGHT;
    public AllianceColor allianceColor;

    public enum SkyStonePosition {
        RIGHT, CENTER, LEFT
    }

    public final float X_START = 38 * 2.54f;
    public final float Y_START;
    public final float X_START2 = 92 * 2.54f;

    public final float BLOCK_LINE_Y;
    public final float BLOCK_GRAB_Y;
    public final float BLOCK_DROP_Y;

    // How far we move out from BLOCK_LINE_Y when driving under the bridge
    public final float LONG_DRIVE_OFFSET = 45;      //Changed from 40 because block hitting bridge on blue side.

    public final float FOUNDATION_POSITION = 108;
    public final float FOUNDATION_LENGTH = 88;
    public final float FOUNDATION_WIDTH = 47;
    public final float FOUNDATION_ARC_RADIUS = 24 * 2.54f;
    public final float FOUNDATION_DRAG_ANGLE = 30.0f * (float)Math.PI/180.0f;

    public final float FIELD_WIDTH = 12 * 12 * 2.54f;
    public final float BOT_LENGTH = 46;
    public final float STD_DRIVE_SPEED = 40;
    public final float FAST_DRIVE_SPEED = 80;
    public final float VERY_FAST_DRIVE_SPEED = 130;

    public final int SKYSTONE_PICTURE_LINE_Y = 160;
    public final int SKYSTONE_PICTURE_LINE_HEIGHT = 240;
    public final HSV_Range SKYSTONE_YELLOW_HSV_RANGE = new HSV_Range(10, 55, 0.25f, 1.0f, 0.45f, 1.0f);
    public final int SKYSTONE_MIN_BLOB_PXLS = 2000;

    public final static float INTAKE_POWER = .5f;

    public enum AllianceColor{
        RED, BLUE
    }

    public NB2BAuto(AllianceColor AC) {
        allianceColor = AC;
        // Negate Y if blue
        Y_START = (AC == AllianceColor.RED ? 1 : -1) * 25.4f;
        BLOCK_LINE_Y = (AC == AllianceColor.RED ? 1 : -1) * 129.54f;
        BLOCK_GRAB_Y = BLOCK_LINE_Y - (BOT_LENGTH / 2 + 7 + 2.5f) * (AC == AllianceColor.RED ? 1 : -1);
        BLOCK_DROP_Y = 104 * (AC == AllianceColor.RED ? 1 : -1);
    }

    public OpenGLMatrix[] targetLocations = new OpenGLMatrix[13];

    protected void setBot(NewBot2Bot newBot2Bot){
        bot = newBot2Bot;
        super.setBot(newBot2Bot);
        bot.setCapServoClosed();
    }

    public SkyStonePosition determinePositionBlobStatistics(SkyStonePosition fallBack) {
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
            while(blobs.size() > 1) {
                blobs.get(0).merge(blobs.get(1));
                blobs.remove(1);
            }
            if(blobs.size() == 0) {
                return fallBack;
            }

            Blob blob = blobs.get(0);

            if(blob.getVarXX() > 8500) {
                return SkyStonePosition.CENTER;
            } else if(blob.getAvgX() > 160) {
                return SkyStonePosition.RIGHT;
            } else {
                return SkyStonePosition.LEFT;
            }
        } else {
            telemetry.addData("No image", "");
            return fallBack;
        }
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

    void dragFoundationArc(RotationDirection direction, boolean dropBlock) {
        bot.setDragServoDown();
        sleep(500);
        float va = FAST_DRIVE_SPEED / FOUNDATION_ARC_RADIUS;
        if(direction == RotationDirection.CLOCK) va = -va;
        float vy = -FAST_DRIVE_SPEED * (float)Math.cos(FOUNDATION_DRAG_ANGLE);
        float vx = FAST_DRIVE_SPEED * (float)Math.sin(FOUNDATION_DRAG_ANGLE) * Math.signum(va);
        bot.setDriveSpeed(vx, vy, va);
        while(opModeIsActive()) {
            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            if(Math.signum(heading) * Math.signum(va) >= 0) {
                break;
            }
        }
        bot.setDrivePower(0,0,0);
        driveDirectionGyro(STD_DRIVE_SPEED, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] > FIELD_WIDTH - 17 * 2.54 - BOT_LENGTH / 2;
            }
        });
        float dist;
        float y = allianceColor == AllianceColor.BLUE ? -(dist = (float)bot.leftDistSensor.getDistance(DistanceUnit.CM)) - BOT_LENGTH / 2 : (dist = (float)bot.rightDistSensor.getDistance(DistanceUnit.CM)) + BOT_LENGTH / 2;
        if(dist > 800)
            y = robotXYTheta[1];

        BetaLog.d("POST DRAG Y: %.1f", y);
        resetOdometry(FIELD_WIDTH - 18 * 2.54f - BOT_LENGTH / 2, y);
        if(dropBlock) {
            if (allianceColor == AllianceColor.BLUE) {
                bot.setRightFlippoFingerOpen();
            } else {
                bot.setLeftFlippoFingerOpen();
            }
        }
        bot.setDragServoUp();
        sleep(400);
    }

    void dragFoundationArcPartial(RotationDirection direction, boolean dropBlock) {
        bot.setDragServoDown();
        sleep(500);
        float va = FAST_DRIVE_SPEED / FOUNDATION_ARC_RADIUS;
        if(direction == RotationDirection.CLOCK) va = -va;
        float vy = -FAST_DRIVE_SPEED * (float)Math.cos(FOUNDATION_DRAG_ANGLE);
        float vx = FAST_DRIVE_SPEED * (float)Math.sin(FOUNDATION_DRAG_ANGLE) * Math.signum(va);
        bot.setDriveSpeed(vx, vy, va);
        while(opModeIsActive()) {
            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            if(Math.signum(heading) * Math.signum(va) >= 0) {
                break;
            }
        }
        bot.setDrivePower(0,0,0);
        float dist;
        float y = allianceColor == AllianceColor.BLUE ? -(dist = (float)bot.leftDistSensor.getDistance(DistanceUnit.CM)) - BOT_LENGTH / 2 : (dist = (float)bot.rightDistSensor.getDistance(DistanceUnit.CM)) + BOT_LENGTH / 2;
        if(dist > 800)
            y = robotXYTheta[1];

        BetaLog.d("POST DRAG Y: %.1f", y);
        resetOdometry(robotXYTheta[0], y);
        if(dropBlock) {
            if (allianceColor == AllianceColor.BLUE) {
                bot.setRightFlippoFingerOpen();
            } else {
                bot.setLeftFlippoFingerOpen();
            }
        }
        bot.setDragServoUp();
        sleep(400);
    }
}
