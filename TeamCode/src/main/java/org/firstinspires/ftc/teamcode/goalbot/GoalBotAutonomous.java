package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Predicate;
import org.firstinspires.ftc.teamcode.cv.Blob;
import org.firstinspires.ftc.teamcode.cv.BlobHelper;
import org.firstinspires.ftc.teamcode.cv.HSV_Range;
import org.firstinspires.ftc.teamcode.mecbot.MecBotAutonomous;

import java.util.List;

public abstract class GoalBotAutonomous extends MecBotAutonomous {
    public enum Rings {
        ZERO, ONE, FOUR
    }

    GoalBot bot;

    HSV_Range HSV_RANGE = new HSV_Range(20,35,0.4f,1,0.4f,1);

    public void setBot(GoalBot bot) {
        this.bot = bot;
        super.setBot(bot);

    }

    public Rings getRings(boolean saveImage) {
        //TODO: CODE THE RING DETECTION
        BlobHelper blobHelper = new BlobHelper(1280, 720, 0, 0, 1280, 720,
                4);
        boolean success = false;
        while (opModeIsActive()) {
            if (blobHelper.updateImage()) {
                success = true;
                break;
            }
        }

        if (success) {
            if (saveImage) {
                blobHelper.saveRawImageFile(false);
            }
            List<Blob> blobs = blobHelper.getBlobs(HSV_RANGE, new org.firstinspires.ftc.robotcore.external.Predicate<Blob>() {
                @Override
                public boolean test(Blob blob) {
                    return blob.getNumPts() > 150;
                }
            });
            if (blobs.size() == 0) {
                return Rings.ZERO;
            } else {
                Blob biggestBlob = blobs.get(0);
                blobs.remove(0);
                while (blobs.size() > 0) {
                    if (blobs.get(0).getNumPts() > biggestBlob.getNumPts()) {
                        biggestBlob = blobs.get(0);
                    }
                    blobs.remove(0);
                }
                float ratio = biggestBlob.getVarXX() / biggestBlob.getVarYY();
                if (ratio > 7) {
                    return Rings.ONE;
                } else {
                    return Rings.FOUR;
                }
            }
        } else {
            return Rings.ONE;
        }

    }

    public void shoot() {
        bot.setKickerEngaged();
        sleep(500);
        bot.setKickerUnengaged();
    }

}
