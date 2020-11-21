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
        /*
         * Create a BlobHelper object. It will obtain and manage new images from the image stream, and crop and
         * reduce them as requested; save images to a file if requested; and find blobs within images.
         */
        BlobHelper blobHelper = new BlobHelper(1280, 720, 0, 0, 1280, 720,
                4);

        /*
         * Use blobHelper to update the current image. If this is successfull, set the variable
         * 'success' to true.
         */
        boolean success = false;
        while (opModeIsActive()) {
            if (blobHelper.updateImage()) {
                success = true;
                break;
            }
        }

        if (success) {          //If blobHelper got an image, use it to analyze the ring stack

            /*
             * If 'saveImage' is true, save the image to file
             */
            if (saveImage) {
                blobHelper.saveRawImageFile(false);
            }

            /*
             * Use blobHelper to get a list of Blob objects from the current image, using the
             * HSV Range that corresponds to the color of the rings, and requiring some minimum
             * number of pixels in each blob (blobs with fewer pixels are ignored)
             */
            List<Blob> blobs = blobHelper.getBlobs(HSV_RANGE, new org.firstinspires.ftc.robotcore.external.Predicate<Blob>() {
                @Override
                public boolean test(Blob blob) {
                    return blob.getNumPts() > 150;
                }
            });


            if (blobs.size() == 0) {        //zero blobs means ZERO stack height
                return Rings.ZERO;
            } else {
                /*
                 * There should be only one blob in the list, but there could be more (e.g., from
                 * some objects outside of the field). Assume that the largest blob (i.e., the one
                 * with the greatest number of points) represents the ring stack.
                 */
                Blob biggestBlob = blobs.get(0);
                blobs.remove(0);
                while (blobs.size() > 0) {
                    if (blobs.get(0).getNumPts() > biggestBlob.getNumPts()) {
                        biggestBlob = blobs.get(0);
                    }
                    blobs.remove(0);
                }

                /*
                 * Obtain the ratio of the variance in the X direction (horizontal) to the variance
                 * in the Y direction (vertical) of the pixels in the blob. These are proportionate
                 * to the square of the length and height of the blob.
                 */
                float ratio = biggestBlob.getVarXX() / biggestBlob.getVarYY();
                
                if (ratio > 7) {
                    return Rings.ONE;
                } else {                       //smaller ratio --> blob closer to being square in shape
                    return Rings.FOUR;
                }
            }
        } else {            //If blobHelper didn't get an image, just take a guess about the stack
            return Rings.ONE;
        }

    }

    public void shoot() {
        bot.setKickerEngaged();
        sleep(500);
        bot.setKickerUnengaged();
    }

}
