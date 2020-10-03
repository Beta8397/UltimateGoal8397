package org.firstinspires.ftc.teamcode.cv;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;

import java.util.List;
import java.util.concurrent.BlockingQueue;

/**
 * Created by FTC Team 8397 on 9/21/2019.
 */
@Disabled
@Autonomous(name="SkystoneBlobTest", group="Test")
public class SkystoneBlobTest extends LoggingLinearOpMode {

    private boolean SAVE_IMAGE = false;

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        VuforiaNavigator.activate(null, null);

        waitForStart();
        while(opModeIsActive()) {
            checkForBlack();
            telemetry.update();
        }

    }

    public Blob checkForBlack(){
        BlobHelper blobHelper = new BlobHelper(1280, 720, 0, 0, 1280, 720, 4);
        if(SAVE_IMAGE) blobHelper.saveRawImageFile(false);
        blobHelper.updateImage();
        HSV_Range black_range = new HSV_Range(0, 360, 0.0f, 1f, 0.0f, 0.4f);
        HSV_Range yellow_range = new HSV_Range(30, 60, 0.8f, 1f, 0.5f, 1f);
        List<Blob> blobs = blobHelper.getBlobs(yellow_range, new org.firstinspires.ftc.robotcore.external.Predicate<Blob>() {
            @Override
            public boolean test(Blob blob) {
                if(blob.getNumPts() >= 1000) {
                    return true;
                }
                return false;
            }
        });

        if(blobs == null || blobs.size() == 0) {
            telemetry.addData("No Yellow Blobs Found", "");
            return null;
        }

        while (blobs.size() > 1) {
            blobs.get(0).merge(blobs.get(1));
            blobs.remove(1);
        }

        Point blobCenter = blobs.get(0).getRectCenter();
        int blobWidth = blobs.get(0).getRectWidth(), blobHeight = blobs.get(0).getRectHeight();
        telemetry.addData("Yellow Blob ", "x: %d y: %d w: %d h: %d", blobCenter.x, blobCenter.y, blobWidth, blobHeight);
        BetaLog.dd("Yellow Blob ", "x: %d y: %d w: %d h: %d", blobCenter.x, blobCenter.y, blobWidth, blobHeight);
        try {
            blobHelper.setSubRange(0, (blobCenter.y - blobHeight / 2) * 4, 1280, blobHeight * 4, 4);
            blobHelper.getBlobs(black_range, new org.firstinspires.ftc.robotcore.external.Predicate<Blob>() {
                @Override
                public boolean test(Blob blob) {
                    if (blob.getNumPts() >= 1000) {
                        return true;
                    }
                    return false;
                }
            });

            if (blobs == null || blobs.size() == 0) {
                telemetry.addData("No Black Blob Found", "");
                return null;
            }

            while (blobs.size() > 1) {
                if (blobs.get(1).getNumPts() > blobs.get(0).getNumPts()) blobs.remove(0);
                else blobs.remove(1);
            }

            telemetry.addData("Black Blob", "x: %.0f y: %.0f w: %.0f h: %.0f", blobs.get(0).getAvgX(), blobs.get(0).getAvgY(), blobs.get(0).getWidth(), blobs.get(0).getLength());
            return blobs.get(0);
        } catch (Exception e){
            telemetry.addData("Failed attempt at black","");
            return null;
        }
    }
}
