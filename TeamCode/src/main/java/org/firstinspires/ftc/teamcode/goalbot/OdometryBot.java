package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.odometry.Encoder;

import java.util.List;

public class OdometryBot extends GoalBot{
    Encoder rightEncoder = null;
    Encoder leftEncoder = null;
    Encoder horizEncoder = null;
    public static final float TICKS_PER_INCH = 1865f; //was 1875.4
    public static final float HORIZ_TICKS_PER_RAD = 4056f; //was 4008
    public static final float ROTATION_COEFF = 26767f; // was 26695
    public static final float FRAC_LEFT = 0.503f; // was 0.507
    public static final float FRAC_RIGHT = 0.497f; // was 0.493
    public int rightTicks, leftTicks, horizTicks;

    public OdometryBot() {
        super();
    }

    @Override
    public boolean init(HardwareMap hwMap) {

                    List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
           }
        boolean result = super.init(hwMap);
        rightEncoder = new Encoder(intakeBack, true);
        leftEncoder = new Encoder(armMotor, true);
        horizEncoder = new Encoder(intakeFront, true);
        return result;
    }

    @Override
    public Pose updateOdometry(){
        int rightCurrentTicks = rightEncoder.getCurrentPosition();
        int leftCurrentTicks = leftEncoder.getCurrentPosition();
        int horizCurrentTicks = horizEncoder.getCurrentPosition();
        int rightNewTicks = rightCurrentTicks - rightTicks;
        int leftNewTicks = leftCurrentTicks - leftTicks;
        int horizNewTicks = horizCurrentTicks - horizTicks;
        rightTicks = rightCurrentTicks;
        leftTicks = leftCurrentTicks;
        horizTicks = horizCurrentTicks;

        float dyR = (FRAC_LEFT * rightNewTicks + FRAC_RIGHT * leftNewTicks) / TICKS_PER_INCH;
        float dT = (rightNewTicks - leftNewTicks) / (ROTATION_COEFF);
        float horizEncoderAngleTicks = HORIZ_TICKS_PER_RAD * dT;
        float dxR = (horizNewTicks - horizEncoderAngleTicks) / TICKS_PER_INCH;
        float avgHeading = (float) AngleUtils.normalizeRadians(pose.theta + 0.5 * dT);

        float dX = dxR * (float)Math.sin(avgHeading) + dyR * (float)Math.cos(avgHeading);
        float dY = -dxR * (float)Math.cos(avgHeading) + dyR * (float)Math.sin(avgHeading);

        /*
         * Update the Pose object with the new values for X, Y, and Heading
         */
        float heading = (float) AngleUtils.normalizeRadians(pose.theta + dT);
        pose = new Pose(pose.x + dX, pose.y + dY, heading);

        /*
         * Return the updated Pose object
         */
        return pose;

    }
    @Override
    public void updateTicks(){
        leftTicks = leftEncoder.getCurrentPosition();
        rightTicks = rightEncoder.getCurrentPosition();
        horizTicks = horizEncoder.getCurrentPosition();

    }

}
