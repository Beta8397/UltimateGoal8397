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
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.skybot.SkyBot;
import org.firstinspires.ftc.teamcode.skybot.opmodes.SkyBotAutonomous;

/**
 * Created by FTC Team 8397 on 10/29/2019.
 */
@Disabled
@Autonomous(name = "Blue Drag N Park", group = "No")
public class AutoBlueDragNPark extends SkyBotAutonomous {
    public SkyBot bot = new SkyBot(MechBot.MotorType.NeverestOrbital20, 31.5, 31.75, 31.75,
            BNO055Enhanced.AxesMap.YZX, BNO055Enhanced.AxesSign.PNN, 1);

    public AutoBlueDragNPark() {super(AllianceColor.BLUE);}

    @Override
    public void runLoggingOpmode() {

        bot.init(hardwareMap, -90f, true);
        super.setBot(bot);

        waitForStart();

        resetOdometry(FIELD_WIDTH - (25 * 2.54f + BOT_LENGTH/2), Y_START, -90);

        driveDirectionGyro(STD_DRIVE_SPEED, -90, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] < -FOUNDATION_POSITION / 2;
            }
        });

        bot.setDragServoUp();

        //Drive to back wall
        driveDirectionGyro(STD_DRIVE_SPEED, 0, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] > FIELD_WIDTH - 60 - BOT_LENGTH / 2;
            }
        });

        driveDirectionGyro(STD_DRIVE_SPEED, -90, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] < -FOUNDATION_POSITION;
            }
        });

        dragFoundationArc(RotationDirection.COUNTER_CLOCK);

        //Drive a short distance away from the back wall
        driveDirectionGyro(STD_DRIVE_SPEED, 180, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] < FIELD_WIDTH - 54 * 2.54;
            }
        });

        //Drive into 2nd lane
        driveDirectionGyro(STD_DRIVE_SPEED, -90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] < -37 * 2.54;
            }
        });

        //Move to the bridge and park
        driveDirectionGyro(STD_DRIVE_SPEED, 180, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] < FIELD_WIDTH / 2 + 5 * 2.54;
            }
        });
    }
}
