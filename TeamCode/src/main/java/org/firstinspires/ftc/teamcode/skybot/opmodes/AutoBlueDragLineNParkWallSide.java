package org.firstinspires.ftc.teamcode.skybot.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.skybot.SkyBot;

@Disabled
@Autonomous(name="Blue Drag Line N Park Wall", group="No")
// Park in the first parking space
public class AutoBlueDragLineNParkWallSide extends SkyBotAutonomous {
    public SkyBot bot = new SkyBot(MechBot.MotorType.NeverestOrbital20, 31.5, 31.75, 31.75,
            BNO055Enhanced.AxesMap.YZX, BNO055Enhanced.AxesSign.PNN, 1);

    public AutoBlueDragLineNParkWallSide() {super(AllianceColor.BLUE);}

    @Override
    public void runLoggingOpmode() {
        bot.init(hardwareMap, -90f, true);
        super.setBot(bot);

        waitForStart();
        bot.setDragServoUp();

        resetOdometry(FIELD_WIDTH - (25 * 2.54f + BOT_LENGTH/2), Y_START, -90);

        //Drive to foundation
        driveDirectionGyro(STD_DRIVE_SPEED, -90, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] < -FOUNDATION_POSITION / 2;
            }
        });

        //Drive to back wall
        driveDirectionGyro(STD_DRIVE_SPEED, 0, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] > FIELD_WIDTH - 40 - BOT_LENGTH / 2;
            }
        });

        driveDirectionGyro(STD_DRIVE_SPEED, -90, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] < -FOUNDATION_POSITION;
            }
        });

        dragFoundationLine(AllianceColor.BLUE);

        //Drive a short distance away from the back wall
        driveDirectionGyro(STD_DRIVE_SPEED, 180, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] < FIELD_WIDTH - 54 * 2.54;
            }
        });


        //Drive into 1st lane
        driveDirectionGyro(STD_DRIVE_SPEED, 90, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > -BOT_LENGTH / 2 ;
            }
        });

        //Move to the bridge and park
        driveDirectionGyro(STD_DRIVE_SPEED, 180, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] < FIELD_WIDTH / 2 + 5 * 2.54;
            }
        });
    }
}
