package org.firstinspires.ftc.teamcode.skybot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.skybot.SkyBot;

/**
 * Created by FTC Team 8397 on 10/29/2019.
 */
@Disabled
@Autonomous(name = "Red Drag N Park", group = "Maybe")
public class AutoRedDragNPark extends SkyBotAutonomous {
    public SkyBot bot = new SkyBot(MechBot.MotorType.NeverestOrbital20, 31.5, 31.75, 31.75,
            BNO055Enhanced.AxesMap.YZX, BNO055Enhanced.AxesSign.PNN, 1);

    public AutoRedDragNPark() {super(AllianceColor.RED);}

    @Override
    public void runLoggingOpmode() {

        bot.init(hardwareMap, 90f, true);
        super.setBot(bot);

        waitForStart();

        resetOdometry(FIELD_WIDTH - (25 * 2.54f + BOT_LENGTH/2), Y_START, 90);

        bot.setDragServoUp();

        driveDirectionGyro(STD_DRIVE_SPEED, 90, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > FOUNDATION_POSITION / 2;
            }
        });

        //Drive to back wall
        driveDirectionGyro(STD_DRIVE_SPEED, 0, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] > FIELD_WIDTH - 40 - BOT_LENGTH / 2;
            }
        });

        driveDirectionGyro(STD_DRIVE_SPEED, 90, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[1] > FOUNDATION_POSITION;
            }
        });

        dragFoundationArc(RotationDirection.CLOCK);

        //Drive a short distance away from the back wall
        driveDirectionGyro(STD_DRIVE_SPEED, 180, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                if(robotXYTheta[0] > FIELD_WIDTH - 27 * 2.54) {
                    armControl.updateArm(ARM_RAISE_DEG, ARM_FIRST_EXTEND_CM);
                } else {
                    armControl.updateArm(ARM_FINAL_DEG, ARM_FIRST_EXTEND_CM);
                }
                return robotXYTheta[0] < FIELD_WIDTH - 54 * 2.54;
            }
        });

        //Drive into 2nd lane
        driveDirectionGyro(STD_DRIVE_SPEED, 90, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_FINAL_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[1] > 37 * 2.54;
            }
        });

        //Move to the bridge and park
        driveDirectionGyro(STD_DRIVE_SPEED, 180, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                armControl.updateArm(ARM_FINAL_DEG, ARM_FIRST_EXTEND_CM);
                return robotXYTheta[0] < FIELD_WIDTH / 2 + 5 * 2.54;
            }
        });
        armControl.stopArmPower();
    }
}
