package org.firstinspires.ftc.teamcode.skybot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.skybot.SkyBot;

/**
 * Created by FTC Team 8397 on 11/25/2019.
 */
@Disabled
@Autonomous(name="Park Only", group="Yes")
public class AutoParkOnly extends SkyBotAutonomous {

    public SkyBot bot = new SkyBot(MechBot.MotorType.NeverestOrbital20, 31.5, 31.75, 31.75,
            BNO055Enhanced.AxesMap.YZX, BNO055Enhanced.AxesSign.PNN, 1);

    public AutoParkOnly() {super(AllianceColor.BLUE);}

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap, 0.0f, true);
        super.setBot(bot);
        waitForStart();
        resetOdometry(0, 0, 0);
        driveDirectionGyro(STD_DRIVE_SPEED, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotXYTheta[0] > 8 * 2.54f;
            }
        });
    }
}
