package org.firstinspires.ftc.teamcode.newbot2bot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.newbot2bot.NewBot2Bot;

/**
 * Created by FTC Team 8397 on 11/25/2019.
 */
@Autonomous(name="NB2B Park Only", group="Yes")
public class AutoParkOnly extends NB2BAuto {

    public NewBot2Bot bot = new NewBot2Bot(MechBot.MotorType.NeverestOrbital20, 32.75, 31.75, 31.75,
            BNO055Enhanced.AxesMap.XZY, BNO055Enhanced.AxesSign.PNP, 1);

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
