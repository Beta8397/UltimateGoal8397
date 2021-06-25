package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.mecbot.MecBotAutonomous;
@Autonomous (name = "TurnFast", group = "test")
public class TurnFast extends MecBotAutonomous {

    private MecBot bot = new MecBot(MecBot.MotorType.NeverestOrbital20, 13.25f, 13, 4.0f, 40.9f, 1, BNO055Enhanced.AxesMap.XZY,
            BNO055Enhanced.AxesSign.NPP);

    @Override
    public void runLoggingOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        super.setBot(bot);
        bot.setPose(0,0,0);
        waitForStart();
        turnToHeadingPD(90,2,12,300,90);
    }
}
