package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class autonomh extends LinearOpMode {
    FunctiiDeAtunonom c = new FunctiiDeAtunonom();
    @Override
    public void runOpMode() throws InterruptedException {
        c.initSisteme(hardwareMap);

    }
}
