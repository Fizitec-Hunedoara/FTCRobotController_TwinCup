package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Parametri.dslider;
import static org.firstinspires.ftc.teamcode.Parametri.islider;
import static org.firstinspires.ftc.teamcode.Parametri.pslider;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Autonomous
public class autonom_mortis_brawlball extends LinearOpMode {
    public DcMotorEx motorBR,motorBL,motorFL,motorFR;
    long lastTime;
    double sm = 1, lb = 1, rb = 1, sliderSlow = 1, intakePos = 0, rotitorPoz = 3, incheieturaPoz = 1;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    boolean stop = false, lastx = false, lasty = false, intaked = true, gherutaL = false, gherutaR = false, lastBumperL, lastBumperR, setSetpoint = true, automatizare = false, started_left = false, started_right = false;
    double servoPos = 0, servoPos2 = 0.85, pidResult = 0;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(pslider,islider,dslider);
    FunctiiDeAtunonom c = new FunctiiDeAtunonom();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        /* Liniile astea de cod fac ca motoarele sa corespunda cu cele din configuratie, cu numele dintre ghilimele.*/
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Back-Left
        /*Liniile astea de cod fac ca motoarele sa aiba puterea inversata fata de cum erau initial,
        sunt fol++osite pentru a face robotul sa mearga in fata dand putere pozitiva la toate cele 4 motoare. */
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);

        /*Liniile astea de cod fac ca motoarele sa poata frana de tot atunci cand ii dai sa franeze*/
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        /*Liniile astea de cod fac ca robotul sa mearga cu ajutorul encoderelor(maresc precizia)*/
        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        while(!isStarted()){

        }
        lastTime = System.currentTimeMillis();
        while(lastTime + 2400 > System.currentTimeMillis() && !isStopRequested()){
            motorBL.setPower(0.2);
            motorBR.setPower(0.2);
            motorFL.setPower(0.2);
            motorFR.setPower(0.2);
        }
    }

}
