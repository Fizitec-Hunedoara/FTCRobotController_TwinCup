 /* La inceputul programului, exista import-urile. Ele se fac de obicei automat asa ca nu te ingrijora de ele numai daca dau eroare(Nu au dat niciodata lol).
De asemenea, daca ceva da eroare in cod si nu stii de ce, verifica mai intai daca este importata chestia sau nu.
 */
package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.Parametri.dslider;
import static org.firstinspires.ftc.teamcode.Parametri.islider;
import static org.firstinspires.ftc.teamcode.Parametri.pslider;
import static java.lang.Math.abs;

import android.util.Log;
import android.widget.Switch;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.util.Encoder;

 @TeleOp
public class TeleOP_clasic extends OpMode {
    public DcMotorEx motorBR,motorBL,motorFL,motorFR;
    public Encoder leftEncoder,rightEncoder,frontEncoder;
    double sm = 1, lb = 1, rb = 1, sliderSlow = 1, intakePos = 0, rotitorPoz = 3, incheieturaPoz = 1;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    boolean stop = false, lastx = false, lasty = false, intaked = true, gherutaL = false, gherutaR = false, lastBumperL, lastBumperR, setSetpoint = true, automatizare = false, started_left = false, started_right = false,closed_left = false, closed_right = false;
    double servoPos = 0, servoPos2 = 0.85, pidResult = 0;
    long lastTime,lastTimeR,lastTimeL;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(pslider,islider,dslider);
    FunctiiDeAtunonom c = new FunctiiDeAtunonom();
    /*Functia de init se ruleaza numai o data, se folosete pentru initializarea motoarelor si chestii :)*/
    @Override
    public void init() {
        c.initSisteme(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorFL"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorFR"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorBL"));
        /* Liniile astea de cod fac ca motoarele sa corespunda cu cele din configuratie, cu numele dintre ghilimele.*/
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Back-Left
        /*Liniile astea de cod fac ca motoarele sa aiba puterea inversata fata de cum erau initial,
        sunt fol++osite pentru a face robotul sa mearga in fata dand putere pozitiva la toate cele 4 motoare. */
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);

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
    }
    public void start(){
        c.deschidere();
        Chassis.start();
        Systems.start();
        PID.start();
    }
    /*Aici se declara thread-ul cu numele chassis, pentru ca contine partea de program care se ocupa de sasiu*/
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while(!stop) {
                /* Liniile astea de cod iau input-ul controller-ului si il pun in niste variabile*/
                y  = -gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x * 1.5;
                rx = gamepad1.right_stick_x;

                /* Liniile astea de cod iau niste variabile care reprezinta puterea fiecarui motor, cu ajutorul puterilor de la controller*/
                pmotorFL = y + x + rx;
                pmotorBL = y - x + rx;
                pmotorBR = y + x - rx;
                pmotorFR = y - x - rx;

                /*Secventele urmatoare de cod stabilesc maximul dintre modulele puterilor motoarelor cu un anumit scop...*/
                max = abs(pmotorFL);
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }
                /*...care este punerea tuturor puterilor motoarelor sub 1, cum puterile de la motoare pot fi numai intre 1 si -1*/
                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }
                if(gamepad1.x != lastx){
                    rb += 0.5;
                    if(rb > 2){
                        rb = 0.5;
                    }
                }
                if(gamepad1.y != lasty){
                    lb += 0.5;
                    if(lb > 2){
                        lb = 0.5;
                    }
                }
                if(rb == 2){
                    sm = 4;
                }
                else if(lb == 2){
                    sm = 2;
                }
                else{
                    sm = 1;
                }
                lastx = gamepad1.x;
                lasty = gamepad1.y;
                POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
            }
        }
    });
    /*Aici se declara thread-ul cu numele systems, pentru ca contine partea de program care se ocupa de sisteme*/
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
//                if(gamepad2.left_trigger > 0){
//                    c.pixelIncheietura();
//                }
//                if(gamepad2.right_stick_y > 0){
//                    c.tablaIncheietura();
//                }
                if(gamepad2.a && gamepad2.dpad_down){
                    c.intake.setPower(1);
                }
                else if(gamepad2.b && gamepad2.dpad_down){
                    c.intake.setPower(0);
                }
                else if(gamepad2.b){
                    double intakepow = -c.intake.getPower();
                    c.intake.setPower(intakepow);
                }
                else if(gamepad2.a){
                    c.servointake.setPosition(0.675);
                }
                if(gamepad2.y){
                    c.servointake.setPosition(0.394);
                }
                if(gamepad2.x){
                    c.servointake.setPosition(0.57);
                }
               if(gamepad2.right_trigger > 0){
                    servoPos = 0.7;
                    automatizare = true;
                    c.slider_target(-300,1000,3000,5);
                    automatizare = false;
                    c.pixelIncheietura();
                    c.brat_stanga.setPosition(servoPos);
                    c.brat_dreapta.setPosition(1 - servoPos);
                    c.tablaIncheietura();
                    incheieturaPoz = 2;
                }
                if(gamepad2.left_trigger > 0){
                    automatizare = true;
                    servoPos = 0.4;
                    rotitorPoz = 3;
                    c.rotitorMijloc();
                    c.pixelIncheietura();
                    incheieturaPoz = 1;
                    c.slider_target(0,1000,3000,5);
                    c.brat_stanga.setPosition(servoPos);
                    c.brat_dreapta.setPosition(1 - servoPos);
                    lastTime = System.currentTimeMillis();
                }
                if(automatizare && lastTime + 800 < System.currentTimeMillis()){
                    servoPos = 0;
                    c.brat_stanga.setPosition(servoPos);
                    c.brat_dreapta.setPosition(1 - servoPos);
                    automatizare = false;
                }
                /*if(gamepad2.left_trigger > 0 && servoPos2 > 0){
                    servoPos2-=0.001;
                }
                if(gamepad2.right_trigger > 0 && servoPos2 < 1){
                    servoPos2+=0.001;
                }
                c.rotitor.setPosition(servoPos2);*/

                if(gamepad2.right_bumper){
                    c.gheara_stanga.setPosition(0.6);
                    started_left = false;
                }
                else if(gamepad2.left_bumper){
                    c.gheara_dreapta.setPosition(0.72);
                    started_right = false;
                }
                else{
                    if(!started_left && c.detect_left() && c.brat_stanga.getPosition() < 0.1){
                        lastTimeL = System.currentTimeMillis();
                        started_left = true;
                        closed_left = false;
                    }
                    if(lastTimeL + 500 <= System.currentTimeMillis() && !closed_left){
                        c.gheara_stanga.setPosition(1);
                        closed_left = true;
                    }
                    if(!started_right && c.detect_right() && c.brat_stanga.getPosition() < 0.1) {
                        lastTimeR = System.currentTimeMillis();
                        started_right = true;
                        closed_right = false;
                    }
                    if (lastTimeR + 500 <= System.currentTimeMillis() && !closed_right) {
                        c.gheara_dreapta.setPosition(0.65);
                        closed_right = true;
                    }
                }
                lastBumperL = gamepad2.left_bumper;
                lastBumperR = gamepad2.right_bumper;
                if(gamepad2.dpad_left){
                    rotitorPoz = 1;
                }
                if(gamepad2.dpad_right){
                    rotitorPoz = 2;
                }
                if(gamepad2.dpad_up){
                    rotitorPoz = 3;
                }
                if(rotitorPoz == 1){
                    c.rotitorStanga();
                }
                else if(rotitorPoz == 2){
                    c.rotitorDreapta();
                }
                else{
                    c.rotitorMijloc();
                }
                if(incheieturaPoz == 1){
                    c.pixelIncheietura();
                }
                else{
                    c.tablaIncheietura();
                }
            }
        }
    });
    private final Thread PID = new Thread(new Runnable() {
        @Override
        public void run() {
            pid.enable();
            while(!stop){
                pid.setPID(pslider, islider, dslider);
                if(gamepad2.left_stick_y != 0){
                    setSetpoint = true;
                    c.sliderL.setPower(gamepad2.left_stick_y);
                    c.sliderR.setPower(gamepad2.left_stick_y);
                }
                else if(!automatizare){
                    if(setSetpoint){
                        setSetpoint = false;
                        pid.setSetpoint(c.sliderL.getCurrentPosition());
                    }
                    else{
                        pidResult = pid.performPID(c.sliderL.getCurrentPosition());
                        c.sliderL.setPower(pidResult);
                        c.sliderR.setPower(pidResult);
                    }
                }
                else{
                    setSetpoint = true;
                }
            }
        }
    });
    /*Aici se afla partea de program care arata cand programul se opreste, este foarte folositor pentru functionarea thread-urilor*/
    public void stop(){stop = true;}

    /*Aici se afla partea de telemetrie a robotului.
    Telemetria iti arata pe driver hub/telefon cu driver station o valoare pe care ai stabilit-o, cu un anumit text dinaintea lui*/
    @Override
    public void loop() {
        /*Exemplu de telemetrie, in care Hotel este scrisul dinainte, si trivago este valoarea, care este un string cu numele trivago :)))))*/
        telemetry.addData("sliderL:",c.sliderL.getCurrentPosition());
        telemetry.addData("sliderR:",c.sliderR.getCurrentPosition());
        telemetry.addData("gheara dreapta:",c.gheara_dreapta.getPosition());
        telemetry.addData("gheara stanga:",c.gheara_stanga.getPosition());
        telemetry.addData("incheietura:", c.incheietura.getPosition());
        telemetry.addData("rotitor:",c.rotitor.getPosition());
        telemetry.addData("brat_stanga:",c.brat_stanga.getPosition());
        telemetry.addData("brat_dreapta:",c.brat_dreapta.getPosition());
        telemetry.addData("error:",pid.getError());
        telemetry.addData("setpoint:",pid.getSetpoint());
        telemetry.addData("color sensor right blue:",c.colorDreapta.blue());
        telemetry.addData("color sensor right red:",c.colorDreapta.red());
        telemetry.addData("color sensor right green:",c.colorDreapta.green());
        telemetry.addData("color sensor left blue:",c.colorStanga.blue());
        telemetry.addData("color sensor left red:",c.colorStanga.red());
        telemetry.addData("color sensor left green:",c.colorStanga.green());
        telemetry.addData("encoder right:",rightEncoder.getCurrentPosition());
        telemetry.addData("encoder left:",leftEncoder.getCurrentPosition());
        telemetry.addData("encoder front:",frontEncoder.getCurrentPosition());
        //telemetry.addData("switch:",swish.getMeasuredState());
        /*Aceasta functie face ca telemetria sa trimita date cat timp ruleaza programul*/
        telemetry.update();
    }
    /*Functia asta face ca toate motoarele a ruleze cu o anumita putere;
    Functiile sunt linii de cod comprimate in una singura, ceea ce este foarte fain daca vrei sa faci o secventa de linii de cod de mai multe ori. De asemenea, cand apelezi o functie, trebuie sa scrii si parametrii ei, daca exista.*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
}