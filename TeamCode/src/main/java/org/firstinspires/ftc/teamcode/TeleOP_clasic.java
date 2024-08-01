/* La inceputul programului, exista import-urile. Ele se fac de obicei automat asa ca nu te ingrijora de ele numai daca dau eroare(Nu au dat niciodata lol).
De asemenea, daca ceva da eroare in cod si nu stii de ce, verifica mai intai daca este importata chestia sau nu.
 */
package org.firstinspires.ftc.teamcode;
import static java.lang.Math.abs;

import android.util.Log;
import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TeleOP_clasic extends OpMode {
    public Switch swish;
    public DcMotorEx motorBR,motorBL,motorFL,motorFR;
    public DcMotorEx sliderR,sliderL,brat;
    public Servo ghearaR,ghearaL,servodubios;
    public TouchSensor sliderTouch;
    double sm = 1, lb = 1, rb = 1, sliderSlow = 1;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    boolean stop = false, lastx = false, lasty = false, sliderState = true, aIntrat = false,aAjuns = true,aInchis = true;
    double intPoz = 0.4, servoPos = 0.0;
    /*Functia de init se ruleaza numai o data, se folosete pentru initializarea motoarelor si chestii :)*/
    @Override
    public void init() {
        /* Liniile astea de cod fac ca motoarele sa corespunda cu cele din configuratie, cu numele dintre ghilimele.*/
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Back-Left

        sliderL = hardwareMap.get(DcMotorEx.class,"sliderL");
        sliderR = hardwareMap.get(DcMotorEx.class,"sliderR");
        /*Liniile astea de cod fac ca motoarele sa aiba puterea inversata fata de cum erau initial,
        sunt fol++osite pentru a face robotul sa mearga in fata dand putere pozitiva la toate cele 4 motoare. */
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        sliderL.setDirection(DcMotorEx.Direction.REVERSE);

        /*Liniile astea de cod fac ca motoarele sa poata frana de tot atunci cand ii dai sa franeze*/
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sliderR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        /*Liniile astea de cod fac ca encoderele(masoara cat a mers motorul, dar nu este foarte precis, este necesar un cablu ca sa accesezi encoder-ul) sa se opreaca si sa se reseteze la valoarea initiala*/
        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        sliderL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        /*Liniile astea de cod fac ca robotul sa mearga cu ajutorul encoderelor(maresc precizia)*/
        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        sliderL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void start(){
        Chassis.start();
        Systems.start();
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
                sliderL.setPower(gamepad2.left_stick_y);
                sliderR.setPower(gamepad2.left_stick_y);
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
        telemetry.addData("sliderL:",sliderL.getCurrentPosition());
        telemetry.addData("sliderR:",sliderR.getCurrentPosition());
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