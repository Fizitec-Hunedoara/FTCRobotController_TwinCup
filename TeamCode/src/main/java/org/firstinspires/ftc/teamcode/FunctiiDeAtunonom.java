package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FunctiiDeAtunonom {
    public DcMotorEx motorBR,motorBL,motorFL,motorFR;
    public DcMotorEx sliderR,sliderL,intake;
    public Servo servointake, brat_stanga, brat_dreapta, gheara_stanga, gheara_dreapta, incheietura,rotitor;
    public boolean isStopRequested = false;
    public void initSisteme(HardwareMap hard){
        sliderL = hard.get(DcMotorEx.class,"sliderL");
        sliderR = hard.get(DcMotorEx.class,"sliderR");

        intake = hard.get(DcMotorEx.class, "intake");

        servointake = hard.get(Servo.class, "servointake");
        brat_dreapta = hard.get(Servo.class, "brat_dreapta");
        brat_stanga = hard.get(Servo.class, "brat_stanga");
        gheara_dreapta = hard.get(Servo.class, "gheara_dreapta");
        gheara_stanga = hard.get(Servo.class, "gheara_stanga");
        incheietura = hard.get(Servo.class, "incheietura");
        rotitor = hard.get(Servo.class, "rotitor");

        sliderL.setDirection(DcMotorEx.Direction.REVERSE);

        intake.setDirection(DcMotorEx.Direction.REVERSE);

        sliderR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*Liniile astea de cod fac ca encoderele(masoara cat a mers motorul, dar nu este foarte precis, este necesar un cablu ca sa accesezi encoder-ul) sa se opreaca si sa se reseteze la valoarea initiala*/
        sliderL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void inchidere(){
        gheara_stanga.setPosition(1);
        gheara_dreapta.setPosition(0.65);
    }
    public void deschidere(){
        gheara_stanga.setPosition(0.6);
        gheara_dreapta.setPosition(0.72);
    }
    public void bratTabla(){
        brat_stanga.setPosition(0.42);
        brat_dreapta.setPosition(0.58);
    }
    public void bratIntake(){
        brat_stanga.setPosition(0);
        brat_dreapta.setPosition(1);
    }
    public void pixelIncheietura(){
        incheietura.setPosition(0.8);
    }
    public void tablaIncheietura(){
        incheietura.setPosition(0);
    }
    public void rotitorStanga(){
        rotitor.setPosition(1);
    }
    public void rotitorDreapta(){
        rotitor.setPosition(0.748);
    }
    public void rotitorMijloc(){
        rotitor.setPosition(0.873);
    }
    public synchronized void target(double poz, double vel, DcMotorEx motor, double t, int tolerance) {
        if (motor.getCurrentPosition() < poz) {
            motor.setVelocity(vel);
        }
        else {
            motor.setVelocity(-vel);
        }
        double lastTime = System.currentTimeMillis();
        while (!isStopRequested
                && lastTime + t > System.currentTimeMillis()
                && (abs(motor.getCurrentPosition() - poz) > tolerance)) {
        }
        motor.setVelocity(0);
    }
    public synchronized void slider_target(double poz, double vel, double t, int tolerance) {
        if (sliderL.getCurrentPosition() < poz) {
            sliderL.setVelocity(vel);
            sliderR.setVelocity(vel);
        }
        else {
            sliderL.setVelocity(-vel);
            sliderR.setVelocity(-vel);
        }
        double lastTime = System.currentTimeMillis();
        while (!isStopRequested
                && lastTime + t > System.currentTimeMillis()
                && (abs(sliderL.getCurrentPosition() - poz) > tolerance)) {
        }
        sliderL.setVelocity(0);
        sliderR.setVelocity(0);
    }
}
