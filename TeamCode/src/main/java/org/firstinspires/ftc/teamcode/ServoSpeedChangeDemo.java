package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import junit.framework.Test;

public class ServoSpeedChangeDemo
{
    private Servo s1;
    void setup(){
        s1.setDirection(Servo.Direction.FORWARD);
        s1.setPosition(0);
    }




    //情况1:直接传入要旋转到的角度参数,则舵机会以最大速度执行.
    void ChangePosition1()
    {
        s1.setPosition(1);
    }
    //情况2:这里使用一个循环,使舵机微调,并使用Thread.sleep方法来使程序停止一会,来达到"曲线"减速的目的.
    void ChangePosition2() throws InterruptedException
    {
        for(int i=0;i<=1;i+=0.05)
        {
            s1.setPosition(i);
            Thread.sleep(60);/*ps:使用这个方法会阻塞主线程,所以尽量新开一个线程来做,如果你不知道如何多线
            *程的话,看下面我给你的示例.*/
        }
    }
    //情况2的完善:多线程模式
    void ChangePosition2Mod()
    {
        Servoss Ch=new Servoss(1,this.s1);
        Ch.start();
    }


}
