package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by students on 1/9/16.
 */
public class TimerHelper {
    private long triggerTime; // In milliseconds

    public void setWaitPeriod(long millis) {
        triggerTime = System.currentTimeMillis() + millis;
    }

    public boolean isDone() {
        if (triggerTime == 0)
            return false;

        long tTime = triggerTime;
        triggerTime = 0;
        return System.currentTimeMillis() >= tTime;
    }
}
