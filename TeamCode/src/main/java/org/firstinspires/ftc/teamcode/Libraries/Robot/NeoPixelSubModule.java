package org.firstinspires.ftc.teamcode.Libraries.Robot;

public enum NeoPixelSubModule {
    PIN(0x1),
    BUFLEN(0x3),
    BUF(0x4),
    SHOW(0x5);

    public int val;

    NeoPixelSubModule(int val) {
        this.val = val;
    }

    public byte getVal() {
        return (byte) val;
    }
}
