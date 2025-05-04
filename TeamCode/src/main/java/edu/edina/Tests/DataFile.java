package edu.edina.Tests;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;

public class DataFile {
    private File file;
    private PrintStream out;

    public DataFile(String name) {
        File root = Environment.getExternalStorageDirectory();
        file = new File(root, name);

        try {
            FileOutputStream stream = new FileOutputStream(file);
            out = new PrintStream(stream);
        } catch (Exception x) {
            out = null;
        }
    }

    public void showStatus(Telemetry telemetry) {
        if (out != null) {
            telemetry.addData("data file", file);
        } else {
            telemetry.addLine("could not open data file");
        }
    }

    public void println(String s) {
        if (out != null) {
            out.println(s);
            out.flush();
        }
    }
}
