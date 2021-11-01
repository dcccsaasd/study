package SACG_20minTiaoTime10;

import ilog.concert.IloException;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;

public class OutConsole {
    public static void write(String name) throws IloException, IOException {
        File f = new File(name);
        f.createNewFile();
        FileOutputStream fileOutputStream = new FileOutputStream(f);
        PrintStream printStream = new PrintStream(fileOutputStream);
        System.setOut(printStream);
    }
}

