package SACG_5minTiaoTime;

import java.io.*;

class DataReader {

    private StreamTokenizer st;

    public DataReader(String filename) throws IOException { //throws IOException 抛出异常
        FileInputStream fstream = new FileInputStream(filename);
        Reader r = new BufferedReader(new InputStreamReader(fstream));
        st = new StreamTokenizer(r);
    }

    public int next() throws IOException {
        st.nextToken();
        return (int) st.nval;
    }
}

