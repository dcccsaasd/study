package SACG_5minTiaoTime;

import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.io.StreamTokenizer;

public class InputDataReader {
    public static class InputDataReaderException extends Exception {
        private static final long serialVersionUID = 1021L;

        InputDataReaderException(String file) {
            super("'" + file + "' contains bad data format");
        }
    }

    StreamTokenizer _tokenizer;
    Reader _reader;
    String _fileName;

    public InputDataReader(String fileName) throws IOException {
        _reader = new FileReader(fileName);
        _fileName = fileName;

        _tokenizer = new StreamTokenizer(_reader);

        // State the '"', '\'' as white spaces.
        _tokenizer.whitespaceChars('"', '"');
        _tokenizer.whitespaceChars('\'', '\'');

        // State the '[', ']' as normal characters.
        _tokenizer.ordinaryChar('[');
        _tokenizer.ordinaryChar(']');
        _tokenizer.ordinaryChar(',');
    }

    protected void finalize() throws Throwable {
        _reader.close();
    }

    double readDouble() throws InputDataReaderException,
            IOException {
        int ntType = _tokenizer.nextToken();

        if (ntType != StreamTokenizer.TT_NUMBER)
            throw new InputDataReaderException(_fileName);

        return _tokenizer.nval;
    }//读取double型数据

    int readInt() throws InputDataReaderException,
            IOException {
        int ntType = _tokenizer.nextToken();

        if (ntType != StreamTokenizer.TT_NUMBER)
            throw new InputDataReaderException(_fileName);

        return (new Double(_tokenizer.nval)).intValue();
    }//读取int型数据

    double[] readDoubleArray() throws InputDataReaderException,
            IOException {
        int ntType = _tokenizer.nextToken(); // Read the '['

        if (ntType != '[')
            throw new InputDataReaderException(_fileName);

        DoubleArray values = new DoubleArray();
        ntType = _tokenizer.nextToken();
        while (ntType == StreamTokenizer.TT_NUMBER) {
            values.add(_tokenizer.nval);
            ntType = _tokenizer.nextToken();

            if (ntType == ',') {
                ntType = _tokenizer.nextToken();
            } else if (ntType != ']') {
                throw new InputDataReaderException(_fileName);
            }
        }

        if (ntType != ']')
            throw new InputDataReaderException(_fileName);

        // Allocate and fill the array.
        double[] res = new double[values.getSize()];
        for (int i = 0; i < values.getSize(); i++) {
            res[i] = values.getElement(i);
        }

        return res;
    }//读取double型一维数组

    double[][] readDoubleArrayArray() throws InputDataReaderException,
            IOException {
        int ntType = _tokenizer.nextToken(); // Read the '['

        if (ntType != '[')
            throw new InputDataReaderException(_fileName);

        DoubleArrayArray values = new DoubleArrayArray();
        ntType = _tokenizer.nextToken();

        while (ntType == '[') {
            _tokenizer.pushBack();

            values.add(readDoubleArray());

            ntType = _tokenizer.nextToken();
            if (ntType == ',') {
                ntType = _tokenizer.nextToken();
            } else if (ntType != ']') {
                throw new InputDataReaderException(_fileName);
            }
        }

        if (ntType != ']')
            throw new InputDataReaderException(_fileName);

        // Allocate and fill the array.
        double[][] res = new double[values.getSize()][];
        for (int i = 0; i < values.getSize(); i++) {
            res[i] = new double[values.getSize(i)];
            for (int j = 0; j < values.getSize(i); j++) {
                res[i][j] = values.getElement(i, j);
            }
        }
        return res;
    }//读取double型二维数组

    int[] readIntArray() throws InputDataReaderException,
            IOException {
        int ntType = _tokenizer.nextToken(); // Read the '['

        if (ntType != '[')
            throw new InputDataReaderException(_fileName);

        IntArray values = new IntArray();
        ntType = _tokenizer.nextToken();
        while (ntType == StreamTokenizer.TT_NUMBER) {
            values.add(_tokenizer.nval);
            ntType = _tokenizer.nextToken();

            if (ntType == ',') {
                ntType = _tokenizer.nextToken();
            } else if (ntType != ']') {
                throw new InputDataReaderException(_fileName);
            }
        }

        if (ntType != ']')
            throw new InputDataReaderException(_fileName);

        // Allocate and fill the array.
        int[] res = new int[values.getSize()];
        for (int i = 0; i < values.getSize(); i++) {
            res[i] = values.getElement(i);
        }
        return res;
    }//读取int型一维数组

    public int[][] readIntArrayArray() throws InputDataReaderException,
            IOException {
        int ntType = _tokenizer.nextToken(); // Read the '['

        if (ntType != '[')
            throw new InputDataReaderException(_fileName);

        IntArrayArray values = new IntArrayArray();
        ntType = _tokenizer.nextToken();

        while (ntType == '[') {
            _tokenizer.pushBack();

            values.add(readIntArray());

            ntType = _tokenizer.nextToken();
            if (ntType == ',') {
                ntType = _tokenizer.nextToken();
            } else if (ntType != ']') {
                throw new InputDataReaderException(_fileName);
            }
        }

        if (ntType != ']')
            throw new InputDataReaderException(_fileName);

        // Allocate and fill the array.
        int[][] res = new int[values.getSize()][];
        for (int i = 0; i < values.getSize(); i++) {
            res[i] = new int[values.getSize(i)];
            for (int j = 0; j < values.getSize(i); j++) {
                res[i][j] = values.getElement(i, j);
            }
        }
        return res;
    }//读取int型二维数组

    private class DoubleArray {
        int _num = 0;
        double[] _array = new double[32];

        final void add(double dval) {
            if (_num >= _array.length) {
                double[] array = new double[2 * _array.length];
                System.arraycopy(_array, 0, array, 0, _num);
                _array = array;
            }
            _array[_num++] = dval;
        }

        final double getElement(int i) {
            return _array[i];
        }

        final int getSize() {
            return _num;
        }
    }//将dval数据添加到DoubleArray对象中

    private class DoubleArrayArray {
        int _num = 0;
        double[][] _array = new double[32][];

        final void add(double[] dray) {

            if (_num >= _array.length) {
                double[][] array = new double[2 * _array.length][];
                for (int i = 0; i < _num; i++) {
                    array[i] = _array[i];
                }
                _array = array;
            }
            _array[_num] = new double[dray.length];
            System.arraycopy(dray, 0, _array[_num], 0, dray.length);
            _num++;
        }

        final double getElement(int i, int j) {
            return _array[i][j];
        }

        final int getSize() {
            return _num;
        }

        final int getSize(int i) {
            return _array[i].length;
        }
    }//将dval数据添加到DoubleArrayArray对象中


    private class IntArray {
        int _num = 0;
        int[] _array = new int[32];

        final void add(double ival) {
            if (_num >= _array.length) {
                int[] array = new int[2 * _array.length];
                System.arraycopy(_array, 0, array, 0, _num);
                _array = array;
            }
            _array[_num++] = (int) Math.round(ival);
        }

        final int getElement(int i) {
            return _array[i];
        }

        final int getSize() {
            return _num;
        }
    }//将dval数据添加到IntArray对象中

    private class IntArrayArray {
        int _num = 0;
        int[][] _array = new int[32][];

        final void add(int[] iray) {

            if (_num >= _array.length) {
                int[][] array = new int[2 * _array.length][];
                for (int i = 0; i < _num; i++) {
                    array[i] = _array[i];
                }
                _array = array;
            }
            _array[_num] = new int[iray.length];
            System.arraycopy(iray, 0, _array[_num], 0, iray.length);
            _num++;
        }

        final int getElement(int i, int j) {
            return _array[i][j];
        }

        final int getSize() {
            return _num;
        }

        final int getSize(int i) {
            return _array[i].length;
        }
    }
}//将dval数据添加到IntArrayArray对象中

