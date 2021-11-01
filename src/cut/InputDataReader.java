/* --------------------------------------------------------------------------
 * File: InputDataReader.java
 * Version 12.4  
 * --------------------------------------------------------------------------
 * Licensed Materials - Property of IBM
 * 5725-A06 5725-A29 5724-Y48 5724-Y49 5724-Y54 5724-Y55
 * Copyright IBM Corporation 2001, 2011. All Rights Reserved.
 *
 * US Government Users Restricted Rights - Use, duplication or
 * disclosure restricted by GSA ADP Schedule Contract with
 * IBM Corp.
 * --------------------------------------------------------------------------
 *
 * This is a helper class used by several examples to read input data files
 * containing arrays in the format [x1, x2, ..., x3].  Up to two-dimensional
 * arrays are supported.
 */
package cut;
import java.io.*;

public class InputDataReader {
   //自定义异常
   public static class InputDataReaderException extends Exception {
      private static final long serialVersionUID = 1021L;//序列号，一般不需要
      InputDataReaderException(String file) {
         super("'" + file + "' contains bad data format");
      }
   }
   StreamTokenizer _tokenizer;
   Reader _reader;
   String _fileName;
   //开始读数据
   public InputDataReader(String fileName) throws IOException {
      _reader = new FileReader(fileName);
      _fileName = fileName;
    //Create a tokenizer that parses the given character stream.
      _tokenizer = new StreamTokenizer(_reader);
    
      // State the '"', '\'' as white spaces.
      _tokenizer.whitespaceChars('"', '"');
      _tokenizer.whitespaceChars('\'', '\'');
    
      // State the '[', ']' as normal characters.
      _tokenizer.ordinaryChar('[');
      _tokenizer.ordinaryChar(']');
      _tokenizer.ordinaryChar(',');
   }
   //结束读数据
   protected void finalize() throws Throwable {
      _reader.close();
   }


   //分割线---------定义读取数据的方法，只有一个main()方法，所以都省略了修饰名public----------//

   //读取double型数据
   double readDouble() throws InputDataReaderException,
                              IOException {
      //读取下一个内容
	   int ntType = _tokenizer.nextToken();
	 //判断读取内容是否是数据
      if ( ntType != StreamTokenizer.TT_NUMBER )//TT_NUMBER指示已读到一个数字标记的常量
         throw new InputDataReaderException(_fileName);
      //返回当前读取的数据，nval字段将包含该数字的值
      return _tokenizer.nval;
   }
   	//读取int型数据
   int readInt() throws InputDataReaderException,
                        IOException {
      int ntType = _tokenizer.nextToken();
    //读取内容不是数
      if ( ntType != StreamTokenizer.TT_NUMBER )
         throw new InputDataReaderException(_fileName);
      
      return (new Double(_tokenizer.nval)).intValue();
   }
   //读取double型一维数组
   double[] readDoubleArray() throws InputDataReaderException,
                                     IOException {
      int ntType = _tokenizer.nextToken(); // Read the '['
      
      if ( ntType != '[' )
         throw new InputDataReaderException(_fileName);

      //声明并实例化对象values，它属于DoubleArry类
      DoubleArray values = new DoubleArray();//下文中有关于DoubleArry类的定义
      ntType = _tokenizer.nextToken();
      while (ntType == StreamTokenizer.TT_NUMBER) {

         values.add(_tokenizer.nval);//方法add在DoubleArry类中有定义
         ntType = _tokenizer.nextToken();
         
         if ( ntType == ',' ) {
            ntType = _tokenizer.nextToken();
         }
         else if ( ntType != ']' ) {
            throw new InputDataReaderException(_fileName);
         }
      }
      
      if ( ntType != ']' )
         throw new InputDataReaderException(_fileName);
    
      // Allocate and fill the array.
      double[] res = new double[values.getSize()];
      //将链表中的数据复制到数组中
      for (int i = 0; i < values.getSize(); i++) {
         res[i] = values.getElement(i);
      }
      
      return res;
   }
 //读取double型二维数组
   double[][] readDoubleArrayArray() throws InputDataReaderException,
                                            IOException {
      int ntType = _tokenizer.nextToken(); // Read the '['
      
      if ( ntType != '[' )
         throw new InputDataReaderException(_fileName);
      
      DoubleArrayArray values = new DoubleArrayArray();
      ntType = _tokenizer.nextToken();
      
      while (ntType == '[') {
         _tokenizer.pushBack();
         
         values.add(readDoubleArray());
         
         ntType = _tokenizer.nextToken();
         if      ( ntType == ',' ) {
           ntType = _tokenizer.nextToken();
         }
         else if ( ntType != ']' ) {
           throw new InputDataReaderException(_fileName);
         }
      }
    
      if ( ntType != ']' )
         throw new InputDataReaderException(_fileName);
    
      // Allocate and fill the array.
      double[][] res = new double[values.getSize()][];
      for (int i = 0; i < values.getSize(); i++) { 
         res[i] = new double[values.getSize(i)];
         for (int j = 0; j < values.getSize(i); j++) { 
            res[i][j] = values.getElement(i,j);
         }
      }
      return res;
   }
 //读取int型一维数组
   int[] readIntArray() throws InputDataReaderException,
                               IOException {
      int ntType = _tokenizer.nextToken(); // Read the '['
      
      if ( ntType != '[' )
         throw new InputDataReaderException(_fileName);
      
      IntArray values = new IntArray();
      ntType = _tokenizer.nextToken();
      while (ntType == StreamTokenizer.TT_NUMBER) {
         values.add(_tokenizer.nval);
         ntType = _tokenizer.nextToken();
         
         if      ( ntType == ',' ) {
            ntType = _tokenizer.nextToken();
         }
         else if ( ntType != ']' ) {
            throw new InputDataReaderException(_fileName);
         }
      }
      
      if ( ntType != ']' )
         throw new InputDataReaderException(_fileName);

      // Allocate and fill the array.
      int[] res = new int[values.getSize()];
      for (int i = 0; i < values.getSize(); i++) {
         res[i] = values.getElement(i);
      }
      return res;
   }
 //读取int型二维数组
   int[][] readIntArrayArray() throws InputDataReaderException,
                                      IOException {
      int ntType = _tokenizer.nextToken(); // Read the '['
      
      if ( ntType != '[' )
         throw new InputDataReaderException(_fileName);
      
      IntArrayArray values = new IntArrayArray();
      ntType = _tokenizer.nextToken();
      
      while (ntType == '[') {
         _tokenizer.pushBack();
         
         values.add(readIntArray());
         
         ntType = _tokenizer.nextToken();
         if      ( ntType == ',' ) {
            ntType = _tokenizer.nextToken();
         }
         else if ( ntType != ']' ) {
            throw new InputDataReaderException(_fileName);
         }
      }
    
      if ( ntType != ']' )
         throw new InputDataReaderException(_fileName);
    
      // Allocate and fill the array.
      int[][] res = new int[values.getSize()][];
      for (int i = 0; i < values.getSize(); i++) {
         res[i] = new int[values.getSize(i)];
         for (int j = 0; j < values.getSize(i); j++) {
            res[i][j] = values.getElement(i,j);
         }
      }    
      return res;
   }

   //分割线--------------定义各种类,与定义方法的方式不同--------------//

   //将dval数据添加到DoubleArray对象中
   private class DoubleArray {
      //类中成员的属性
      int      _num   = 0;
      double[] _array = new double[32];

      //定义三种方法
      final void add(double dval) {
    	  	//当数据的长度超过DoubleArray对象数组的长度，则增加数组长度
         if ( _num >= _array.length ) {
            double[] array = new double[2 * _array.length];
            //_num是_array的长度
            System.arraycopy(_array, 0, array, 0, _num);//这里的i和i1是编辑器自己提示的，没有特别的含义
            _array = array;
         }
         _array[_num++] = dval;
      }
      final double getElement(int i) { return _array[i]; }
      final int    getSize()         { return _num; }
   }
 //将dval数据添加到DoubleArrayArray对象中
   private class DoubleArrayArray {
      int        _num   = 0;
      double[][] _array = new double[32][];

      final void add(double[] dray) {

         if ( _num >= _array.length ) {
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
      //获取矩阵_array[i][j]元素
      final double getElement(int i, int j) { return _array[i][j]; }
      //获取数组一维长度
      final int    getSize()                { return _num; }
    //获取数组二维维长度
      final int    getSize(int i)           { return _array[i].length; }
   }

 //将dval数据添加到IntArray对象中
   private class IntArray {
      int   _num   = 0;
      int[] _array = new int[32];

      final void add(double ival) {
         if ( _num >= _array.length ) {
            int[] array = new int[2 * _array.length];
            System.arraycopy(_array, 0, array, 0, _num);
            _array = array;
         }
         _array[_num++] = (int)Math.round(ival);
      }
    //获取矩阵_array[i]元素
      final int getElement(int i) { return _array[i]; }
      //获取数组长度
      final int getSize()         { return _num; }
   }
 //将dval数据添加到IntArrayArray对象中
   private class IntArrayArray {
      int     _num   = 0;
      int[][] _array = new int[32][];

      final void add(int[] iray) {

         if ( _num >= _array.length ) {
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

      final int getElement(int i, int j) { return _array[i][j]; }
      final int getSize()                { return _num; }
      final int getSize(int i)           { return _array[i].length; }
   }
}