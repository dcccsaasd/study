package crewpairing;

import java.nio.file.Path;
import java.sql.Time;

public class crew {
    //机场节点类
     class AP_Node{
        private String City_Name;
        private boolean Is_AP_Base;
        private int Children_Num;
        private Flight_Node Children;

        public AP_Node(){//内有参数，String,bollean,int,Flight_Node

        }
    }
    //航班节点类
    class Flight_Node{
        private int Flight_No;
        private boolean Is_Leaf;
        //private int Leg_Num;
        //private Leg Leg_Node;
        private Flight_Node Children;
        private Flight_Node Father;
        private int Children_Num;
        private int Current_Children_Num;
        private Flight_Node Left_Flight_Node,Right_Flight_Node;
        private AP_Node Dep_Ctiy,Arr_City;
        //private time Dep_Time,Arr_Time;
        //private time Flight_Time,Turnover_Time;
        private Flight_Node Dep_Time,Arr_Time;
        private Flight_Node Flight_Time,Turnover_Time;

        public Flight_Node() { //内有参数int,int,int
            Current_Children_Num=Children_Num;
        }

        public void Cal_Flight_Time(){

        }

        public void Cal_Turnover_Time(){

        }

        public Flight_Node Copy(){
            return null;
        }

    }
    //航节类，由于是单航节，所以不考虑
    class Leg{

    }
    //机场出发航班树类
    public class AP_Flight_Tree extends AP_Node{

         public AP_Flight_Tree(){

         }

         public void Children_Connect(){

         }

    }
    //航班树类
    class Flight_Tree{
         private int AP_Num;
         private int Flight_Node_Num;
         private int Level_Num;
         private int Leaf_Node_Num;
         private AP_Node Root_AP;
         private Flight_Node Current_Flight;
         private Flight_Node Leaf_Flight_Node;
         private AP_Flight_Tree AP_Flight_Tree_Node;

         //Flight_Tree(){}//;缺省构造函数
         public Flight_Tree(){//内有参数int,int,int,int,AP_Node

         }

         public void Connect_AP_Tree(){

         }

         public void Cut_Branch(){

         }

         public void Construct(){

         }

         public void Show_Tree(){

         }

         public void Save_Tree(){

         }
    }
    //航班连接网络类
    class Flight_Network extends Flight_Tree{
         private Path Flight_Path;

         public Flight_Network(){

         }

         public void Construct_Flight_Network(){

         }

         public void Save_Flight_Network(){

         }

         public Path Search_Path(){//内有参数int
             return null;
         }

         public void Save_Path(){

         }
    }
    //航班环路(路径类)
    class Path{
         private int min_Flight_Num,max_Flight_Num;
         //private time max_FLight_Time,max_Elapse_Time;
         private Flight_Network thisNetwork;
         //private Path Flight_String;
         //private Stack Current_path;
         private Flight_Node Current_Flight;
         //private time Flight_Time,Elapse_Time;

         /*
         public Path(){//内有参数Flight_Network

         }
         */
         public void Deep_Search(){

         }

         public boolean Judge_Regulation(){
             return true;
         }

         public void Cal_Flight_Time(){

         }

         public void Cal_Elapse_Time(){

         }

         public void Output_path(){

         }

         public void Save_Path(){

         }

    }
}