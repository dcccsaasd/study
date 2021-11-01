package UWR;
public class LocationMath {


    public static void printf_matrix(float m[][]) throws Exception{
        int i = 0;
        int j = 0;

        for (i = 0;i < 3;i++)
        {
            for (j = 0;j < 3;j++)
            {
                System.out.println(m[i*3][j]);
            }
        }

    }

    //��ά����ʽ��ֵ
    //m:3 * 3����
    public static double det(float m[][]) throws Exception{
        double value = 0.0;

        value = m[0][0] * m[1][1] * m[2][2] +
                m[0][1] * m[1][2] * m[2][0] +
                m[0][2] * m[1][0] * m[2][1] -
                m[0][1] * m[1][0] * m[2][2] -
                m[0][2] * m[1][1] * m[2][0] -
                m[0][0] * m[1][2] * m[2][1];

        return value;
    }

    //��һ������ʽ��ֵ������һ��
    //src,dst:3 * 3����
    public static void copy_matrix(float src[][],float dst[][]) throws Exception {
        int i = 0;
        int j = 0;

        for (i = 0;i < 3;i++)
        {
            for (j = 0;j < 3;j++)
            {
                dst[i][j] = src[i][j];
            }
        }

    }

    //�ⷽ��
    //m:����,3 * 3����
    //b:��
    //x:����ֵ
    //fail:back -1
    public static float[] solve(float m[][],float b[]) throws Exception {

        float det_m;
        float det_m_temp;
        float m_temp[][] = new float[3][3];
        int i = 0;
        int j = 0;

        float x[]=new float[3];

        det_m = (float) det(m);
        if (det_m == 0)
        {
            return null;
        }
        for (j = 0;j < 3;j++)
        {
            //�õ��µ�����ʽ
            copy_matrix(m,m_temp);
            for (i = 0;i < 3;i++)
            {
                m_temp[i][j] = b [i];
            }
            det_m_temp = (float) det(m_temp);

            //���
            x[j] = det_m_temp / det_m;
        }

        return x;

    }

    //����ռ�㵽ԭ������ƽ��
    public static float d_p_square(float p[]) throws Exception {

        float d = 0;
        int i = 0;

        for (i = 0;i < 3;i++)
        {
            d += p[i] * p [i];
        }

        return d;

    }

}