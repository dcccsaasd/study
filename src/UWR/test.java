package UWR;
public class test {
    public static void main(String args[]) {

        try {

            float point[] = new float[3];
            Location loc = new Location();

            //»ñµÃ×ø±ê
            point[0] = 0;
            point[1] = 0;
            point[2] = 1300;
            loc.set_point(point, 1);

            point[0] = 5000;
            point[1] = 0;
            point[2] = 1700;
            loc.set_point(point, 2);

            point[0] = 0;
            point[1] = 5000;
            point[2] = 1700;
            loc.set_point(point, 3);

            point[0] = 5000;
            point[1] = 5000;
            point[2] = 1300;
            loc.set_point(point, 4);

            //distance
            loc.set_distance(1, 1);
            loc.set_distance(2, 2);
            loc.set_distance(3, 3);
            loc.set_distance(4, 4);

            //calc
            float x[] = loc.calc();
            if (x == null) {
                System.out.println("fail");
            } else {
                System.out.println(x[0] + "," + x[1] + "," + x[2]);
            }

        } catch (Exception ex) {
            ex.printStackTrace();
        }

    }
}