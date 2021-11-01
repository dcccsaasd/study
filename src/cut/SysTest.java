package cut;

public class SysTest {
    public static void main(String[] args) {
        String src[] = new String[] { "hello", "huang", "bao", "kang" };
        String dest[] = new String[5];
        System.arraycopy(src, 0, dest,  1, 4);
        for (String str : dest) {
            System.out.println(str);
        }
        System.out.println("=========华丽的分割线=========");
        System.arraycopy(src, 0, src, 1, 3);
        for (String str : src) {
            System.out.println(str);
        }
    }
}