import org.junit.Test;

public class CSVTests {

    CSV csv;
    double[] config = new double[] {3.14159/6,-0.1,0.1,0,-0.2,0.2,-1.6,0,0,0,0,0,0};;
    String filePath = "test.csv";

    @Test
    public void checkCsvWriter() {
        CSV.writeToCSV(filePath, config);
    }

    @Test
    public void checkCsvReader() {
        CSV.readFromCSV(filePath);
    }
}
