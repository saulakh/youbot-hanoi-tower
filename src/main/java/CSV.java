import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class CSV {

    public void writeToCSV(String filePath) throws IOException {
        File file = new File(filePath);
        FileWriter outputFile = new FileWriter(file);
        //CSVWriter writer = new CSVWriter(outputFile);
    }
}
