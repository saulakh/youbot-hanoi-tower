import java.io.*;
import java.util.Arrays;
import java.util.Scanner;

public class CSV {

    public static void writeToCSV(String filePath, double[] config) {
        try (PrintWriter csvWriter = new PrintWriter(new FileOutputStream(filePath, true))) {
            csvWriter.println(Arrays.toString(config).replace("[", "").replace("]",""));
        } catch (FileNotFoundException e) {
            System.out.println("Cannot open or write to file.");
        }
    }

    public static void readFromCSV(String filePath) {
        File csvFile = new File(filePath);
        try(Scanner csvReader = new Scanner(csvFile)) {
            while (csvReader.hasNextLine()) {
                String currentConfig = csvReader.nextLine();
                String[] configArray = currentConfig.split(",");
            }
        }
        catch (FileNotFoundException e) {
            System.out.println("Cannot open or read file.");
        }
    }
}
