import java.io.*;
import java.util.Arrays;
import java.util.Scanner;

public class CSV {

    public static void clearCSVFile(String filePath) {
        try (PrintWriter csvWriter = new PrintWriter(filePath)) {
            csvWriter.close();
        } catch (FileNotFoundException e) {
            System.out.println("Cannot clear file contents.");
        }
    }

    public static void writeToCSV(String filePath, double[] config) {
        try (PrintWriter csvWriter = new PrintWriter(new FileOutputStream(filePath, true))) {
            csvWriter.println(Arrays.toString(config).replace("[", "").replace("]",""));
        } catch (FileNotFoundException e) {
            System.out.println("Cannot open or write to file.");
        }
    }

    public static double[] readFromCSV(String filePath) {
        File csvFile = new File(filePath);
        double[] configArray = null;
        try(Scanner csvReader = new Scanner(csvFile)) {
            while (csvReader.hasNextLine()) {
                String currentConfig = csvReader.nextLine();
                String[] configStrings = currentConfig.split(",");
                configArray = convertStringToDoubleArray(configStrings);
            }
        }
        catch (FileNotFoundException e) {
            System.out.println("Cannot open or read file.");
        }
        return configArray;
    }

    public static double[] convertStringToDoubleArray(String[] stringArray) {
        double[] doubleArray = new double[stringArray.length];
        for (int i=0; i < stringArray.length; i++) {
            doubleArray[i] = Double.parseDouble(stringArray[i]);
        }
        return doubleArray;
    }
}
