package drone.decentralizedswarmcoverage_fbe_rps;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * Headless batch tester for the Decentralized Swarm Coverage project.
 *
 * Uses the SAME DecentralizedExploration constructor as CanvasView.run().
 * Only the JVM property swarm.lambda changes between paired experiments.
 * Does not use or start the JavaFX application.
 *
 * Usage (IntelliJ): Run Tester.main() with the project root as working directory.
 * Optional program arguments: [dataset_directory] [output_csv_file]
 */
public final class Tester {
    // Change to "," if the target spreadsheet application uses comma-separated CSV.
    private static final String delimiter = ";";
    private static final int SENSOR_RANGE_CELLS = 4; // CanvasView.sensorFoV
    private static final int CELL_SIZE = 10;         // MainView.cellSize; rendering-only
    private static final int MAX_ROWS = 200;        // CanvasView's file limits
    private static final int MAX_COLS = 400;
    private static final int EXPECTED_DRONES = 4;
    private static final int EXPECTED_SAMPLES = 100;
    private static final String COMPLETE = "ALL_REACHABLE_FREE_CELLS_OBSERVED";
    private static final String PACKAGE_PATH = "drone/decentralizedswarmcoverage_fbe_rps";
    private static final DateTimeFormatter DATE_FORMAT = DateTimeFormatter.ofPattern("yyyyMMddHHmmss", Locale.ROOT);

    private static final List<String> CSV_COLUMNS = List.of(
            "Environment", "Sample ID", "Must Explored",
            "LAMBDA_0 Explored", "LAMBDA_0 Num Revisit", "LAMBDA_0 Num Overlap",
            "LAMBDA_0 Coverage(%)", "LAMBDA_0 Revisit (%)", "LAMBDA_0 Overlap (%)", "LAMBDA_0 Steps",
            "LAMBDA_1 Explored", "LAMBDA_1 Num Revisit", "LAMBDA_1 Num Overlap",
            "LAMBDA_1 Coverage(%)", "LAMBDA_1 Revisit (%)", "LAMBDA_1 Overlap (%)", "LAMBDA_1 Steps"
    );

    private Tester() { }

    public static void main(String[] args) {
        if (args.length > 2) {
            System.err.println("Usage: Tester [dataset_directory] [output_csv_file]");
            System.exit(1);
        }

        String originalLambda = System.getProperty("swarm.lambda");
        try {
            validateProtocol();
            Path dataset = args.length >= 1
                    ? Paths.get(args[0]).toAbsolutePath().normalize()
                    : findDatasetDirectory();
            Path csv = args.length == 2
                    ? Paths.get(args[1]).toAbsolutePath().normalize()
                    : Paths.get(System.getProperty("user.dir"), "results",
                            "ExplorationResults" + LocalDateTime.now().format(DATE_FORMAT) + ".csv")
                            .toAbsolutePath().normalize();

            if (!csv.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".csv")) {
                throw new IOException("Output file must have the .csv extension: " + csv);
            }
            if (!Files.isDirectory(dataset)) {
                throw new IOException("Dataset directory not found: " + dataset);
            }

            List<Path> files = collectSamples(dataset);
            System.out.println("Dataset : " + dataset);
            System.out.println("Samples : " + files.size() + " (each tested with λ=0 and λ=1)");
            System.out.println("Output  : " + csv);
            System.out.println("Protocol: drones=4, sensing=4 cells, rho=0.05, deposit=1.0, "
                    + "recovery=true, maxSteps=2500, stallLimit=30");

            // Fully validate the dataset and paired geometry BEFORE running experiments.
            List<Sample> samples = new ArrayList<>();
            Map<String, int[][]> geometryById = new HashMap<>();
            Map<String, Set<String>> startsByGeometry = new HashMap<>();
            for (Path file : files) {
                Sample sample = loadSample(file);
                String id = sample.sampleId();
                String geometryId = id.substring(0, id.length() - 2);
                int[][] previous = geometryById.putIfAbsent(geometryId, sample.environment());
                if (previous != null && !Arrays.deepEquals(previous, sample.environment())) {
                    throw new IOException("Paired C/D geometry mismatch: " + geometryId);
                }
                startsByGeometry.computeIfAbsent(geometryId, unused -> new HashSet<>())
                        .add(id.substring(id.length() - 1));
                samples.add(sample);
            }
            if (startsByGeometry.size() != EXPECTED_SAMPLES / 2 ||
                    startsByGeometry.values().stream().anyMatch(s -> !s.equals(Set.of("C", "D")))) {
                throw new IOException("Expected 50 geometries, each with both C and D start configurations.");
            }
            System.out.println("Dataset validation: PASS (100 samples, 50 matched C/D geometries).");

            Path parent = csv.getParent();
            if (parent != null) Files.createDirectories(parent);
            Path statusFile = csv.resolveSibling(csv.getFileName().toString().replaceFirst("(?i)\\.csv$", "_status.txt"));
            int incomplete = 0;
            int row = 0;
            try (BufferedWriter output = Files.newBufferedWriter(csv, StandardCharsets.UTF_8);
                 BufferedWriter status = Files.newBufferedWriter(statusFile, StandardCharsets.UTF_8)) {
                output.write(csvLine(CSV_COLUMNS));
                output.newLine();
                status.write("Dataset: " + dataset + "\nOutput: " + csv + "\n");
                status.write("Each Sample ID has M0 (lambda=0) and M1 (lambda=1) in one CSV row.\n");
                status.write("Steps = N/A for incomplete exploration.\n");
                status.write("Sample ID\tM0 termination\tM1 termination\tM0 simulation steps\tM1 simulation steps\n");

                for (Sample sample : samples) {
                    String id = sample.sampleId();
                    System.out.printf(Locale.ROOT, "[%03d/%03d] %-12s ", ++row, files.size(), id);
                    Trial m0 = evaluate(sample, 0.0);
                    Trial m1 = evaluate(sample, 1.0);
                    if (m0.mustExplored() != m1.mustExplored()) {
                        throw new IllegalStateException("Reachable-free count differs across λ: " + id);
                    }
                    List<String> cells = new ArrayList<>(CSV_COLUMNS.size());
                    cells.add(sample.environmentName());
                    cells.add(id);
                    cells.add(Integer.toString(m0.mustExplored()));
                    cells.addAll(m0.asCsvCells());
                    cells.addAll(m1.asCsvCells());
                    output.write(csvLine(cells));
                    output.newLine();
                    output.flush();  // Persist each completed sample pair.

                    status.write(id + "\t" + m0.termination() + "\t" + m1.termination()
                            + "\t" + m0.simulationSteps() + "\t" + m1.simulationSteps() + "\n");
                    status.flush();
                    if (!COMPLETE.equals(m0.termination())) incomplete++;
                    if (!COMPLETE.equals(m1.termination())) incomplete++;
                    System.out.printf(Locale.ROOT, "M0=%s (%s), M1=%s (%s)%n",
                            m0.stepsCell(), m0.termination(), m1.stepsCell(), m1.termination());
                }
                status.write("\nRuns: " + (samples.size() * 2) + ", incomplete: " + incomplete + "\n");
            }
            System.out.println("\nFINISHED: " + files.size() + " paired rows / " + (files.size() * 2) + " runs.");
            System.out.println("Incomplete runs: " + incomplete);
            System.out.println("Results CSV: " + csv);
            System.out.println("Termination log: " + statusFile);
        } catch (Exception ex) {
            System.err.println("TESTER STOPPED: " + ex.getMessage());
            ex.printStackTrace(System.err);
            System.exit(1);
        } finally {
            // Do not leave a modified lambda for a subsequent GUI execution in this JVM.
            if (originalLambda == null) System.clearProperty("swarm.lambda");
            else System.setProperty("swarm.lambda", originalLambda);
        }
    }

    private static void validateProtocol() {
        checkProperty("swarm.rho", "0.05");
        checkProperty("swarm.deposit", "1.0");
        checkProperty("swarm.recovery", "true");
        checkProperty("swarm.maxSteps", "2500");
        checkProperty("swarm.stallLimit", "30");
        if (!System.getProperty("swarm.trace", "").isBlank()
                || Integer.getInteger("swarm.traceLast", 0) != 0) {
            throw new IllegalArgumentException("Disable optional swarm trace settings for the main experiments.");
        }
    }

    private static void checkProperty(String key, String expected) {
        String value = System.getProperty(key, expected);
        if (!value.equals(expected)) {
            throw new IllegalArgumentException("Protocol parameter " + key + " is " + value
                    + ", expected " + expected + ". Remove the conflicting JVM -D option.");
        }
    }

    private static Path findDatasetDirectory() throws IOException {
        Path base = Paths.get(System.getProperty("user.dir")).toAbsolutePath().normalize();
        for (Path dir = base; dir != null; dir = dir.getParent()) {
            Path candidate = dir.resolve("src/main/java").resolve(PACKAGE_PATH).resolve("dataset");
            if (Files.isDirectory(candidate)) return candidate;
        }
        throw new IOException("Dataset not found under src/main/java/" + PACKAGE_PATH
                + "/dataset. Set the IntelliJ working directory to the project root "
                + "or provide the dataset directory as the first program argument.");
    }

    private static List<Path> collectSamples(Path dataset) throws IOException {
        List<Path> files;
        try (Stream<Path> stream = Files.walk(dataset)) {
            files = stream.filter(Files::isRegularFile)
                    .filter(p -> p.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".sc"))
                    .sorted(Comparator.comparing(p -> dataset.relativize(p).toString().replace('\\', '/')))
                    .collect(Collectors.toList());
        }
        if (files.size() != EXPECTED_SAMPLES) {
            throw new IOException("Expected exactly " + EXPECTED_SAMPLES + " .sc samples, found "
                    + files.size() + " in " + dataset);
        }
        Set<String> seen = new HashSet<>();
        int[] countByEnvironment = new int[5];
        for (Path file : files) {
            String name = file.getFileName().toString();
            if (!name.matches("S0[1-5]_G(0[1-9]|10)_[CD]\\.sc")) {
                throw new IOException("Unexpected sample filename: " + file);
            }
            String id = name.substring(0, name.length() - 3);
            if (!seen.add(id)) throw new IOException("Duplicate Sample ID: " + id);
            int index = Integer.parseInt(id.substring(1, 3)) - 1;
            String expectedFolder = switch (index) {
                case 0 -> "S01_Open_Space";
                case 1 -> "S02_Partitioned_Rooms";
                case 2 -> "S03_Branching_Corridors";
                case 3 -> "S04_Serpentine";
                case 4 -> "S05_Bottleneck";
                default -> throw new IllegalStateException("Unexpected environment number");
            };
            if (file.getParent() == null ||
                    !file.getParent().getFileName().toString().equals(expectedFolder)) {
                throw new IOException("Sample is not in the expected topology directory: " + file);
            }
            countByEnvironment[index]++;
        }
        for (int i = 0; i < countByEnvironment.length; i++) {
            if (countByEnvironment[i] != 20) {
                throw new IOException("Topology S0" + (i + 1) + " has " + countByEnvironment[i]
                        + " samples, expected 20.");
            }
        }
        return files;
    }

    /** Mirrors CanvasView.readFromFile()'s .sc parsing rules; no JavaFX is started. */
    private static Sample loadSample(Path file) throws IOException {
        List<String> lines = Files.readAllLines(file, StandardCharsets.UTF_8);
        if (lines.isEmpty()) throw new IOException("Empty file: " + file);
        int index = 0;
        if (lines.get(0).startsWith("ENVIRONMENT")) index++;
        int rows = headerInt(lines, index++, "NUM ROWS", file);
        int cols = headerInt(lines, index++, "NUM COLS", file);
        if (rows <= 0 || cols <= 0 || rows > MAX_ROWS || cols > MAX_COLS) {
            throw new IOException("Invalid grid dimensions in " + file + ": " + rows + " x " + cols);
        }
        int[][] environment = new int[rows][cols];
        for (int r = 0; r < rows; r++) {
            if (index >= lines.size()) throw new IOException("Missing row " + r + " in " + file);
            String[] parts = lines.get(index++).trim().split(":");
            if (parts.length != cols) throw new IOException("Incorrect column count at row " + r + " in " + file);
            for (int c = 0; c < cols; c++) {
                try {
                    int cell = Integer.parseInt(parts[c].trim());
                    if (cell != 0 && cell != -1)
                        throw new IOException("Invalid occupancy at (" + r + "," + c + ") in " + file);
                    environment[r][c] = cell;
                } catch (NumberFormatException ex) {
                    throw new IOException("Invalid number at (" + r + "," + c + ") in " + file, ex);
                }
            }
        }
        if (index < lines.size() && lines.get(index).startsWith("DRONES")) index++;
        int droneCount = headerInt(lines, index++, "NUM DRONES", file);
        if (droneCount != EXPECTED_DRONES) throw new IOException("Expected 4 drones in " + file);
        ArrayList<Point> starts = new ArrayList<>();
        Set<Integer> distinct = new HashSet<>();
        for (int i = 0; i < droneCount; i++) {
            if (index >= lines.size()) throw new IOException("Missing drone position D" + (i + 1) + " in " + file);
            String[] parts = lines.get(index++).trim().split(":");
            if (parts.length < 2) throw new IOException("Invalid drone position D" + (i + 1) + " in " + file);
            int r, c;
            try {
                r = Integer.parseInt(parts[0].trim());
                c = Integer.parseInt(parts[1].trim());
            } catch (NumberFormatException ex) {
                throw new IOException("Invalid drone coordinates in " + file, ex);
            }
            if (r < 0 || r >= rows || c < 0 || c >= cols || environment[r][c] == -1
                    || !distinct.add(r * cols + c)) {
                throw new IOException("Blocked, out-of-bounds, or duplicate start at D" + (i + 1)
                        + " in " + file);
            }
            starts.add(new Point(r, c));
        }
        if (index != lines.size()) {
            // Reject unknown trailing content rather than silently testing a malformed file.
            while (index < lines.size() && lines.get(index).isBlank()) index++;
            if (index != lines.size()) throw new IOException("Unexpected trailing content in " + file);
        }
        String sampleId = file.getFileName().toString().replaceFirst("(?i)\\.sc$", "");
        return new Sample(file.getParent().getFileName().toString(), sampleId, environment, starts);
    }

    private static int headerInt(List<String> lines, int index, String key, Path file) throws IOException {
        if (index >= lines.size()) throw new IOException("Missing " + key + " in " + file);
        String[] parts = lines.get(index).trim().split(":");
        if (parts.length < 2 || !parts[0].trim().equals(key))
            throw new IOException("Expected " + key + " at line " + (index + 1) + " in " + file);
        try {
            return Integer.parseInt(parts[1].trim());
        } catch (NumberFormatException ex) {
            throw new IOException("Invalid " + key + " in " + file, ex);
        }
    }

    private static Trial evaluate(Sample sample, double lambda) {
        // Instance fields in DecentralizedExploration read this property on construction.
        System.setProperty("swarm.lambda", Double.toString(lambda));
        ArrayList<Drone> drones = new ArrayList<>(EXPECTED_DRONES);
        for (int i = 0; i < sample.starts().size(); i++) {
            Point p = sample.starts().get(i);
            Drone drone = new Drone(sample.environment(), p.row(), p.col(), SENSOR_RANGE_CELLS, CELL_SIZE);
            drone.setName("D" + (i + 1));
            drones.add(drone);
        }
        DecentralizedExploration result = new DecentralizedExploration(sample.environment(), drones);

        boolean[][] reachable = result.getMustExploreMask();
        int[][] firstObservedAt = result.getFirstObservedAt();
        int must = 0, explored = 0;
        for (int r = 0; r < reachable.length; r++) {
            for (int c = 0; c < reachable[r].length; c++) {
                if (!reachable[r][c]) continue;
                must++;
                if (firstObservedAt[r][c] >= 0) explored++;
            }
        }

        // EXACT MainView.buildSimulationReport() physical-visit counting semantics:
        // initial position is a visit, consecutive stationary replay frames are not.
        int cols = sample.environment()[0].length;
        Map<Integer, Integer> cellVisits = new HashMap<>();
        Map<Integer, BitSet> distinctDrones = new HashMap<>();
        for (int i = 0; i < drones.size(); i++) {
            Point previous = null;
            for (Point point : drones.get(i).path) {
                if (point.equals(previous)) continue;
                int key = point.row() * cols + point.col();
                cellVisits.merge(key, 1, Integer::sum);
                distinctDrones.computeIfAbsent(key, unused -> new BitSet(EXPECTED_DRONES)).set(i);
                previous = point;
            }
        }
        int numRevisit = (int) cellVisits.values().stream().filter(n -> n > 1).count();
        int numOverlap = (int) distinctDrones.values().stream().filter(b -> b.cardinality() > 1).count();

        // Internal consistency checks; the primary percentage metrics come from
        // the SAME algorithm getters used by the GUI.
        double coverageCheck = must == 0 ? 100 : 100.0 * explored / must;
        if (Math.abs(coverageCheck - result.getObservedRatio()) > 1e-8)
            throw new IllegalStateException("Coverage mismatch in " + sample.sampleId() + " lambda=" + lambda);
        long totalVisits = cellVisits.values().stream().mapToLong(Integer::intValue).sum();
        int uniqueVisits = cellVisits.size();
        double revisitCheck = totalVisits == 0 ? 0 : 100.0 * (totalVisits - uniqueVisits) / totalVisits;
        double overlapCheck = uniqueVisits == 0 ? 0 : 100.0 * numOverlap / uniqueVisits;
        if (Math.abs(revisitCheck - result.getRevisit()) > 1e-8
                || Math.abs(overlapCheck - result.getOverlap()) > 1e-8) {
            throw new IllegalStateException("Revisit/overlap mismatch in " + sample.sampleId()
                    + " lambda=" + lambda);
        }
        return new Trial(must, explored, numRevisit, numOverlap,
                result.getObservedRatio(), result.getRevisit(), result.getOverlap(),
                result.getSteps(), result.getTerminationReason());
    }

    private static String csvLine(List<String> cells) {
        return cells.stream().map(Tester::escapeCsv).collect(Collectors.joining(delimiter));
    }

    private static String escapeCsv(String raw) {
        if (raw.contains(delimiter) || raw.contains("\"") || raw.contains("\n") || raw.contains("\r"))
            return "\"" + raw.replace("\"", "\"\"") + "\"";
        return raw;
    }

    private record Sample(String environmentName, String sampleId, int[][] environment, List<Point> starts) { }

    private record Trial(int mustExplored, int explored, int numRevisit, int numOverlap,
                         double coverage, double revisit, double overlap,
                         int simulationSteps, String termination) {
        String stepsCell() {
            return COMPLETE.equals(termination) ? Integer.toString(simulationSteps) : "N/A";
        }

        List<String> asCsvCells() {
            return List.of(Integer.toString(explored), Integer.toString(numRevisit),
                    Integer.toString(numOverlap), fmt(coverage), fmt(revisit), fmt(overlap), stepsCell());
        }

        private static String fmt(double number) {
            return String.format(Locale.ROOT, "%.2f", number);
        }
    }
}
