package drone.decentralizedswarmcoverage_fbe_rps;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.geom.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.security.MessageDigest;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;
import java.util.List;
import java.util.function.Consumer;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

/**
 * ResearchResultsGenerator — ONE source class, Java SE only (no Maven dependencies).
 * Creates Excel (.xlsx), CSV tables, PNG and SVG figures, and a linked README.md
 * from an existing paired ExplorationResults CSV. Does NOT invoke/change the simulator.
 *
 * Project location: src/main/java/drone/decentralizedswarmcoverage_fbe_rps/
 * IntelliJ: Run ResearchResultsGenerator.main() with project root working directory.
 * Input:  <project-root>/results/ExplorationResults<14-digit-code>.csv
 * Output: <project-root>/analysis/, with the same 14-digit code in every filename.
 * Optional program args: [input.csv or path] [output_directory]
 * This class uses Java2D, so named modules need `requires java.desktop;`.
 */
public final class ResearchResultsGenerator {
    // ===== USER SETTINGS =====
    // Change INPUT_CSV when you want to analyze another experiment batch.
    // Default path: <project root>/results/<INPUT_CSV>
    private static String INPUT_CSV = "ExplorationResults20261023105835.csv";
    // All generated files go into <project root>/analysis/, not the source tree.
    private static String OUTPUT_DIRECTORY = "analysis";
    private static String CSV_DELIMITER = ";";   // CSV exports; input is auto-detected
    private static final int BOOTSTRAP_REPETITIONS = 4000;
    private static final long BOOTSTRAP_SEED = 20261023L;
    // Set headless mode before java.awt.Color and other graphics classes are initialized.
    static { System.setProperty("java.awt.headless", "true"); }
    // ===== VISUAL THEME =====
    private static final Color TEAL = Color.decode("#007E6E");
    private static final Color ORANGE = Color.decode("#F79A19");
    private static final Color TEAL_LIGHT = Color.decode("#D7F0EC");
    private static final Color ORANGE_LIGHT = Color.decode("#FFF0D9");
    private static final Color NAVY = Color.decode("#17313D");
    private static final Color GREY = Color.decode("#8B9BA5");
    private static final Color GRID = Color.decode("#E7EEF0");
    private static final Color BG = Color.WHITE;

    private static final String[] TOPOS = {"Open Space", "Partitioned Rooms", "Branching Corridors", "Serpentine", "Bottleneck"};
    private static final String[] CODES = {"S01", "S02", "S03", "S04", "S05"};
    private static final String[] SOURCE_HEADERS = {
        "Environment", "Sample ID", "Must Explored", "LAMBDA_0 Explored", "LAMBDA_0 Num Revisit", "LAMBDA_0 Num Overlap",
        "LAMBDA_0 Coverage(%)", "LAMBDA_0 Revisit (%)", "LAMBDA_0 Overlap (%)", "LAMBDA_0 Steps",
        "LAMBDA_1 Explored", "LAMBDA_1 Num Revisit", "LAMBDA_1 Num Overlap", "LAMBDA_1 Coverage(%)",
        "LAMBDA_1 Revisit (%)", "LAMBDA_1 Overlap (%)", "LAMBDA_1 Steps"
    };
    private static final String[] METRIC_NAMES = {"Revisit Ratio", "Overlap Ratio", "Completion Steps"};
    private static final int[] METRIC_M0 = {7,8,9};
    private static final int[] METRIC_M1 = {14,15,16};

    static final class Sample {
        final String environment, id, topo, geom, start;
        final double[] v = new double[15]; // original cols 2..16
        Sample(List<String> row) {
            environment=row.get(0).trim(); id=row.get(1).trim();
            if (!id.matches("S0[1-5]_G\\d{2}_[CD]")) throw new IllegalArgumentException("Invalid Sample ID: " + id);
            topo=id.substring(0,3); geom=id.substring(0,7); start=id.substring(id.length()-1);
            for(int i=2;i<17;i++) v[i-2]=number(row.get(i));
        }
        double col(int zeroBasedOriginalColumn) {return v[zeroBasedOriginalColumn-2];}
        double savings(int m) {return col(METRIC_M0[m])-col(METRIC_M1[m]);}
        boolean complete(){return Double.isFinite(col(9))&&Double.isFinite(col(16));}
        String outcome(){double s=savings(2);return !Double.isFinite(s)?"INCOMPLETE":s>0?"M1 faster":s<0?"M0 faster":"Tie";}
    }
    static final class Sheet {
        final String name; final List<String> headers; final List<List<Object>> rows=new ArrayList<>();
        Sheet(String name, String... headers) {this.name=name;this.headers=List.of(headers);}
        Sheet add(Object... vals) {rows.add(Arrays.asList(vals));return this;}
    }
    static final class Interval {
        final double value,lo,hi;
        Interval(double a,double b,double c){value=a;lo=b;hi=c;}
    }
    private static List<Sample> samples;
    private static List<Sheet> sheets;
    private static Path output;
    private static Path projectRoot;
    private static String resultCode;
    private static String sourceSha;
    private static Map<String, Map<String, Interval>> topoCI;
    private static Map<String, Map<String, Interval>> startCI;

    private ResearchResultsGenerator(){}
    public static void main(String[] args) throws Exception {
        if (args.length > 2)
            throw new IllegalArgumentException("Usage: ResearchResultsGenerator [results CSV path] [output directory]");

        // Resolve paths from the IntelliJ/Maven project root rather than from the package directory.
        projectRoot = findProjectRoot();
        Path input = resolveInput(args.length >= 1 ? args[0] : INPUT_CSV);
        resultCode = extractResultCode(input);
        Path requestedOutput = Paths.get(args.length >= 2 ? args[1] : OUTPUT_DIRECTORY);
        output = (requestedOutput.isAbsolute() ? requestedOutput : projectRoot.resolve(requestedOutput))
                .toAbsolutePath().normalize();
        Files.createDirectories(output.resolve("figures"));
        Files.createDirectories(output.resolve("tables"));

        sourceSha=sha256(input);
        samples=load(input);
        validate(samples);
        sheets=new ArrayList<>();
        calculateConfidenceIntervals();
        buildTables();
        writeExcel(output.resolve(workbookFilename()),sheets);
        for(Sheet sheet:sheets)
            writeCsv(output.resolve("tables").resolve(tableFilename(sheet.name)),sheet);
        createFigures();
        makeReadme(input);

        System.out.println("Project : " + projectRoot);
        System.out.println("Input   : " + input);
        System.out.println("Batch ID: " + resultCode);
        System.out.println("SUCCESS: "+samples.size()+" paired samples; "+sheets.size()+" Excel sheets; 10 PNG + 10 SVG charts.");
        System.out.println("Output  : "+output);
        System.out.println("Excel   : "+output.resolve(workbookFilename()));
        System.out.println("README  : "+output.resolve(readmeFilename()));
    }

    private static Path findProjectRoot() throws IOException {
        Path cwd = Paths.get(System.getProperty("user.dir")).toAbsolutePath().normalize();
        for (Path dir = cwd; dir != null; dir = dir.getParent()) {
            if (Files.isRegularFile(dir.resolve("pom.xml"))
                    && Files.isDirectory(dir.resolve("src/main/java"))) return dir;
            if (Files.isDirectory(dir.resolve("src/main/java"))
                    && Files.isDirectory(dir.resolve("results"))) return dir;
        }
        throw new IOException("Project root not found. In IntelliJ set Run > Edit Configurations "
                + "> Working directory to $PROJECT_DIR$. Current: " + cwd);
    }

    private static Path resolveInput(String nameOrPath) throws IOException {
        Path given = Paths.get(nameOrPath);
        if (given.isAbsolute() && Files.isRegularFile(given)) return given.normalize();
        Path resultFile = projectRoot.resolve("results").resolve(given).normalize();
        if (Files.isRegularFile(resultFile)) return resultFile;
        Path relativeToRoot = projectRoot.resolve(given).normalize();
        if (Files.isRegularFile(relativeToRoot)) return relativeToRoot;
        throw new FileNotFoundException("Experiment CSV not found: " + nameOrPath
                + ". Expected in " + projectRoot.resolve("results")
                + ". Update INPUT_CSV or provide an absolute path.");
    }

    private static String extractResultCode(Path input) {
        String name = input.getFileName().toString();
        java.util.regex.Matcher matcher = java.util.regex.Pattern
                .compile("(\\d{14})(?:\\s*\\(\\d+\\))?\\.csv$", java.util.regex.Pattern.CASE_INSENSITIVE)
                .matcher(name);
        if (!matcher.find())
            throw new IllegalArgumentException("Input filename must end with a 14-digit result code, "
                    + "e.g. ExplorationResults20261023105835.csv: " + name);
        return matcher.group(1);
    }

    private static String workbookFilename() { return "Research_Tables_" + resultCode + ".xlsx"; }
    private static String readmeFilename() { return "README_" + resultCode + ".md"; }
    private static String tableFilename(String base) { return base + "_" + resultCode + ".csv"; }
    private static String figureFilename(String base, String extension) {
        return base + "_" + resultCode + "." + extension;
    }

    private static String sha256(Path p)throws Exception{
        MessageDigest md=MessageDigest.getInstance("SHA-256");
        try(InputStream in=Files.newInputStream(p)){byte[] b=new byte[8192];int n;while((n=in.read(b))!=-1)md.update(b,0,n);}
        StringBuilder sb=new StringBuilder(); for(byte b:md.digest())sb.append(String.format(Locale.ROOT,"%02x",b&255));return sb.toString();
    }
    private static double number(String raw){String x=raw.trim();if(x.isEmpty()||x.equalsIgnoreCase("N/A"))return Double.NaN;return Double.parseDouble(x);}
    private static List<Sample> load(Path file)throws Exception{
        String content=Files.readString(file,StandardCharsets.UTF_8).replace("\uFEFF", "");
        String header=content.split("\\R",2)[0];
        char delim=header.indexOf(';')>=0?';':',';
        List<List<String>> data=parseDelimited(content,delim);
        if(data.isEmpty()||data.get(0).size()!=17)throw new IOException("Expected 17 CSV input columns");
        // By positional schema; M0/M1 column names accept LAMBDA_0 / M0 / lambda variants.
        if(!data.get(0).get(0).trim().equalsIgnoreCase("Environment")||
            !data.get(0).get(1).trim().equalsIgnoreCase("Sample ID")||
            !data.get(0).get(9).toLowerCase(Locale.ROOT).contains("steps")||
            !data.get(0).get(16).toLowerCase(Locale.ROOT).contains("steps"))
            throw new IOException("Unexpected CSV schema; expected Environment, Sample ID and M0/M1 Steps columns");
        List<Sample> result=new ArrayList<>();
        for(int i=1;i<data.size();i++){
            if(data.get(i).size()==1&&data.get(i).get(0).isBlank())continue;
            if(data.get(i).size()!=17)throw new IOException("CSV line "+(i+1)+" contains "+data.get(i).size()+" cells (expected 17)");
            result.add(new Sample(data.get(i)));
        }
        return result;
    }
    private static List<List<String>> parseDelimited(String raw,char delimiter)throws IOException{
        List<List<String>> rows=new ArrayList<>();List<String> row=new ArrayList<>();StringBuilder cell=new StringBuilder();boolean quoted=false;
        for(int i=0;i<raw.length();i++){
            char c=raw.charAt(i);
            if(c=='"'){if(quoted&&i+1<raw.length()&&raw.charAt(i+1)=='"'){cell.append('"');i++;}else quoted=!quoted;}
            else if(c==delimiter&&!quoted){row.add(cell.toString());cell.setLength(0);}
            else if((c=='\r'||c=='\n')&&!quoted){if(c=='\r'&&i+1<raw.length()&&raw.charAt(i+1)=='\n')i++;
                row.add(cell.toString());rows.add(row);row=new ArrayList<>();cell.setLength(0);}
            else cell.append(c);
        }
        if(quoted)throw new IOException("Unbalanced CSV quotes");
        if(cell.length()>0||!row.isEmpty()){row.add(cell.toString());rows.add(row);}return rows;
    }
    private static void validate(List<Sample> all){
        if(all.isEmpty())throw new IllegalArgumentException("CSV has no data rows");
        HashSet<String> ids=new HashSet<>();
        for(Sample s:all){if(!ids.add(s.id))throw new IllegalArgumentException("Duplicate Sample ID: "+s.id);
            if(!s.environment.startsWith(s.topo))throw new IllegalArgumentException("Environment/ID mismatch: "+s.id);
            if(s.col(2)<=0)throw new IllegalArgumentException("Invalid Must Explored: "+s.id);
            if(s.col(2)!=s.col(2))throw new IllegalArgumentException("Invalid Must Explored: "+s.id);
        }
        all.sort(Comparator.comparing(a->a.id));
        System.out.println("Input: "+all.size()+" distinct paired samples. Complete pairs: "+all.stream().filter(Sample::complete).count());
    }
    private static String topoName(String code){for(int i=0;i<CODES.length;i++)if(code.equals(CODES[i]))return TOPOS[i];return code;}
    private static List<Sample> byTopo(String code){List<Sample> a=new ArrayList<>();for(Sample s:samples)if(s.topo.equals(code))a.add(s);return a;}
    private static List<Sample> byStart(String start){List<Sample> a=new ArrayList<>();for(Sample s:samples)if(s.start.equals(start))a.add(s);return a;}
    private static double mean(Collection<Double> x){double sum=0;int n=0;for(double v:x)if(Double.isFinite(v)){sum+=v;n++;}return n==0?Double.NaN:sum/n;}
    private static double meanMetric(List<Sample> data,int m,boolean m1){List<Double> x=new ArrayList<>();for(Sample s:data)x.add(s.col((m1?METRIC_M1:METRIC_M0)[m]));return mean(x);}
    private static double meanSaving(List<Sample> data,int m){List<Double>x=new ArrayList<>();for(Sample s:data)x.add(s.savings(m));return mean(x);}
    private static double quantile(Collection<Double> vals,double q){List<Double>x=new ArrayList<>();for(double a:vals)if(Double.isFinite(a))x.add(a);Collections.sort(x);if(x.isEmpty())return Double.NaN;
        double p=(x.size()-1)*q;int lo=(int)Math.floor(p),hi=(int)Math.ceil(p);return x.get(lo)+(x.get(hi)-x.get(lo))*(p-lo);}
    private static List<Double> saved(List<Sample> data,int metric){List<Double>x=new ArrayList<>();for(Sample s:data)x.add(s.savings(metric));return x;}
    private static List<String> geometryIds(List<Sample> data){TreeSet<String> ids=new TreeSet<>();for(Sample s:data)ids.add(s.geom);return new ArrayList<>(ids);}
    private static double geometricSaving(List<Sample> data,String geometry,int metric,String start){
        List<Double>x=new ArrayList<>();for(Sample s:data)if(s.geom.equals(geometry)&&(start==null||s.start.equals(start)))x.add(s.savings(metric));return mean(x);}
    private static Interval bootstrap(List<Sample> data,int metric,String start,long salt){
        List<String> geoms=geometryIds(data);List<Double> clusterMeans=new ArrayList<>();
        for(String g:geoms){double v=geometricSaving(data,g,metric,start);if(Double.isFinite(v))clusterMeans.add(v);}
        if(clusterMeans.isEmpty())return new Interval(Double.NaN,Double.NaN,Double.NaN);
        double v=mean(clusterMeans); Random rng=new Random(BOOTSTRAP_SEED+salt);
        List<Double> boot=new ArrayList<>(BOOTSTRAP_REPETITIONS);
        for(int b=0;b<BOOTSTRAP_REPETITIONS;b++){
            double sum=0;for(int i=0;i<clusterMeans.size();i++)sum+=clusterMeans.get(rng.nextInt(clusterMeans.size()));
            boot.add(sum/clusterMeans.size());
        }
        return new Interval(v,quantile(boot,.025),quantile(boot,.975));
    }
    private static void calculateConfidenceIntervals(){
        topoCI=new LinkedHashMap<>();startCI=new LinkedHashMap<>();
        for(int t=0;t<5;t++){
            String c=CODES[t]; Map<String,Interval> row=new HashMap<>(); Map<String,Interval> byStart=new HashMap<>();
            for(int m=0;m<3;m++){
                row.put("metric"+m,bootstrap(byTopo(c),m,null,t*41L+m));
                byStart.put("C"+m,bootstrap(byTopo(c),m,"C",t*421L+m));
                byStart.put("D"+m,bootstrap(byTopo(c),m,"D",t*421L+m+9));
            }
            topoCI.put(c,row);startCI.put(c,byStart);
        }
    }
    private static double zeroIfNaN(double x){return Double.isFinite(x)?x:0;}
    private static double relativeReduction(double m0,double m1){return !Double.isFinite(m0)||m0==0?Double.NaN:100*(m0-m1)/m0;}
    private static double r2(double x){return Double.isFinite(x)?Math.round(x*100.0)/100.0:Double.NaN;}
    private static int countOutcome(List<Sample> data,String label){int n=0;for(Sample s:data)if(s.outcome().equals(label))n++;return n;}
    private static Sheet register(Sheet s){sheets.add(s);return s;}
    private static void buildTables(){
        // Ordered around the matching figures rather than alphabetical.
        Sheet methods=register(new Sheet("01_Method_Means","Topology","Samples","M0 Coverage %","M1 Coverage %","M0 Revisit %","M1 Revisit %",
                "M0 Overlap %","M1 Overlap %","M0 Steps","M1 Steps","Revisit reduced pp","Overlap reduced pp","Steps saved","Steps reduction %"));
        for(int t=0;t<=5;t++){
            List<Sample>a=t==5?samples:byTopo(CODES[t]);String label=t==5?"Overall":TOPOS[t];
            double a0=meanMetric(a,2,false),a1=meanMetric(a,2,true);
            methods.add(label,a.size(),r2(meanColumn(a,6)),r2(meanColumn(a,13)),r2(meanMetric(a,0,false)),r2(meanMetric(a,0,true)),
                r2(meanMetric(a,1,false)),r2(meanMetric(a,1,true)),r2(a0),r2(a1),r2(meanSaving(a,0)),r2(meanSaving(a,1)),
                r2(meanSaving(a,2)),r2(relativeReduction(a0,a1)));
        }
        Sheet placing=register(new Sheet("02_Starting_Placement","Topology","Start","Samples","M0 Steps","M1 Steps","Steps saved","95% CI low","95% CI high",
                "Revisit reduced pp","Overlap reduced pp","M1 faster","M0 faster","Tie"));
        for(int t=0;t<5;t++)for(String st:List.of("C","D")){
            List<Sample> a=new ArrayList<>();for(Sample s:byTopo(CODES[t]))if(s.start.equals(st))a.add(s);
            Interval ci=startCI.get(CODES[t]).get(st+"2");
            placing.add(TOPOS[t],st,a.size(),r2(meanMetric(a,2,false)),r2(meanMetric(a,2,true)),r2(meanSaving(a,2)),
                r2(ci.lo),r2(ci.hi),r2(meanSaving(a,0)),r2(meanSaving(a,1)),countOutcome(a,"M1 faster"),countOutcome(a,"M0 faster"),countOutcome(a,"Tie"));
        }
        Sheet outcomes=register(new Sheet("03_Outcome_By_Topology","Topology","Samples","M1 faster","M0 faster","Tie","M1 faster %","M0 faster %","Tie %"));
        for(int t=0;t<5;t++){
            List<Sample>a=byTopo(CODES[t]);int n=a.size(),w=countOutcome(a,"M1 faster"),l=countOutcome(a,"M0 faster"),tie=countOutcome(a,"Tie");
            int completed=w+l+tie;
            outcomes.add(TOPOS[t],n,w,l,tie,completed==0?"N/A":r2(100.0*w/completed),
                completed==0?"N/A":r2(100.0*l/completed),completed==0?"N/A":r2(100.0*tie/completed));
        }
        Sheet consistency=register(new Sheet("04_Improvement_Consistency","Topology","Samples","M1 faster","Tie","M1 slower","Mean saved steps",
                "Median saved steps","IQR Q1","IQR Q3","Worst saved steps","Best saved steps","Mean revisit reduction pp","Mean overlap reduction pp"));
        for(int t=0;t<=5;t++){
            List<Sample>a=t==5?samples:byTopo(CODES[t]);List<Double>x=saved(a,2);
            consistency.add(t==5?"Overall":TOPOS[t],a.size(),countOutcome(a,"M1 faster"),countOutcome(a,"Tie"),countOutcome(a,"M0 faster"),
                r2(mean(x)),r2(quantile(x,.5)),r2(quantile(x,.25)),r2(quantile(x,.75)),r2(quantile(x,0)),r2(quantile(x,1)),
                r2(meanSaving(a,0)),r2(meanSaving(a,1)));
        }
        Sheet ci=register(new Sheet("05_Topology_Bootstrap_CI","Topology","Revisit reduction mean pp","Revisit CI low","Revisit CI high",
                "Overlap reduction mean pp","Overlap CI low","Overlap CI high","Saved steps mean","Saved steps CI low","Saved steps CI high"));
        for(int t=0;t<5;t++){
            Map<String,Interval> z=topoCI.get(CODES[t]);Interval a=z.get("metric0"),b=z.get("metric1"),c=z.get("metric2");
            ci.add(TOPOS[t],r2(a.value),r2(a.lo),r2(a.hi),r2(b.value),r2(b.lo),r2(b.hi),r2(c.value),r2(c.lo),r2(c.hi));
        }
        Sheet joint=register(new Sheet("06_Overlap_Steps_Joint","Steps outcome","Overlap reduced","Overlap increased","Overlap equal","Total"));
        for(String label:List.of("M1 faster","Tie","M0 faster")){
            int reduced=0,increased=0,equal=0;
            for(Sample s:samples)if(s.outcome().equals(label)){
                double d=s.savings(1);if(d>1e-9)reduced++;else if(d< -1e-9)increased++;else equal++;
            }
            joint.add(label,reduced,increased,equal,reduced+increased+equal);
        }
        int red=0,inc=0,eq=0;for(Sample s:samples){double v=s.savings(1);if(v>1e-9)red++;else if(v< -1e-9)inc++;else eq++;}
        joint.add("TOTAL",red,inc,eq,red+inc+eq);
        Sheet slower=register(new Sheet("07_M1_Slower_Cases","Environment","Sample ID","Must Explored","M0 Steps","M1 Steps","Saved steps M0-M1",
                "Revisit reduction pp","Overlap reduction pp"));
        List<Sample> losing=new ArrayList<>();for(Sample s:samples)if("M0 faster".equals(s.outcome()))losing.add(s);
        losing.sort(Comparator.comparingDouble((Sample s)->s.savings(2)));
        for(Sample s:losing)slower.add(s.environment,s.id,(int)s.col(2),(int)s.col(9),(int)s.col(16),(int)s.savings(2),r2(s.savings(0)),r2(s.savings(1)));
        Sheet paired=register(new Sheet("08_Paired_100", "Environment","Sample ID","Topology ID","Geometry ID","Start","Must Explored",
            "Saved Revisit Ratio pp","Saved Overlap Ratio pp","Saved Completion Steps","M0 Steps","M1 Steps","Outcome"));
        for(Sample s:samples)paired.add(s.environment,s.id,s.topo,s.geom,s.start,(int)s.col(2),r2(s.savings(0)),r2(s.savings(1)),
                numOrNA(s.savings(2)),numOrNA(s.col(9)),numOrNA(s.col(16)),s.outcome());
        Sheet raw=register(new Sheet("09_Source_and_Formulas",concat(SOURCE_HEADERS,new String[]{"Saved steps = M0-M1",
            "Saved revisit pp = M0-M1","Saved overlap pp = M0-M1"})));
        for(Sample s:samples){List<Object> r=new ArrayList<>();r.add(s.environment);r.add(s.id);
            for(int i=2;i<17;i++)r.add(numOrNA(s.col(i)));
            // Generate actual Excel formulas in the last three columns when exporting workbook; CSV stores evaluated numeric values.
            r.add(numOrNA(s.savings(2)));r.add(numOrNA(s.savings(0)));r.add(numOrNA(s.savings(1)));
            raw.rows.add(r);
        }
    }
    private static Object numOrNA(double v){if(!Double.isFinite(v))return "N/A";return Math.rint(v)==v?(int)v:r2(v);}
    private static String[] concat(String[] a,String[] b){String[] c=Arrays.copyOf(a,a.length+b.length);System.arraycopy(b,0,c,a.length,b.length);return c;}
    private static double meanColumn(List<Sample>a,int idx){List<Double>x=new ArrayList<>();for(Sample s:a)x.add(s.col(idx));return mean(x);}
    private static void writeCsv(Path p,Sheet sheet)throws IOException{
        try(BufferedWriter w=Files.newBufferedWriter(p,StandardCharsets.UTF_8)){
            w.write(csvRow(sheet.headers));w.newLine();for(List<Object>row:sheet.rows){w.write(csvRow(row));w.newLine();}
        }
    }
    private static String csvRow(List<?> x){List<String>out=new ArrayList<>();for(Object o:x){String v=String.valueOf(o);if(v.contains(CSV_DELIMITER)||v.contains("\"")||v.contains("\n"))v="\""+v.replace("\"","\"\"")+"\"";out.add(v);}return String.join(CSV_DELIMITER,out);}
    // ------------------------------------------------------------------------
    // Minimal standards-compliant XLSX writer. No Apache POI / Maven dependency.
    // ------------------------------------------------------------------------
    private static void writeExcel(Path file,List<Sheet> sheets)throws IOException{
        try(ZipOutputStream zip=new ZipOutputStream(Files.newOutputStream(file),StandardCharsets.UTF_8)){
            StringBuilder types=new StringBuilder("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?><Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\"><Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/><Default Extension=\"xml\" ContentType=\"application/xml\"/><Override PartName=\"/xl/workbook.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml\"/><Override PartName=\"/xl/styles.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml\"/>");
            for(int i=1;i<=sheets.size();i++)types.append("<Override PartName=\"/xl/worksheets/sheet").append(i).append(".xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml\"/>");
            types.append("</Types>");put(zip,"[Content_Types].xml",types.toString());
            put(zip,"_rels/.rels","<?xml version=\"1.0\" encoding=\"UTF-8\"?><Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\"><Relationship Id=\"rId1\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument\" Target=\"xl/workbook.xml\"/></Relationships>");
            StringBuilder book=new StringBuilder("<?xml version=\"1.0\" encoding=\"UTF-8\"?><workbook xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\" xmlns:r=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships\"><sheets>");
            StringBuilder rels=new StringBuilder("<?xml version=\"1.0\" encoding=\"UTF-8\"?><Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">");
            for(int i=1;i<=sheets.size();i++){
                book.append("<sheet name=\"").append(xml(sheets.get(i-1).name)).append("\" sheetId=\"").append(i).append("\" r:id=\"rId").append(i).append("\"/>");
                rels.append("<Relationship Id=\"rId").append(i).append("\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet\" Target=\"worksheets/sheet").append(i).append(".xml\"/>");
            }
            book.append("</sheets></workbook>");rels.append("<Relationship Id=\"rId").append(sheets.size()+1).append("\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles\" Target=\"styles.xml\"/></Relationships>");
            put(zip,"xl/workbook.xml",book.toString());put(zip,"xl/_rels/workbook.xml.rels",rels.toString());
            put(zip,"xl/styles.xml",stylesXml());
            for(int i=0;i<sheets.size();i++)put(zip,"xl/worksheets/sheet"+(i+1)+".xml",sheetXml(sheets.get(i)));
        }
    }
    private static String stylesXml(){return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"+
        "<styleSheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\">"+
        "<fonts count=\"4\"><font><sz val=\"10\"/><name val=\"Aptos\"/><color rgb=\"FF17313D\"/></font>"+
        "<font><b/><sz val=\"10\"/><name val=\"Aptos\"/><color rgb=\"FFFFFFFF\"/></font>"+
        "<font><b/><sz val=\"10\"/><color rgb=\"FF17313D\"/></font>"+
        "<font><b/><sz val=\"10\"/><color rgb=\"FF007E6E\"/></font></fonts>"+
        "<fills count=\"4\"><fill><patternFill patternType=\"none\"/></fill><fill><patternFill patternType=\"gray125\"/></fill>"+
        "<fill><patternFill patternType=\"solid\"><fgColor rgb=\"FF007E6E\"/><bgColor indexed=\"64\"/></patternFill></fill>"+
        "<fill><patternFill patternType=\"solid\"><fgColor rgb=\"FFFFF0D9\"/><bgColor indexed=\"64\"/></patternFill></fill></fills>"+
        "<borders count=\"1\"><border><left/><right/><top/><bottom/><diagonal/></border></borders>"+
        "<cellStyleXfs count=\"1\"><xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"0\"/></cellStyleXfs>"+
        "<cellXfs count=\"4\"><xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\"/>"+
        "<xf numFmtId=\"0\" fontId=\"1\" fillId=\"2\" borderId=\"0\" xfId=\"0\" applyFill=\"1\" applyFont=\"1\"/>"+
        "<xf numFmtId=\"0\" fontId=\"2\" fillId=\"3\" borderId=\"0\" xfId=\"0\" applyFill=\"1\" applyFont=\"1\"/>"+
        "<xf numFmtId=\"2\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\" applyNumberFormat=\"1\"/></cellXfs>"+
        "<cellStyles count=\"1\"><cellStyle name=\"Normal\" xfId=\"0\" builtinId=\"0\"/></cellStyles></styleSheet>";
    }
    private static String sheetXml(Sheet s){
        int ncols=s.headers.size(),nrows=s.rows.size()+1;
        StringBuilder b=new StringBuilder("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"+
          "<worksheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\"><dimension ref=\"A1:"+colName(ncols-1)+nrows+"\"/>"+
          "<sheetViews><sheetView workbookViewId=\"0\"><pane ySplit=\"1\" topLeftCell=\"A2\" activePane=\"bottomLeft\" state=\"frozen\"/></sheetView></sheetViews><cols>");
        for(int j=0;j<ncols;j++){
            int max=s.headers.get(j).length();for(List<Object>r:s.rows)if(j<r.size())max=Math.max(max,String.valueOf(r.get(j)).length());
            int width=Math.min(36,Math.max(13,max+3));
            b.append("<col min=\"").append(j+1).append("\" max=\"").append(j+1).append("\" width=\"").append(width).append("\" customWidth=\"1\"/>");
        }
        b.append("</cols><sheetData><row r=\"1\" ht=\"29\" customHeight=\"1\">");
        for(int j=0;j<ncols;j++)cell(b,1,j,s.headers.get(j),1,null);
        b.append("</row>");
        for(int k=0;k<s.rows.size();k++){
            int row=k+2;b.append("<row r=\"").append(row).append("\">");
            List<Object>vals=s.rows.get(k);int style= ("Overall".equals(String.valueOf(vals.get(0))) || "TOTAL".equals(String.valueOf(vals.get(0))))?2:0;
            for(int j=0;j<vals.size();j++){
                String formula=null;
                if(s.name.equals("09_Source_and_Formulas")&&j>=17&&j<=19){
                    if(j==17) formula="IF(AND(ISNUMBER(J"+row+"),ISNUMBER(Q"+row+")),J"+row+"-Q"+row+",\"\")";
                    if(j==18) formula="H"+row+"-O"+row;
                    if(j==19) formula="I"+row+"-P"+row;
                }
                cell(b,row,j,vals.get(j),style,formula);
            }
            b.append("</row>");
        }
        b.append("</sheetData><autoFilter ref=\"A1:").append(colName(ncols-1)).append(nrows).append("\"/>");
        b.append("</worksheet>");return b.toString();
    }
    private static void cell(StringBuilder b,int r,int c,Object value,int style,String formula){
        String ref=colName(c)+r;
        if(value instanceof Number n && Double.isFinite(n.doubleValue())){
            b.append("<c r=\"").append(ref).append("\" s=\"").append(style==0?3:style).append("\">");
            if(formula!=null)b.append("<f>").append(xml(formula)).append("</f>");
            b.append("<v>").append(n).append("</v></c>");
        }else{
            b.append("<c r=\"").append(ref).append("\" s=\"").append(style).append("\" t=\"inlineStr\"><is><t>")
                .append(xml(String.valueOf(value))).append("</t></is></c>");
        }
    }
    private static String colName(int i){StringBuilder b=new StringBuilder();while(i>=0){b.insert(0,(char)('A'+i%26));i=i/26-1;}return b.toString();}
    private static String xml(String s){return s.replace("&","&amp;").replace("<","&lt;").replace(">","&gt;").replace("\"","&quot;").replace("'","&apos;");}
    private static void put(ZipOutputStream z,String name,String text)throws IOException{z.putNextEntry(new ZipEntry(name));z.write(text.getBytes(StandardCharsets.UTF_8));z.closeEntry();}
    // ------------------------------------------------------------------------
    // Chart renderer: one drawing command writes to BOTH PNG (Java2D) and SVG.
    // ------------------------------------------------------------------------
    private static final class Drawing implements AutoCloseable {
        final int width,height; final BufferedImage image; final Graphics2D g; final StringBuilder svg;
        Drawing(int w,int h){width=w;height=h;image=new BufferedImage(w,h,BufferedImage.TYPE_INT_RGB);
            g=image.createGraphics();g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);
            g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
            g.setColor(BG);g.fillRect(0,0,w,h);
            svg=new StringBuilder("<?xml version=\"1.0\" encoding=\"UTF-8\"?><svg xmlns=\"http://www.w3.org/2000/svg\" width=\""+w+"\" height=\""+h+"\" viewBox=\"0 0 "+w+" "+h+"\"><rect width=\"100%\" height=\"100%\" fill=\"white\"/>");
        }
        String hex(Color c){return String.format(Locale.ROOT,"#%02X%02X%02X",c.getRed(),c.getGreen(),c.getBlue());}
        void rect(double x,double y,double w,double h,Color fill){if(fill==null)return;
            g.setColor(fill);g.fill(new Rectangle2D.Double(x,y,w,h));
            svg.append("<rect x=\"").append(f(x)).append("\" y=\"").append(f(y)).append("\" width=\"").append(f(w)).append("\" height=\"").append(f(h)).append("\" fill=\"").append(hex(fill)).append("\"/>");}
        void line(double x1,double y1,double x2,double y2,Color color,double sw){
            g.setColor(color);g.setStroke(new BasicStroke((float)sw,BasicStroke.CAP_ROUND,BasicStroke.JOIN_ROUND));g.draw(new Line2D.Double(x1,y1,x2,y2));
            svg.append("<line x1=\"").append(f(x1)).append("\" y1=\"").append(f(y1)).append("\" x2=\"").append(f(x2)).append("\" y2=\"").append(f(y2))
                .append("\" stroke=\"").append(hex(color)).append("\" stroke-width=\"").append(f(sw)).append("\" stroke-linecap=\"round\"/>");}
        void circle(double x,double y,double radius,Color fill){
            g.setColor(fill);g.fill(new Ellipse2D.Double(x-radius,y-radius,2*radius,2*radius));
            svg.append("<circle cx=\"").append(f(x)).append("\" cy=\"").append(f(y)).append("\" r=\"").append(f(radius)).append("\" fill=\"").append(hex(fill)).append("\"/>");}
        void text(String msg,double x,double y,int size,Color color,boolean bold,String anchor){
            g.setFont(new Font("Arial",bold?Font.BOLD:Font.PLAIN,size));g.setColor(color);
            FontMetrics fm=g.getFontMetrics();int dw=fm.stringWidth(msg);
            double ox=x;if(anchor.equals("middle"))ox-=dw/2.0;else if(anchor.equals("end"))ox-=dw;
            g.drawString(msg,(float)ox,(float)y);
            svg.append("<text x=\"").append(f(x)).append("\" y=\"").append(f(y)).append("\" fill=\"").append(hex(color))
                .append("\" font-family=\"Arial, sans-serif\" font-size=\"").append(size).append("\" font-weight=\"").append(bold?"bold":"normal")
                .append("\" text-anchor=\"").append(anchor).append("\">").append(xml(msg)).append("</text>");
        }
        void legend(double x,double y,Color col,String msg){circle(x,y-4,5,col);text(msg,x+13,y,14,NAVY,false,"start");}
        String f(double d){return String.format(Locale.ROOT,"%.2f",d);}
        void gridLine(double x,double y1,double y2){line(x,y1,x,y2,GRID,1);}
        void footer(String s){line(65,height-49,width-65,height-49,GRID,1.3);text(s,65,height-23,13,GREY,false,"start");}
        void title(String t,String sub){text(t,68,61,28,NAVY,true,"start");text(sub,68,89,15,GREY,false,"start");line(68,109,width-68,109,GRID,1.5);}
        void save(String name)throws IOException{
            Path dir=output.resolve("figures");
            ImageIO.write(image,"png",dir.resolve(figureFilename(name, "png")).toFile());
            Files.writeString(dir.resolve(figureFilename(name, "svg")),svg.toString()+"</svg>",StandardCharsets.UTF_8);
        }
        public void close(){g.dispose();}
    }
    static double map(double v,double lo,double hi,double x0,double x1){return x0+(v-lo)*(x1-x0)/(hi-lo);}
    static String nice(double x,int dec){return String.format(Locale.ROOT,"%."+dec+"f",x);}
    static String fmtNum(double x){return Double.isFinite(x)?nice(x,1):"N/A";}

    private static void createFigures()throws IOException{
        dumbbell(0,"Fig_Revisit_Ratio");
        dumbbell(1,"Fig_Overlap_Ratio");
        dumbbell(2,"Fig_Completion_Steps");
        placementPlot();
        stackedOutcome();
        pairedDistribution();
        forestPlot();
        tradeoffScatter();
        heatmap();
        revisitScatter();
    }
    private static void dumbbell(int metric,String name)throws IOException{
        try(Drawing d=new Drawing(1350,760)){
            boolean steps=metric==2;
            double max=steps?850:100;
            d.title(METRIC_NAMES[metric]+": paired method comparison",
                "Means of paired samples within each environmental topology | M0 = baseline, M1 = repulsive pheromone");
            d.legend(890,151,ORANGE,"M0 | lambda = 0");d.legend(1120,151,TEAL,"M1 | lambda = 1");
            double x0=280,x1=1170,y0=244,dy=81;
            for(int v=0;v<=5;v++){
                double val=steps?v*150:v*20;if(val>max)continue;
                double x=map(val,0,max,x0,x1);d.gridLine(x,202,630);
                d.text(String.format(Locale.ROOT,"%.0f",val),x,656,14,GREY,false,"middle");
            }
            for(int t=0;t<5;t++){
                List<Sample> a=byTopo(CODES[t]);double m0=meanMetric(a,metric,false),m1=meanMetric(a,metric,true);
                double y=y0+t*dy,xM0=map(m0,0,max,x0,x1),xM1=map(m1,0,max,x0,x1);
                d.text(TOPOS[t],x0-25,y+6,17,NAVY,false,"end");
                d.line(xM0,y,xM1,y,Color.decode("#B8CDD1"),3.2);
                d.circle(xM0,y,7.5,ORANGE);d.circle(xM1,y,7.5,TEAL);
                d.text((steps?"-"+nice(m0-m1,0)+" steps":"-"+nice(m0-m1,1)+" pp"),
                    Math.max(xM0,xM1)+16,y+4,13,NAVY,false,"start");
            }
            d.text(steps?"Completion Steps (global simulation timesteps)":METRIC_NAMES[metric]+" (%)",720,696,16,NAVY,false,"middle");
            d.footer("Two observations per geometry (C/D), ten geometries per topology; mean shown for 20 samples per topology.");
            d.save(name);
        }
    }
    private static void placementPlot()throws IOException{
        try(Drawing d=new Drawing(1380,780)){
            d.title("Influence of initial drone placement",
                "Clustered (C) vs distributed (D): positive savings mean M1 needs fewer completion steps");
            d.legend(850,150,TEAL,"Clustered start (C)");d.legend(1080,150,ORANGE,"Distributed start (D)");
            double min=-170,max=450,x0=285,x1=1260,y0=247,dy=84;
            for(int t=-150;t<=450;t+=75){double x=map(t,min,max,x0,x1);d.gridLine(x,195,664);d.text(Integer.toString(t),x,688,13,GREY,false,"middle");}
            double zx=map(0,min,max,x0,x1);d.line(zx,194,zx,666,Color.decode("#A0AEB7"),2);
            for(int t=0;t<5;t++){
                d.text(TOPOS[t],x0-25,y0+t*dy+7,16,NAVY,false,"end");
                for(String start:List.of("C","D")){
                    Interval z=startCI.get(CODES[t]).get(start+"2");if(!Double.isFinite(z.value))continue;
                    Color c=start.equals("C")?TEAL:ORANGE;double y=y0+t*dy+(start.equals("C")?-11:12);
                    d.line(map(z.lo,min,max,x0,x1),y,map(z.hi,min,max,x0,x1),y,c,3);
                    for(double q:new double[]{z.lo,z.hi}){double x=map(q,min,max,x0,x1);d.line(x,y-7,x,y+7,c,2);}
                    d.circle(map(z.value,min,max,x0,x1),y,6.5,c);
                }
            }
            d.text("Completion-step savings (M0 - M1)",760,727,17,NAVY,false,"middle");
            d.footer("Points: mean over 10 geometries; error bars: geometry-cluster bootstrap 95% percentile intervals.");
            d.save("Fig_Starting_Position_Effects");
        }
    }
    private static void stackedOutcome()throws IOException{
        try(Drawing d=new Drawing(1350,745)){
            d.title("Pheromone benefit by environment topology",
                "Share of paired test cases where each method requires fewer completion steps");
            d.legend(300,158,TEAL,"M1 faster");d.legend(530,158,ORANGE,"M0 faster");d.legend(755,158,GREY,"Equal steps");
            double x0=310,x1=1125,y0=245,dy=77;
            for(int t=0;t<=5;t++){
                double x=map(t*20,0,100,x0,x1);d.gridLine(x,209,639);d.text(t*20+"%",x,663,14,GREY,false,"middle");
            }
            for(int t=0;t<5;t++){
                List<Sample> a=byTopo(CODES[t]);
                double n=countOutcome(a,"M1 faster")+countOutcome(a,"M0 faster")+countOutcome(a,"Tie");
                double w=n==0?0:100.*countOutcome(a,"M1 faster")/n,
                       l=n==0?0:100.*countOutcome(a,"M0 faster")/n,
                       tie=n==0?0:100.*countOutcome(a,"Tie")/n;
                double y=y0+t*dy,barh=43;
                d.text(TOPOS[t],x0-22,y+26,17,NAVY,false,"end");
                double xx=x0;
                double[] pct={w,l,tie};Color[] color={TEAL,ORANGE,GREY};
                for(int j=0;j<3;j++){
                    if(pct[j]<=0)continue;
                    double wid=(x1-x0)*pct[j]/100.;d.rect(xx,y,wid,barh,color[j]);
                    if(wid>38)d.text(nice(pct[j],0)+"%",xx+wid/2,y+27,14,j==0?Color.WHITE:NAVY,true,"middle");xx+=wid;
                }
                d.text(String.format(Locale.ROOT,"%d / %d / %d",countOutcome(a,"M1 faster"),countOutcome(a,"M0 faster"),countOutcome(a,"Tie")),
                    x1+26,y+27,14,NAVY,false,"start");
            }
            d.text("Percentage of paired samples within each topology",728,703,16,NAVY,false,"middle");
            d.footer("The three categories are mutually exclusive and exhaustive; each topology contains 20 paired samples in this dataset.");
            d.save("Fig_100_Percent_Stacked_Bar_Topology");
        }
    }
    private static void pairedDistribution()throws IOException{
        try(Drawing d=new Drawing(1370,770)){
            d.title("Distribution of paired completion-step savings",
                "Each point represents one geometry/start sample | Positive values indicate M1 finishes sooner");
            d.legend(860,149,TEAL,"Clustered (C)");d.legend(1060,149,ORANGE,"Distributed (D)");
            double y0=195,y1=648,lo=-430,hi=430,x0=250,dx=225;
            for(int tick=-400;tick<=400;tick+=100){double y=map(tick,lo,hi,y1,y0);d.line(130,y,1300,y,GRID,1);d.text(Integer.toString(tick),110,y+4,13,GREY,false,"end");}
            double zero=map(0,lo,hi,y1,y0);d.line(130,zero,1300,zero,ORANGE,1.6);
            for(int t=0;t<5;t++){
                double x=x0+t*dx;List<Sample>a=byTopo(CODES[t]);List<Double> sv=saved(a,2);
                double q1=quantile(sv,.25),q3=quantile(sv,.75),med=quantile(sv,.5);
                double upper=map(q3,lo,hi,y1,y0),lower=map(q1,lo,hi,y1,y0);
                d.rect(x-25,upper,50,Math.max(2,lower-upper),Color.decode("#D1E5E1"));
                d.line(x-25,map(med,lo,hi,y1,y0),x+25,map(med,lo,hi,y1,y0),NAVY,3);
                for(int k=0;k<a.size();k++){
                    Sample s=a.get(k);if(!s.complete())continue;
                    double jitter=(s.start.equals("C")?-18:18)+((Integer.parseInt(s.id.substring(5,7))%4)-1.5)*3.0;
                    d.circle(x+jitter,map(s.savings(2),lo,hi,y1,y0),5.8,s.start.equals("C")?TEAL:ORANGE);
                }
                d.text(TOPOS[t],x,689,15,NAVY,false,"middle");
            }
            d.text("Saved Completion Steps (M0 - M1)",700,737,15,NAVY,false,"middle");
            d.footer("Shaded boxes: Q1–Q3 of 20 samples; dark horizontal bar: median; dashed direction is not time.");
            d.save("Fig_A_Paired_Step_Difference_Distribution");
        }
    }
    private static void forestPlot()throws IOException{
        try(Drawing d=new Drawing(1610,730)){
            d.title("Effect sizes by environment topology",
                "Points: mean paired improvement | Error bars: 95% geometry-cluster bootstrap percentile intervals");
            d.legend(1310,149,TEAL,"Improvement M0 - M1");
            int[] metric={0,1,2};double[] mins={0,0,-60},maxs={55,80,290};
            double top=237,dy=75,panelW=400;
            for(int m:metric){
                double left=245+m*445,right=left+panelW;
                d.text(m==0?"Revisit reduction (pp)":m==1?"Overlap reduction (pp)":"Completion-step savings",(left+right)/2,192,17,NAVY,true,"middle");
                int ticks=5;for(int z=0;z<=ticks;z++){
                    double val=mins[m]+(maxs[m]-mins[m])*z/ticks;
                    double x=map(val,mins[m],maxs[m],left,right);
                    d.gridLine(x,211,613);d.text(nice(val,0),x,638,12,GREY,false,"middle");
                }
                if(mins[m]<=0&&maxs[m]>=0){double xx=map(0,mins[m],maxs[m],left,right);d.line(xx,210,xx,612,ORANGE,2);}
                for(int t=0;t<5;t++){
                    Interval ci=topoCI.get(CODES[t]).get("metric"+m);double y=top+t*dy;
                    if(m==0)d.text(TOPOS[t],210,y+5,16,NAVY,false,"end");
                    double lo=Math.max(mins[m],ci.lo),hi=Math.min(maxs[m],ci.hi);
                    double xa=map(lo,mins[m],maxs[m],left,right),xb=map(hi,mins[m],maxs[m],left,right);
                    d.line(xa,y,xb,y,TEAL,3);
                    d.line(xa,y-8,xa,y+8,TEAL,2);d.line(xb,y-8,xb,y+8,TEAL,2);
                    d.circle(map(ci.value,mins[m],maxs[m],left,right),y,6,TEAL);
                    d.text(nice(ci.value,1),map(ci.value,mins[m],maxs[m],left,right),y-13,12,NAVY,true,"middle");
                }
            }
            d.footer("Resampling unit = geometry (10 per topology); C and D retained together. Seed: "+BOOTSTRAP_SEED+"; B="+BOOTSTRAP_REPETITIONS+".");
            d.save("Fig_B_Topology_Effect_Forest");
        }
    }
    private static void tradeoffScatter()throws IOException{
        try(Drawing d=new Drawing(1420,830)){
            d.title("Redundancy–efficiency trade-off at the sample level",
                "Every dot = one paired sample | positive reductions mean M1 performs better");
            d.legend(860,152,TEAL,"Clustered (C)");d.legend(1090,152,ORANGE,"Distributed (D)");
            double x0=184,x1=1320,y0=210,y1=697,xmin=-20,xmax=110,ymin=-450,ymax=440;
            d.rect(x0,map(0,ymin,ymax,y1,y0),x1-x0,y1-map(0,ymin,ymax,y1,y0),ORANGE_LIGHT);
            d.rect(map(0,xmin,xmax,x0,x1),y0,x1-map(0,xmin,xmax,x0,x1),map(0,ymin,ymax,y1,y0)-y0,TEAL_LIGHT);
            for(int v=-20;v<=100;v+=20){double x=map(v,xmin,xmax,x0,x1);d.gridLine(x,y0,y1);d.text(Integer.toString(v),x,y1+27,13,GREY,false,"middle");}
            for(int v=-400;v<=400;v+=100){double y=map(v,ymin,ymax,y1,y0);d.line(x0,y,x1,y,GRID,1);d.text(Integer.toString(v),x0-19,y+5,13,GREY,false,"end");}
            double zy=map(0,ymin,ymax,y1,y0),zx=map(0,xmin,xmax,x0,x1);d.line(x0,zy,x1,zy,GREY,1.6);d.line(zx,y0,zx,y1,GREY,1.6);
            for(Sample s:samples){double xx=s.savings(1),yy=s.savings(2);
                if(!Double.isFinite(xx)||!Double.isFinite(yy))continue;
                d.circle(map(xx,xmin,xmax,x0,x1),map(yy,ymin,ymax,y1,y0),5.5,s.start.equals("C")?TEAL:ORANGE);
            }
            d.text("Overlap reduction (percentage points, M0 - M1)",(x0+x1)/2,767,16,NAVY,false,"middle");
            d.text("Saved steps",x0,190,14,NAVY,true,"start");
            d.footer("Quadrants show joint direction of change, not causal effects. Negative step savings indicate slower M1.");
            d.save("Fig_C_Overlap_vs_Step_Tradeoff");
        }
    }
    private static Color interpolate(Color a,Color b,double t){t=Math.max(0,Math.min(1,t));return new Color((int)(a.getRed()+(b.getRed()-a.getRed())*t),
            (int)(a.getGreen()+(b.getGreen()-a.getGreen())*t),(int)(a.getBlue()+(b.getBlue()-a.getBlue())*t));}
    private static void heatmap()throws IOException{
        try(Drawing d=new Drawing(1550,755)){
            d.title("Completion-step improvement across all paired samples",
                "Rows are topologies; columns are geometry G01–G10 with paired starting configurations C and D");
            double x0=269,y0=234,cellW=55,cellH=73;
            for(int t=0;t<5;t++){
                d.text(TOPOS[t],x0-24,y0+t*cellH+43,15,NAVY,false,"end");
                for(int g=1;g<=10;g++)for(int p=0;p<2;p++){
                    String id=String.format(Locale.ROOT,"%s_G%02d_%s",CODES[t],g,p==0?"C":"D");
                    Sample found=null;for(Sample s:byTopo(CODES[t]))if(s.id.equals(id)){found=s;break;}
                    double value=found==null?Double.NaN:found.savings(2);
                    Color fill=!Double.isFinite(value)?GRID:value>=0?interpolate(Color.WHITE,TEAL,.11+.72*Math.min(value,400)/400):
                        interpolate(Color.WHITE,ORANGE,.15+.75*Math.min(-value,400)/400);
                    double x=x0+(g-1)*2*cellW+p*cellW,y=y0+t*cellH;
                    d.rect(x+1,y+1,cellW-2,cellH-2,fill);
                    if(Double.isFinite(value))d.text(String.format(Locale.ROOT,"%+.0f",value),x+cellW/2,y+40,12,(Math.abs(value)>250?Color.WHITE:NAVY),true,"middle");
                }
            }
            for(int g=1;g<=10;g++){
                double cx=x0+(g-1)*2*cellW+cellW;
                d.text(String.format(Locale.ROOT,"G%02d",g),cx,y0-16,14,NAVY,true,"middle");
            }
            d.legend(270,668,TEAL,"M1 faster (positive)");d.legend(525,668,ORANGE,"M1 slower (negative)");
            d.text("Within every geometry: left cell = C, right cell = D",1220,672,14,NAVY,false,"end");
            d.footer("Values are saved timesteps M0 - M1. Color saturation reflects magnitude, capped at 400 steps.");
            d.save("Fig_Per_Sample_Step_Heatmap");
        }
    }
    private static void revisitScatter()throws IOException{
        try(Drawing d=new Drawing(1390,795)){
            d.title("Revisit reduction versus completion efficiency",
                "Descriptive association across paired samples (not a causal analysis)");
            d.legend(810,147,TEAL,"Clustered (C)");d.legend(1040,147,ORANGE,"Distributed (D)");
            double x0=180,x1=1285,y0=206,y1=680,xmin=-20,xmax=85,ymin=-430,ymax=420;
            for(int v=-20;v<=80;v+=20){double x=map(v,xmin,xmax,x0,x1);d.gridLine(x,y0,y1);d.text(Integer.toString(v),x,y1+24,13,GREY,false,"middle");}
            for(int v=-400;v<=400;v+=100){double y=map(v,ymin,ymax,y1,y0);d.line(x0,y,x1,y,GRID,1);d.text(Integer.toString(v),x0-18,y+5,13,GREY,false,"end");}
            d.line(map(0,xmin,xmax,x0,x1),y0,map(0,xmin,xmax,x0,x1),y1,GREY,1.6);
            d.line(x0,map(0,ymin,ymax,y1,y0),x1,map(0,ymin,ymax,y1,y0),GREY,1.6);
            for(Sample s:samples){double x=s.savings(0),y=s.savings(2);
                if(Double.isFinite(x)&&Double.isFinite(y))d.circle(map(x,xmin,xmax,x0,x1),map(y,ymin,ymax,y1,y0),5.4,s.start.equals("C")?TEAL:ORANGE);
            }
            double rho=spearman(samples,0,2);
            d.text("Spearman rho = "+nice(rho,3)+" (descriptive; n = "+samples.size()+")",190,174,15,TEAL,true,"start");
            d.text("Revisit reduction (percentage points)",(x0+x1)/2,742,16,NAVY,false,"middle");
            d.footer("Positive x means fewer repeated physical visits; positive y means fewer completion steps for M1.");
            d.save("Fig_Revisit_vs_Steps_Exploratory");
        }
    }
    private static List<Double> rank(List<Double> v){
        int n=v.size();Integer[] ix=new Integer[n];for(int i=0;i<n;i++)ix[i]=i;
        Arrays.sort(ix,Comparator.comparingDouble(v::get));double[] rank=new double[n];
        int i=0;while(i<n){int j=i+1;while(j<n&&Double.compare(v.get(ix[j]),v.get(ix[i]))==0)j++;
            double r=(i+1+j)/2.0;for(int k=i;k<j;k++)rank[ix[k]]=r;i=j;
        }
        List<Double> out=new ArrayList<>();for(double a:rank)out.add(a);return out;
    }
    private static double spearman(List<Sample>a,int i,int j){List<Double>x=new ArrayList<>(),y=new ArrayList<>();
        for(Sample s:a)if(Double.isFinite(s.savings(i))&&Double.isFinite(s.savings(j))){x.add(s.savings(i));y.add(s.savings(j));}
        List<Double>rx=rank(x),ry=rank(y);double mx=mean(rx),my=mean(ry),a1=0,b1=0,c1=0;
        for(int k=0;k<rx.size();k++){double u=rx.get(k)-mx,v=ry.get(k)-my;a1+=u*v;b1+=u*u;c1+=v*v;}
        return b1==0||c1==0?Double.NaN:a1/Math.sqrt(b1*c1);
    }
    // ------------------------------------------------------------------------
    // README: inventories, links, embeds, whole Markdown tables, and formulas.
    // ------------------------------------------------------------------------
    private static final String[] FIG_NAMES={
        "Fig_Revisit_Ratio", "Fig_Overlap_Ratio", "Fig_Completion_Steps", "Fig_Starting_Position_Effects",
        "Fig_100_Percent_Stacked_Bar_Topology", "Fig_A_Paired_Step_Difference_Distribution",
        "Fig_B_Topology_Effect_Forest", "Fig_C_Overlap_vs_Step_Tradeoff",
        "Fig_Per_Sample_Step_Heatmap", "Fig_Revisit_vs_Steps_Exploratory"
    };
    private static final String[] FIG_TITLES={
        "Revisit Ratio — paired M0/M1 mean comparison",
        "Overlap Ratio — paired M0/M1 mean comparison",
        "Completion Steps — paired M0/M1 mean comparison",
        "Influence of initial drone placement (C vs D)",
        "100% stacked bar — faster method by topology",
        "Paired completion-step savings distribution",
        "Forest plot — 95% CI for three performance effects",
        "Overlap reduction vs saved steps — trade-off",
        "Heatmap — per-sample completion-step savings",
        "Revisit reduction vs saved steps — descriptive correlation"
    };
    private static final String[] FIG_TABLE={
        "01_Method_Means", "01_Method_Means", "01_Method_Means", "02_Starting_Placement",
        "03_Outcome_By_Topology", "04_Improvement_Consistency", "05_Topology_Bootstrap_CI",
        "06_Overlap_Steps_Joint", "08_Paired_100", "08_Paired_100"
    };
    private static final String[] FIG_EXPLANATIONS={
        "Dumbbell plot of **mean physical revisit ratio (%)** for M0 and M1 in each topology. Dots are topology-level means; the connecting segment is the paired-method mean difference. Lower values are desirable at equal coverage.",
        "Dumbbell plot of **mean inter-drone overlap ratio (%)** for M0 and M1. Overlap is the share of unique visited cells reached by at least two distinct drones. It is not sensing overlap.",
        "Dumbbell plot of **mean completion steps** (global simulation timesteps) for completed runs. Differences use M0 minus M1; these are not wall-clock seconds.",
        "Geometry-cluster bootstrap intervals for mean saved steps, independently within Clustered (C) and Distributed (D) starts. Their difference describes a possible modification of effect by start arrangement.",
        "Outcome share from mutually exclusive M1 faster, M0 faster, or tie categories based on paired completion steps. Topology is categorical; stacked bars are preferable to stacked area plots.",
        "All sample-level paired step differences displayed by environment topology and starting configuration. The center line is the median; the pale box spans Q1–Q3. Negative values explicitly show worse M1 outcomes.",
        "Three-panel forest plot showing geometry-cluster bootstrap 95% confidence intervals for mean revisit reduction, overlap reduction, and step savings. This is descriptive uncertainty for designed simulation maps.",
        "Scatter plot of paired overlap reduction (horizontal) versus saved steps (vertical); each marker is one sample. Four quadrants reveal that reducing overlap does not always yield faster exploration.",
        "Five-by-twenty heatmap of sample-level saved completion steps. Within each geometry, the left square is C and the right square is D. Teal denotes positive savings; orange denotes negative savings.",
        "Scatter plot of paired revisit reduction versus saved steps, with Spearman rank correlation. Association is descriptive, not a claim that revisit reduction causes step savings."
    };
    private static String md(String x){return x.replace("|","\\|").replace("\n"," ");}
    private static String valueMd(Object x){return x instanceof Double d?nice(d,2):md(String.valueOf(x));}
    private static void mdTable(StringBuilder b,Sheet s){
        b.append("\n| ");for(String h:s.headers)b.append(md(h)).append(" | ");b.append("\n|");
        for(int i=0;i<s.headers.size();i++)b.append("---|");b.append("\n");
        for(List<Object>r:s.rows){b.append("|");for(Object a:r)b.append(" ").append(valueMd(a)).append(" |");b.append("\n");}
        b.append("\n");
    }
    private static String trimPath(Path p){return p.getFileName().toString();}
    private static void makeReadme(Path input)throws IOException{
        StringBuilder b=new StringBuilder();
        b.append("# Research Results — Reproducible Tables and Figures\n\n")
         .append("**Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy**\n\n")
         .append("Generated by `ResearchResultsGenerator.java` on ")
         .append(LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")))
         .append(" (local computer time). This generator is **independent of JavaFX and the simulation source**.\n\n")
         .append("## Inputs and reproducibility\n\n")
         .append("- Input CSV: `").append(trimPath(input)).append("` (configurable using `INPUT_CSV` or command-line argument).\n")
         .append("- Input SHA-256: `").append(sourceSha).append("`.\n")
         .append("- Rows read: **").append(samples.size()).append("** paired sample IDs; completed pairs: **")
         .append(samples.stream().filter(Sample::complete).count()).append("**.\n")
         .append("- Methods: M0 = lambda 0; M1 = lambda 1. CSV stores M0 and M1 in the same row for each Sample ID.\n")
         .append("- Colors: M1 / positive = **teal `#007E6E`**; M0 / negative = **orange `#F79A19`**; gray = ties. In scatterplots, teal = clustered C and orange = distributed D.\n")
         .append("- Bootstrap: geometry-resampling, B=").append(BOOTSTRAP_REPETITIONS).append(", seed=").append(BOOTSTRAP_SEED).append("; 2.5th/97.5th percentiles.\n")
         .append("- Result code inherited from the source file: **`").append(resultCode).append("`**.\n")
         .append("- Output directory: `").append(output).append("`.\n")
         .append("- Files: `").append(workbookFilename()).append("` (9 worksheets), `tables/*_")
         .append(resultCode).append(".csv`, `figures/*_").append(resultCode).append(".png`/`.svg`, and `")
         .append(readmeFilename()).append("`.\n\n")
         .append("## IntelliJ IDEA — compile and run\n\n")
         .append("Place **only this class** at `src/main/java/drone/decentralizedswarmcoverage_fbe_rps/ResearchResultsGenerator.java`.\n\n")
         .append("IntelliJ: right-click `ResearchResultsGenerator.java` and select **Run 'ResearchResultsGenerator.main()'**. ")
         .append("Under Run > Edit Configurations, ensure **Working directory = `$PROJECT_DIR$`**.\n\n")
         .append("Since this is a **named JPMS module**, graphics output with Java2D/ImageIO requires ")
         .append("**one line** in `src/main/java/module-info.java`: `requires java.desktop;`. ")
         .append("This is a module dependency, not an algorithm/source-logic change.\n\n")
         .append("Default file layout:\n\n")
         .append("```text\n")
         .append("<PROJECT ROOT>/\n  results/ExplorationResults<CODE>.csv\n")
         .append("  src/main/java/drone/decentralizedswarmcoverage_fbe_rps/ResearchResultsGenerator.java\n")
         .append("  analysis/Research_Tables_<CODE>.xlsx\n  analysis/README_<CODE>.md\n")
         .append("  analysis/tables/<TABLE>_<CODE>.csv\n  analysis/figures/<FIGURE>_<CODE>.png and .svg\n")
         .append("```\n\n")
         .append("Edit the class fields to select another existing `ExplorationResults<CODE>.csv` in `results/`: ")
         .append("`INPUT_CSV`, `OUTPUT_DIRECTORY = \"analysis\"`, `CSV_DELIMITER = \";\"`.\n\n")
         .append("You may also supply two **optional program arguments**: `inputCsvPath outputDirectory`. ")
         .append("Relative output directories resolve from the project root, and the code always comes from the input filename. ")
         .append("The input delimiter is auto-detected (`;` or `,`).\n\n")
         .append("## Original attachment inventory and generated equivalents\n\n")
         .append("The reference archive contained **10 different figures** (each PNG and SVG), 5 summary CSV tables, and 1 supplemental paired-differences CSV. The following figures are regenerated from the input, not copied from the reference images.\n\n")
         .append("| # | Original / recreated figure | Excel worksheet giving the underlying numbers |\n|---|---|---|\n");
        for(int i=0;i<FIG_NAMES.length;i++)b.append("| ").append(i+1).append(" | [").append(FIG_NAMES[i]).append("](figures/")
            .append(figureFilename(FIG_NAMES[i], "png")).append(") | [").append(FIG_TABLE[i]).append("](tables/").append(tableFilename(FIG_TABLE[i])).append(") |\n");
        b.append("\n**Summary tables from the reference archive:** `topology_outcome_summary.csv` → worksheet 03; ")
         .append("`Table_A_Improvement_Consistency_by_Topology.csv` → 04; ")
         .append("`Table_B_All_14_Slower_M1_Cases.csv` → 07; ")
         .append("`Table_C_Speed_Redundancy_Joint_Outcome.csv` → 06; ")
         .append("`Table_D_Topology_Cluster_Bootstrap_CI.csv` → 05; ")
         .append("`Supplement_Paired_Differences_100_Samples.csv` → 08. ")
         .append("Two additional tables (01 method means and 02 starting placement) were created to expose the data behind the corresponding original figures. Raw input with Excel formulas is worksheet 09.\n\n")
         .append("## Formula definitions and interpretation\n\n")
         .append("For each sample i, M0 and M1 are the two runs on the **same .sc map and the same four starting positions**.\n\n")
         .append("- **Revisit reduction (percentage points):** `R0_i - R1_i`; R denotes Revisit Ratio (%).\n")
         .append("- **Overlap reduction (percentage points):** `O0_i - O1_i`; O denotes Overlap Ratio (%).\n")
         .append("- **Saved completion steps:** `S0_i - S1_i`; positive = M1 faster, negative = M1 slower, zero = tie.\n")
         .append("- **Relative mean step reduction (%):** `100 × (mean(S0) - mean(S1)) / mean(S0)`. This is a **ratio of means**, not the mean of per-sample percentages.\n")
         .append("- **Topology mean (metric):** arithmetic mean of sample values within that topology; for this dataset 10 geometries × 2 start placements = 20 sample pairs.\n")
         .append("- **Median and Q1/Q3:** linear-interpolated quantile at p=0.50, p=0.25, p=0.75, using rank position `(n-1)×p`.\n")
         .append("- **Outcome percentage:** `100 × category_count / complete_sample_count` within topology.\n")
         .append("- **Geometry-cluster bootstrap CI:** average C and D improvements within each geometry, sample the 10 geometry means with replacement, and take the 2.5% and 97.5% empirical quantiles across ")
         .append(BOOTSTRAP_REPETITIONS).append(" resamples. For starting-position analysis, C and D are bootstrapped separately over geometry IDs.\n")
         .append("- **Spearman rho:** Pearson correlation of the midranks of sample-level revisit reductions and saved steps; tied values receive average rank.\n")
         .append("- **Explored coverage:** preserved from the original CSV. If the final coverage is 100% for all runs, it is a completion prerequisite rather than an observed difference between methods.\n")
         .append("- **Missing/incomplete runs:** the original `N/A` steps are preserved; paired step comparisons and step CIs exclude nonnumeric pairs. Coverage/Revisit/Overlap descriptive summaries still use available valid input values.\n\n")
         .append("**Statistical limitations.** Bootstrap intervals summarize variability across the designed geometries. They do not establish external validity for arbitrary real environments. The generator does not compute or claim p-values, temporal exploration curves, runtime seconds, or energy. A distinct Wilcoxon/permutation analysis would need to be reported separately.\n\n")
         .append("## Figures with their corresponding tables\n\n");
        int[] done=new int[sheets.size()];
        for(int i=0;i<FIG_NAMES.length;i++){
            Sheet related=null;int k=-1;
            for(int q=0;q<sheets.size();q++)if(sheets.get(q).name.equals(FIG_TABLE[i])){related=sheets.get(q);k=q;break;}
            b.append("### Figure ").append(i+1).append(" — ").append(FIG_TITLES[i]).append("\n\n");
            // Place the source table immediately before its corresponding chart.
            if(related!=null && done[k]==0){
                b.append("**Source Table — `").append(related.name).append("`** ")
                 .append("([Excel worksheet](").append(workbookFilename()).append(") · [CSV](tables/")
                 .append(tableFilename(related.name)).append("))\n\n");
                mdTable(b,related);done[k]=1;
            }else if(related!=null){
                b.append("Underlying numbers: [").append(related.name).append("](tables/")
                    .append(tableFilename(related.name)).append(") (shown earlier).\n\n");
            }
            // Attach the diagnostic slower-M1 table directly to the trade-off discussion.
            if(i==7){
                Sheet slower=sheets.get(6);
                b.append("**Related diagnostic table — every sample where M1 is slower**\n\n");
                mdTable(b,slower);done[6]=1;
            }
            b.append("![").append(FIG_TITLES[i]).append("](figures/").append(figureFilename(FIG_NAMES[i], "png")).append(")\n\n")
              .append("[Vector SVG](figures/").append(figureFilename(FIG_NAMES[i], "svg")).append(") · ")
              .append("[Data table CSV](tables/").append(tableFilename(FIG_TABLE[i])).append(")\n\n")
              .append(FIG_EXPLANATIONS[i]).append("\n\n");
        }
        b.append("## Additional tables and supplementary data\n\n");
        for(int q=0;q<sheets.size();q++){
            Sheet s=sheets.get(q);if(done[q]==1)continue;
            b.append("### ").append(s.name).append("\n\n")
              .append("[CSV](tables/").append(tableFilename(s.name)).append(") · sheet `").append(s.name).append("` in the Excel workbook.\n\n");
            if(s.rows.size()<=35)mdTable(b,s);
            else{
                b.append("The following complete table has ").append(s.rows.size()).append(" rows and is also in the Excel workbook.\n\n");
                mdTable(b,s);
            }
        }
        b.append("## Research-use notes\n\n")
         .append("1. **Never stack raw M0 and M1 values** of Revisit/Overlap/Steps as an area: they are alternative measurements, not additive components. Stacking mutually exclusive faster/tie/slower shares is appropriate.\n")
         .append("2. Completion Steps are algorithmic global timesteps, not elapsed seconds. Revisit and Overlap describe *physical cell visits*, not repeated sensor observation.\n")
         .append("3. Revisit/Overlap improvements may occur even when Completion Steps become worse: interpret the trade-off chart accordingly.\n")
         .append("4. Original figure files in the supplied ZIP serve as layout/type references. The new PNG and SVG files use the same computed points but redesigned colors (#007E6E, #F79A19); pixel-identical replication of the originals is not claimed.\n")
         .append("5. Source data, input SHA-256, fixed bootstrap seed, all tables, and charts are retained to make figure production auditable.\n");
        Files.writeString(output.resolve(readmeFilename()),b.toString(),StandardCharsets.UTF_8);
    }
}
