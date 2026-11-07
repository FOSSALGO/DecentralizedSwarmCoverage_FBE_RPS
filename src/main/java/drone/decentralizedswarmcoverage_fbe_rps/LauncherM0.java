package drone.decentralizedswarmcoverage_fbe_rps;

import javafx.application.Application;

/**
 * M0 GUI entry point (lambda = 0.0) for visual comparison with the existing M1.
 *
 * Launches the ORIGINAL MainView, so every UI control, graphical component,
 * playback mechanism and report is shared without copying or modifying source.
 * Only the JVM-local swarm.lambda property differs.
 *
 * Run this class and the existing LauncherM1 in separate JVM processes.
 */
public final class LauncherM0 {

    private LauncherM0() { }

    public static void main(String[] args) {
        // DecentralizedExploration reads this property upon construction.
        // Set it before launching JavaFX, including if -Dswarm.lambda was supplied.
        System.setProperty("swarm.lambda", "0.0");

        // Same class, UI, and Run pathway as the existing M1 application.
        Application.launch(MainView.class, args);
    }
}
