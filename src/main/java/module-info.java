module drone.decentralizedswarmcoverage_fbe_rps {
    requires javafx.controls;
    requires javafx.fxml;
    requires java.desktop;

    opens drone.decentralizedswarmcoverage_fbe_rps to javafx.fxml;
    exports drone.decentralizedswarmcoverage_fbe_rps;
}