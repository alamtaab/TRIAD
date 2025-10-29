import com.fazecast.jSerialComm.SerialPort;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.util.Scanner;

public class SerialApp extends JFrame {

    private JComboBox<String> portList;
    private JTextField baudField;
    private JButton connectBtn, disconnectBtn, sendBtn;
    private JTextField inputField;
    private JTextArea logArea;


    public SerialApp() {
        super("Triad GUI");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        setSize(700, 500);
        setLayout(new BorderLayout());

        // top [anel
        JPanel top = new JPanel();
        portList = new JComboBox<>();

        baudField = new JTextField("9600", 8);
        connectBtn = new JButton("Connect");
        disconnectBtn = new JButton("Disconnect");
        top.add(new JLabel("Port:"));
        top.add(portList);
        top.add(baudField);
        top.add(connectBtn);
        top.add(disconnectBtn);


        setVisible(true);
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(SerialApp::new);
    }

}
