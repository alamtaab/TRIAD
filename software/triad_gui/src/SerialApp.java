import com.fazecast.jSerialComm.SerialPort;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.util.Scanner;
import java.nio.charset.StandardCharsets;

public class SerialApp extends JFrame {

    private JComboBox<String> portList;
    private JTextField baudField;
    private JButton connectBtn, disconnectBtn, sendBtn;
    private JTextField inputField;
    private JTextArea logArea;

    private SerialPort serialPort;
    private boolean running = false;

    public SerialApp() {
        super("Triad GUI");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        setSize(700, 500);
        setLayout(new BorderLayout());

        // top [anel
        JPanel top = new JPanel();
        portList = new JComboBox<>();
        refreshPorts();
        baudField = new JTextField("9600", 8);
        connectBtn = new JButton("Connect");
        disconnectBtn = new JButton("Disconnect");
        top.add(new JLabel("Port:"));
        top.add(portList);
        top.add(new JLabel("Baud:"));
        top.add(baudField);
        top.add(connectBtn);
        top.add(disconnectBtn);
        add(top, BorderLayout.NORTH);

        // center log
        logArea = new JTextArea();
        logArea.setEditable(false);
        logArea.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 13));
        add(new JScrollPane(logArea), BorderLayout.CENTER);

        // send line
        JPanel bottom = new JPanel(new BorderLayout());
        inputField = new JTextField();
        sendBtn = new JButton("Send");
        bottom.add(inputField, BorderLayout.CENTER);
        bottom.add(sendBtn, BorderLayout.EAST);
        add(bottom, BorderLayout.SOUTH);



        setVisible(true);
    }


    private void refreshPorts() {
        portList.removeAllItems();
        for (SerialPort port : SerialPort.getCommPorts()) {
            portList.addItem(port.getSystemPortName());
        }
    }

    private void onConnect(ActionEvent e)  {
        if(running) return;
        try {
            String portName = (String) portList.getSelectedItem();

            int baud = Integer.parseInt(baudField.getText().trim());

            serialPort = serialPort.getCommPort(portName);

            serialPort.setBaudRate(baud);
            serialPort.openPort();
            running = true;

        } catch (Exception ex) {
            append("err + "+ ex.getMessage());
        }
    }

    private void append(String msg) {
        logArea.append(msg + "\n");
        logArea.setCaretPosition(logArea.getDocument().getLength());
    }
    public static void main(String[] args) {
        SwingUtilities.invokeLater(SerialApp::new);
    }

}
