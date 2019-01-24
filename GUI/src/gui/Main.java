/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package gui;

import com.fazecast.jSerialComm.SerialPort;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import javax.swing.ButtonModel;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

/**
 *
 * @author rdavi
 */
public class Main extends javax.swing.JFrame implements KeyListener {

    private static final SerialPort SERIAL = SerialPort.getCommPort("COM4");
    private static OutputStream outputStream;
    private static InputStream inputStream;

    /**
     * Creates new form Main
     */
    public Main() {
        initComponents();

        //sets up keyListener
        setKeyListener();

        // Readys port
        SERIAL.openPort();
        // Get port as output stream for ease of writing 
        outputStream = SERIAL.getOutputStream();
        inputStream = SERIAL.getInputStream();

        // Bind each button to a changelistener with a decimal attached
        forwardButton.getModel().addChangeListener(new BtnModelListener("W"));
        backwardsButton.getModel().addChangeListener(new BtnModelListener("S"));
        leftButton.getModel().addChangeListener(new BtnModelListener("A"));
        rightButton.getModel().addChangeListener(new BtnModelListener("D"));

    }

    //adds global keylistener to buttons and such to get around focus restrictions
    private void setKeyListener() {
        addKeyListener(this);
        setFocusable(true);
        setFocusTraversalKeysEnabled(false);
        forwardButton.addKeyListener(this);
        backwardsButton.addKeyListener(this);
        leftButton.addKeyListener(this);
        rightButton.addKeyListener(this);
        sendButton.addKeyListener(this);
    }

    // Writing to outputstream
    private void writeToOutputStream(String value) {
        try (PrintWriter writer = new PrintWriter(outputStream)) {
            writer.print(value);
        }
    }

    // generated code
    /**
     * This method is called from within the constructor to initialise the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        forwardButton = new javax.swing.JButton();
        backwardsButton = new javax.swing.JButton();
        rightButton = new javax.swing.JButton();
        leftButton = new javax.swing.JButton();
        inputPane = new javax.swing.JScrollPane();
        inputText = new javax.swing.JTextArea();
        inputStreamLabel = new javax.swing.JLabel();
        outputField = new javax.swing.JTextField();
        sendButton = new javax.swing.JButton();
        outputLabel = new javax.swing.JLabel();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setResizable(false);

        forwardButton.setText("Forward");

        backwardsButton.setText("Back");

        rightButton.setText("Right");

        leftButton.setText("Left");

        inputText.setColumns(20);
        inputText.setRows(5);
        inputPane.setViewportView(inputText);

        inputStreamLabel.setText("Input");

        sendButton.setText("Send");
        sendButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                sendButtonActionPerformed(evt);
            }
        });

        outputLabel.setText("Output");

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addContainerGap()
                        .addComponent(leftButton, javax.swing.GroupLayout.PREFERRED_SIZE, 76, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(74, 74, 74)
                        .addComponent(rightButton, javax.swing.GroupLayout.PREFERRED_SIZE, 76, javax.swing.GroupLayout.PREFERRED_SIZE))
                    .addGroup(layout.createSequentialGroup()
                        .addGap(83, 83, 83)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                            .addComponent(backwardsButton, javax.swing.GroupLayout.PREFERRED_SIZE, 76, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(forwardButton, javax.swing.GroupLayout.PREFERRED_SIZE, 76, javax.swing.GroupLayout.PREFERRED_SIZE))))
                .addGap(18, 18, 18)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(inputPane, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(inputStreamLabel))
                .addGap(18, 18, 18)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(outputField, javax.swing.GroupLayout.PREFERRED_SIZE, 95, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(sendButton))
                    .addComponent(outputLabel))
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addGap(6, 6, 6)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(inputStreamLabel)
                    .addComponent(outputLabel))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(inputPane, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(forwardButton)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(leftButton)
                            .addComponent(rightButton))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(backwardsButton, javax.swing.GroupLayout.PREFERRED_SIZE, 23, javax.swing.GroupLayout.PREFERRED_SIZE))
                    .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                        .addComponent(outputField, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addComponent(sendButton)))
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void sendButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_sendButtonActionPerformed
        // TODO add your handling code here:
        // Take the value in the outputText turn into byte array and send down output stream
        // Clear text box
    }//GEN-LAST:event_sendButtonActionPerformed

    /**
     * @param args the command line arguments
     */
    public static void main(String args[]) {
        /* Set the Nimbus look and feel */
        //<editor-fold defaultstate="collapsed" desc=" Look and feel setting code (optional) ">
        /* If Nimbus (introduced in Java SE 6) is not available, stay with the default look and feel.
         * For details see http://download.oracle.com/javase/tutorial/uiswing/lookandfeel/plaf.html 
         */
        try {
            for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager.getInstalledLookAndFeels()) {
                if ("Nimbus".equals(info.getName())) {
                    javax.swing.UIManager.setLookAndFeel(info.getClassName());
                    break;
                }
            }
        } catch (ClassNotFoundException ex) {
            java.util.logging.Logger.getLogger(Main.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (InstantiationException ex) {
            java.util.logging.Logger.getLogger(Main.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (IllegalAccessException ex) {
            java.util.logging.Logger.getLogger(Main.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (javax.swing.UnsupportedLookAndFeelException ex) {
            java.util.logging.Logger.getLogger(Main.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        }
        //</editor-fold>

        /* Create and display the form */
        java.awt.EventQueue.invokeLater(() -> {
            new Main().setVisible(true);
        });

    }

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JButton backwardsButton;
    private javax.swing.JButton forwardButton;
    private javax.swing.JScrollPane inputPane;
    private javax.swing.JLabel inputStreamLabel;
    private javax.swing.JTextArea inputText;
    private javax.swing.JButton leftButton;
    private javax.swing.JTextField outputField;
    private javax.swing.JLabel outputLabel;
    private javax.swing.JButton rightButton;
    private javax.swing.JButton sendButton;
    // End of variables declaration//GEN-END:variables

    // Component implementaion
    private class BtnModelListener implements ChangeListener {

        private boolean pressed = false;  // holds the last pressed state of the button
        private String buttonValue = " ";

        public BtnModelListener(String value) {
            this.buttonValue = value;
        }

        @Override
        public void stateChanged(ChangeEvent e) {
            ButtonModel model = (ButtonModel) e.getSource();

            // if the current state differs from the previous state
            if (model.isPressed() != pressed) {
                if (pressed != true) {
                    writeToOutputStream(buttonValue);
                } else {
                    writeToOutputStream(" ");
                }
                pressed = model.isPressed();
            }
        }
    }

    @Override
    public void keyTyped(KeyEvent e) {
        // empty method
    }

    @Override
    public void keyPressed(KeyEvent e) {
        performAction(e);
    }

    @Override
    public void keyReleased(KeyEvent e) {
        writeToOutputStream(" ");
    }

    private void performAction(KeyEvent e) {
        int keyCode = e.getKeyCode();
        switch (e.getKeyCode()) {
            case KeyEvent.VK_W:
                writeToOutputStream(Character.toString((char)keyCode));
                break;
            case KeyEvent.VK_A:
                writeToOutputStream(Character.toString((char)keyCode));
                break;
            case KeyEvent.VK_S:
                writeToOutputStream(Character.toString((char)keyCode));
                break;
            case KeyEvent.VK_D:
                writeToOutputStream(Character.toString((char)keyCode));
                break;
            default:
                break;
        }
    }
}
