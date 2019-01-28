/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package gui;

import com.fazecast.jSerialComm.SerialPort;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
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

    private static final SerialPort SERIAL = SerialPort.getCommPort("COM6");
    private static OutputStream outputStream;
    private static InputStream inputStream;
    private InputListener il;

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

        // Create inputStream listener to keep inputText upto date
        il = new InputListener();
        il.start();
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
        leftRoomBtn.addKeyListener(this);
        rightRoomBtn.addKeyListener(this);
        startButton.addKeyListener(this);
        endBtn.addKeyListener(this);
        continueBtn.addKeyListener(this);
        sendButton.addKeyListener(this);
        stopButton.addKeyListener(this);
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
        leftRoomBtn = new javax.swing.JButton();
        rightRoomBtn = new javax.swing.JButton();
        startButton = new javax.swing.JButton();
        endBtn = new javax.swing.JButton();
        continueBtn = new javax.swing.JButton();
        stopButton = new javax.swing.JButton();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setResizable(false);

        forwardButton.setText("Forward");

        backwardsButton.setText("Back");

        rightButton.setText("Right");

        leftButton.setText("Left");

        inputText.setColumns(20);
        inputText.setRows(5);
        inputText.setFocusable(false);
        inputPane.setViewportView(inputText);

        inputStreamLabel.setText("Input");

        sendButton.setText("Send");
        sendButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                sendButtonActionPerformed(evt);
            }
        });

        outputLabel.setText("Output");

        leftRoomBtn.setText("Room L");
        leftRoomBtn.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                leftRoomBtnActionPerformed(evt);
            }
        });

        rightRoomBtn.setText("Room R");
        rightRoomBtn.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                rightRoomBtnActionPerformed(evt);
            }
        });

        startButton.setText("Start");
        startButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                startButtonActionPerformed(evt);
            }
        });

        endBtn.setText("End");
        endBtn.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                endBtnActionPerformed(evt);
            }
        });

        continueBtn.setText("Continue");
        continueBtn.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                continueBtnActionPerformed(evt);
            }
        });

        stopButton.setText("Stop");
        stopButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                stopButtonActionPerformed(evt);
            }
        });

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(leftButton, javax.swing.GroupLayout.PREFERRED_SIZE, 76, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addGap(74, 74, 74)
                                .addComponent(rightButton, javax.swing.GroupLayout.PREFERRED_SIZE, 76, javax.swing.GroupLayout.PREFERRED_SIZE))
                            .addGroup(layout.createSequentialGroup()
                                .addGap(73, 73, 73)
                                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                                    .addComponent(backwardsButton, javax.swing.GroupLayout.PREFERRED_SIZE, 76, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addComponent(forwardButton, javax.swing.GroupLayout.PREFERRED_SIZE, 76, javax.swing.GroupLayout.PREFERRED_SIZE))))
                        .addGap(18, 18, 18)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(outputField, javax.swing.GroupLayout.PREFERRED_SIZE, 95, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(sendButton))
                            .addComponent(outputLabel)))
                    .addComponent(inputStreamLabel))
                .addGap(8, 8, 8)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createSequentialGroup()
                                .addComponent(leftRoomBtn)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                                .addComponent(rightRoomBtn))
                            .addComponent(endBtn, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                        .addContainerGap())
                    .addGroup(layout.createSequentialGroup()
                        .addGap(28, 28, 28)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING, false)
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(startButton)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                                .addComponent(stopButton))
                            .addComponent(continueBtn, javax.swing.GroupLayout.PREFERRED_SIZE, 146, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))))
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createSequentialGroup()
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addComponent(inputPane, javax.swing.GroupLayout.PREFERRED_SIZE, 589, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addGap(26, 26, 26))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap(21, Short.MAX_VALUE)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                            .addGroup(layout.createSequentialGroup()
                                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                                    .addComponent(startButton, javax.swing.GroupLayout.PREFERRED_SIZE, 23, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addComponent(stopButton))
                                .addGap(14, 14, 14)
                                .addComponent(continueBtn))
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(outputLabel)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                                    .addComponent(outputField, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addComponent(sendButton))))
                        .addGap(18, 18, 18)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(rightRoomBtn)
                            .addComponent(leftRoomBtn)))
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(forwardButton)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(leftButton)
                            .addComponent(rightButton))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(backwardsButton, javax.swing.GroupLayout.PREFERRED_SIZE, 23, javax.swing.GroupLayout.PREFERRED_SIZE)))
                .addGap(18, 18, 18)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addComponent(endBtn)
                    .addComponent(inputStreamLabel))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addComponent(inputPane, javax.swing.GroupLayout.PREFERRED_SIZE, 239, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void sendButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_sendButtonActionPerformed
        // Take the value in the outputText turn into byte array and send down output stream
        String messageToSend = outputField.getText();
        writeToOutputStream(messageToSend);
        // Clear text box
        outputField.setText("");
    }//GEN-LAST:event_sendButtonActionPerformed

    private void leftRoomBtnActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_leftRoomBtnActionPerformed
        writeToOutputStream("RL");
    }//GEN-LAST:event_leftRoomBtnActionPerformed

    private void rightRoomBtnActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_rightRoomBtnActionPerformed
        writeToOutputStream("RR");
    }//GEN-LAST:event_rightRoomBtnActionPerformed

    private void startButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_startButtonActionPerformed
        // stop/start recording
        writeToOutputStream("G");
        // then start moving
        writeToOutputStream("C");
    }//GEN-LAST:event_startButtonActionPerformed

    private void continueBtnActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_continueBtnActionPerformed
        writeToOutputStream("C");
    }//GEN-LAST:event_continueBtnActionPerformed

    private void endBtnActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_endBtnActionPerformed
        // Save temp actions when recording 
        // if and object is 
        writeToOutputStream("E");
    }//GEN-LAST:event_endBtnActionPerformed

    private void stopButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_stopButtonActionPerformed
        writeToOutputStream("0");
    }//GEN-LAST:event_stopButtonActionPerformed

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
    private javax.swing.JButton continueBtn;
    private javax.swing.JButton endBtn;
    private javax.swing.JButton forwardButton;
    private javax.swing.JScrollPane inputPane;
    private javax.swing.JLabel inputStreamLabel;
    private javax.swing.JTextArea inputText;
    private javax.swing.JButton leftButton;
    private javax.swing.JButton leftRoomBtn;
    private javax.swing.JTextField outputField;
    private javax.swing.JLabel outputLabel;
    private javax.swing.JButton rightButton;
    private javax.swing.JButton rightRoomBtn;
    private javax.swing.JButton sendButton;
    private javax.swing.JButton startButton;
    private javax.swing.JButton stopButton;
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
                writeToOutputStream(Character.toString((char) keyCode));
                break;
            case KeyEvent.VK_A:
                writeToOutputStream(Character.toString((char) keyCode));
                break;
            case KeyEvent.VK_S:
                writeToOutputStream(Character.toString((char) keyCode));
                break;
            case KeyEvent.VK_D:
                writeToOutputStream(Character.toString((char) keyCode));
                break;
            case KeyEvent.VK_C:
                writeToOutputStream(" ");
                break;
            case KeyEvent.VK_0:
                writeToOutputStream("0");
            default:
                break;
        }
    }

    // saving commands after the start button has been pressed using a List of this class
    // Listens on the input stream concurrently.
    private class InputListener extends Thread {

        @Override
        public void run() {
            BufferedReader br = new BufferedReader(new InputStreamReader(inputStream));
            while (true) {
                try {
                    char message = (char) br.read();
                    inputText.append("" + message);
                } catch (IOException e) {
                }
            }
        }
    }
}
