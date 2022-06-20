from DR3_main import *


class UIFunctions(MainWindow):
    #btn_Setting
    def toggleMenu_setting(self, maxWidth, enable):
        if enable:
            # GET WIDTH
            width = self.ui.frame_menu.width()
            maxExtend = maxWidth
            standard = 65

            #SET MAX WIDTH
            if width == 65:
                self.ui.connection.show()
                self.ui.group_name.show()
                self.ui.simulation.show()
                widthExtended = maxExtend
            else:
                self.ui.connection.hide()
                self.ui.group_name.hide()
                self.ui.simulation.hide()
                widthExtended = standard
            # ANIMATION
            self.animation = QPropertyAnimation(self.frame_menu, b"minimumWidth")
            self.animation.setStartValue(width)
            self.animation.setDuration(450)
            self.animation.setEndValue(widthExtended)
            self.animation.setEasingCurve(QtCore.QEasingCurve.InOutQuart)
            self.animation.start()

    def shadow_effect(self, widget):
        #ref: https://doc.qt.io/qt-5/qgraphicsdropshadoweffect.html
        # creating a QGraphicsDropShadowEffect object
        self.shadow = QGraphicsDropShadowEffect(self)
        # setting blur radius
        self.shadow.setBlurRadius(20)
        self.shadow.setXOffset(0)
        self.shadow.setYOffset(0)
        self.shadow.setColor(QColor(0, 0, 0, 60))
        # adding shadow to the label
        widget.setGraphicsEffect(self.shadow)

    def uiDefinitions(self):
        UIFunctions.list_port(self)
        self.ui.time_respond.setText("3 (s)")   #QLineEdit
        self.ui.mode_check.setChecked(False)    #QCheckbox
        self.ui.btn_start.setEnabled(False)     #QPushButton
        self.ui.connection.hide()               #QGroupBox
        self.ui.group_name.hide()               #QGroupBox
        self.ui.simulation.hide()               #QGroupBox
        UIFunctions.shadow_effect(self, self.ui.Toggle_menu)        #QFrame
        UIFunctions.shadow_effect(self, self.ui.Universal_Display)  #QWidget
        UIFunctions.shadow_effect(self, self.ui.frame_time_adjust)  #QFrame
        UIFunctions.shadow_effect(self, self.ui.frame_current_DOF)  #QFrame
        UIFunctions.shadow_effect(self, self.ui.frame_current_pos)  #QFrame
        UIFunctions.shadow_effect(self, self.ui.frame_button)       #QFrame
        UIFunctions.shadow_effect(self, self.ui.btn_plus)           #QPushButton   
        UIFunctions.shadow_effect(self, self.ui.btn_minus)          #QPushButton
        UIFunctions.shadow_effect(self, self.ui.btn_home)           #QPushButton
        UIFunctions.shadow_effect(self, self.ui.btn_reset)          #QPushButton
        # initialize parameter: 20 25 10. 5 10 5
        self.length1.setText("5.5")
        self.length2.setText("15")
        self.length3.setText("15")
        self.solution.setText("1")
        # UIFunctions.Update_value(self)
        # self.ui.mode_check.setChecked(False)

    #the1_adjust, the2_adjust, the3_adjust
    def valuechange(self):
        self.ui.out_workspace.setText("")
        value_the1 = str(self.ui.the1_adjust.value())
        value_the2 = str(self.ui.the2_adjust.value())
        value_the3 = str(self.ui.the3_adjust.value())
        self.ui.the1_set.setText(value_the1)
        self.ui.the2_set.setText(value_the2)
        self.ui.the3_set.setText(value_the3)

    #btn_plus
    def timechange_plus(self):
        time = self.ui.time_respond.text().split()
        plus = int(time[0]) + 1
        if 0 < plus and plus < 20:
            self.ui.time_respond.setText(str(plus) + " (s)")
        else:
            pass
    #btn_minus
    def timechange_minus(self):
        time = self.ui.time_respond.text().split()
        minus = int(time[0]) - 1
        if 0 < minus and minus < 20:
            self.ui.time_respond.setText(str(minus) + " (s)")
        else:
            pass

    def length_change(self):
        self.ui.length1.setText("5.5") #50
        self.ui.length2.setText("15") #40 
        self.ui.length3.setText("15") #30
        self.link = [
            float(self.length1.text()),
            float(self.length2.text()),
            float(self.length3.text()),
        ]

    def Update_value(self):
        self.ui.the1_adjust.setValue(int(self.ui.the1_set.text()))
        self.ui.the2_adjust.setValue(int(self.ui.the2_set.text()))
        self.ui.the3_adjust.setValue(int(self.ui.the3_set.text()))

    def reset(self):
        self.ui.the1_current.clear()
        self.ui.the2_current.clear()
        self.ui.the3_current.clear()

        self.ui.the1_set.clear()
        self.ui.the2_set.clear()
        self.ui.the3_set.clear()

        self.ui.length1.clear()
        self.ui.length2.clear()
        self.ui.length3.clear()

        self.ui.the1_adjust.setValue(0)
        self.ui.the2_adjust.setValue(0)
        self.ui.the3_adjust.setValue(0)

        self.ui.xpos.clear()
        self.ui.ypos.clear()
        self.ui.zpos.clear()

        self.ui.mode_check.setChecked(False)

    #mode_check
    def simulation_check(self):
        if self.ui.mode_check.isChecked():
            self.btn_start.setEnabled(True)
        else:
            self.btn_start.setEnabled(False)

    # NEW DEF
    def Serial_connect(self,comm,baud):
        self.ser.port = comm
        self.ser.baudrate =  baud
        self.ser.bytesize = serial.EIGHTBITS #number of bits per bytes
        self.ser.parity = serial.PARITY_NONE #set parity check: no parity
        self.ser.stopbits = serial.STOPBITS_ONE #number of stop bits            #timeout block read
        self.ser.xonxoff = False     #disable software flow control
        self.ser.rtscts = False     #disable hardware (RTS/CTS) flow control
        self.ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
        self.ser.writeTimeout = 0    #timeout for write
        self.ser.timeout =0
        self.ser.open()

    def list_port(self):
        ports = serial.tools.list_ports.comports()
        self.commPort =([comport.device for comport in ports])
        self.numConnection = len(self.commPort)
        if  self.numConnection == 0 :
            pass
        elif self.numConnection == 1 :
            self.port_arduino.addItem(str(self.commPort[0]))
        else:
            self.port_arduino.addItem(str(self.commPort[0]))
            self.port_arduino.addItem(str(self.commPort[1]))

    def connect_arduino_clicked(self):
        comport = self.port_arduino.currentText()
        baurate = self.baud_arduino.currentText()
        self.ui.mode_check.setChecked(False)
        self.ui.mode_check.setEnabled(False) #test lai chuc nang nay
        self.ui.btn_start.setEnabled(False)
        
        UIFunctions.Serial_connect(self,comport,baurate)

        if self.ser.isOpen():
            self.ui.check_arduino.setChecked(True)
            self.ui.label_arduino.setText('Connected')
            # if self.mode_check.isChecked() == False:
            #     self.start_worker_streamdata()
            #     time.sleep(0.1)
            #     self.start_worker_read()

            self.the = [0.0,0.0,0.0]
            #Tinh toan vi tri EE va lam tron mang
            position = np.round_(self.Robot.Forward_kinematis(self.the), 2)
            self.ui.xpos.setText(str(position[0]))
            self.ui.ypos.setText(str(position[1]))
            self.ui.zpos.setText(str(position[2]))

            self.btnconnect_arduino.setStyleSheet('QPushButton {background-color:#1eff1e; color: white;}') 
            self.btn_disconnect_arduino.setStyleSheet('QPushButton {background-color:#1b1d23; color: white;}') 
            print("Connected Arduino")
                        
    def disconnect_arduino_clicked(self):
        self.ser.close()
        if not self.ser.isOpen():
            self.mode_check.setEnabled(True)
            self.btnconnect_arduino.setStyleSheet('QPushButton {background-color:#1b1d23; color: white;}') 
            self.btn_disconnect_arduino.setStyleSheet('QPushButton {background-color:#ff1e00; color: white;}')
            self.check_arduino.setChecked(False)
            self.label_arduino.setText('Disconnected')
            print("Disconnected")
            self.xpos.clear()       
            self.ypos.clear()
            self.zpos.clear()
            self.the1_current.clear()
            self.the2_current.clear()
            self.the3_current.clear()
            self.port_arduino.setCurrentIndex(0)
    
