'''
Created on Sep 20, 2016

@author: Loc
'''
import serial

#####################################################################################################################################################
#
#
#####################################################################################################################################################
def scanCOMPorts():
    available = []
    for i in range(256):
        try:
            s = serial.Serial(i)
            available.append( (i, s.portstr))
            s.close()   # explicit close 'cause of delayed GC in java
        except serial.SerialException:
            pass
    return available

#####################################################################################################################################################
#
#
#####################################################################################################################################################
def selectComport(availableCommPorts):
    i = 0
    print(" ")
    print("Scanning for Available COM Ports.....")
    print("Available COM Ports:")
    for port in availableCommPorts:
        print "(%d) %s" %(i,port)
        i = i + 1

    print(" ")
    userSelectedComPort = raw_input('Select a COM port to open (0 to %d):' %(i-1))
    while((userSelectedComPort == "") or (int(userSelectedComPort) > len(availableCommPorts))):
        userSelectedComPort = raw_input('Incorrect Entry.  Select a COM port to open (0 to %d):')

    print("COM Port Selected: %s" %(availableCommPorts[0][1]))
    return userSelectedComPort

#####################################################################################################################################################
#
#
#####################################################################################################################################################
def parseString(text):
    mlist = text.split(",")
    return int(mlist[0]),int(mlist[1]),int(mlist[2]),int(mlist[3])
#####################################################################################################################################################
#
#
#####################################################################################################################################################
def startGetDataTimer():
    getZynqPeripheralDataTimer.start(200)



def getDataHandler():
    if (commandMenuComboBox.currentIndex()== 0):
        getWaveform()
    elif (commandMenuComboBox.currentIndex() == 1):
        getSpectrum()
    else:
        print ("invalid command... should generate exception here to debug")
#####################################################################################################################################################
#
#
#####################################################################################################################################################    
def getSpectrum():
    global numberOfPass,NFFTPoints,ADCSamplingFrequency
    
    numberOfPass = 0
    if (numberOfPass == 0):
        Frequency = np.zeros((NFFTPoints,1))
        adcSammples = np.zeros((NFFTPoints,1))
        RqPacket = bytearray([0x02,0xF7,0x00,0x00,0xF9,0x03])
        ser.flushInput()
        ser.write(RqPacket)
        response = ser.read(NFFTPoints)
        for i in range(0,len(response)):
            temp = ord(response[i])                  #(ord(response[2*i+1]) << 8) | ord(response[2*i]) 
            if(temp > 127):
                temp = temp - 256
#             print("sect0: 0x%X, i:%d" %(temp,i))
#             print("%i, %d" %(i,temp))
            dataLog.write(str(temp) + "\n")
            Frequency[i] = i*1000000/NFFTPoints
            adcSammples[i] = temp
            
        GraphWin1.setLabel('left',"dBVV")
        GraphWin1.setLabel('bottom',"Frequency, Hz")
        GraphWin1.setXRange(0, 4096)
        GraphWin1.setYRange(0, np.amax(adcSammples))       
        GraphWin1Plot1.setData(Frequency[0:4095,0],adcSammples[0:4095,0])   
        text = textBox.displayText()
        [xmin,xmax,ymin,ymax] = parseString(text)
        if (xmin < xmax):
            GraphWin1.setXRange(xmin, xmax)
        if (ymin < ymax):
            GraphWin1.setYRange(ymin, ymax)               

#####################################################################################################################################################
#
#
##################################################################################################################################################### 
def getWaveform():
    global numberOfPass,NFFTPoints,FontTimesSize20
    
    numberOfPass = 0
    if (numberOfPass == 0):
        sampleIndex = np.zeros((NFFTPoints,1))
        adcSammples = np.zeros((NFFTPoints,1))
        RqPacket = bytearray([0x02,0xF8,0x00,0x00,0xFA,0x03])
        ser.flushInput()
        ser.write(RqPacket)
        response = ser.read(NFFTPoints<<1)
        for i in range(0,len(response)>>1):
            temp = (ord(response[2*i+1]) << 8) | ord(response[2*i]) 
            temp = temp*(3.0/65536.0)
#             print("sect0: 0x%X, i:%d" %(temp,i))
            dataLog.write(str(temp) + "\n")
            sampleIndex[i] = i
            adcSammples[i] = temp
 
        GraphWin1.setLabel('left',"Volt")
        GraphWin1.setLabel('bottom',"Sample, n")
        GraphWin1.setXRange(0, NFFTPoints)
        GraphWin1.setYRange(0, np.amax(adcSammples))       
        GraphWin1Plot1.setData(sampleIndex[0:NFFTPoints-1,0],adcSammples[0:NFFTPoints-1,0]) 
        text = textBox.displayText()
        [xmin,xmax,ymin,ymax] = parseString(text)
        if (xmin < xmax):
            GraphWin1.setXRange(xmin, xmax)
        if (ymin < ymax):
            GraphWin1.setYRange(ymin, ymax)     
#         numberOfPass += 1
          
#####################################################################################################################################################
#
#
#####################################################################################################################################################     
if __name__ == '__main__':
    
    dataLog = open("dataLog.dat","w")
    from pyqtgraph.Qt import QtGui, QtCore
    import numpy as np
    import struct
    import pyqtgraph as pyGraph
    
    NFFTPoints = 4096
    ADCSamplingFrequency = 1000000 # sample sampling frequency (from XADC on the Zynq)
    numberOfPass = 0
    app = QtGui.QApplication([])
    windowPane = QtGui.QWidget()
    windowPane.resize(800,600)
    
    FontTimesSize20 = QtGui.QFont("Times", 20, QtGui.QFont.Bold)
             
    availableCommPorts = scanCOMPorts()     
    userSelectedComPort = selectComport(availableCommPorts)
    # available baudrates: 230400,921600
    ser = serial.Serial(port=availableCommPorts[int(userSelectedComPort)][1],baudrate=921600,parity=serial.PARITY_NONE,bytesize=serial.EIGHTBITS,stopbits=serial.STOPBITS_ONE,timeout=1.0)
    print ("Open Comport %s." %(ser.portstr))       # check which port was really used

    commandMenu = ["time waveform","spectrum"]
    commandMenuComboBox = QtGui.QComboBox()
    commandMenuComboBox.setFont(FontTimesSize20)
    model = commandMenuComboBox.model()
    for row in range(len(commandMenu)):
        item = QtGui.QStandardItem(str(row))
        item.setForeground(QtGui.QColor('blue'))
        item.setFont(FontTimesSize20)
        item.setText(commandMenu[row])
        model.appendRow(item)

# use QtGui to create push buttons and text box
    getDataBtn = QtGui.QPushButton('Get Data')
    getDataBtn.setFont(FontTimesSize20)
    getDataBtn.clicked.connect(startGetDataTimer)
    getDataBtn.setStyleSheet("Text-align:left")
    
    textBox = QtGui.QLineEdit()
    textBox.setFont(FontTimesSize20)
    textBox.setText('200,400,0,1')
    
    GraphWin1 = pyGraph.PlotWidget(background=(100,100,100))
    GraphWin1.showGrid(True,True,0.25)
    GraphWin1Plot1 = GraphWin1.plot([0],[0],pen='r',symbol='o',symbolPen='r',symbolBrush=None)
#     FFTPeakCrosshairMax = GraphWin1.plot([0],[0],pen=None,symbol = 'd',symbolPen='r',symbolBrush='w')
    # add more plot(i.e. traces) to GraphWin1....
    
#Populate the buttons and text box on the window pane.
    wPLayout = QtGui.QGridLayout()
    windowPane.setLayout(wPLayout)
     
# add layout to window pane0
    wPLayout.addWidget(GraphWin1,             0,0,50,100)
    wPLayout.addWidget(getDataBtn,            51,45,1,1) 
    wPLayout.addWidget(commandMenuComboBox,   51,50,1,1)
    wPLayout.addWidget(textBox,               53,0,1,100)
           
    getZynqPeripheralDataTimer = QtCore.QTimer()
    getZynqPeripheralDataTimer.timeout.connect(getDataHandler)
    getZynqPeripheralDataTimer.stop()
        
    windowPane.show()     
    app.exec_()
#     sys.exit(app.exec_())
