import serial
import time
import numpy
import binascii
import math
import sys

from PSOC_cmd import *

# Script for testing the AESOP-Lite system via the Event PSOC

asicReset = True
# Set the number of events to acquire:
numberEvents = 1000
runNumber = 320
numberOfRuns = 3
portName = "COM7"
address = 8   # Address of the event PSOC

exCaught = 0 #count the number of exceptions handled -B
exitOnEx = False #exit on all exceptions, not just critical ones -b

try: #try and catch exceptions during setup -B

    openCOM(portName)

    print("Entering testItEvtAESOP.py at " + time.strftime("%c"))

    #LED2("on", address)
    #time.sleep(1)
    #LED2("off", address)

    #This is supposed to set the real time clock, but it fails the first time after rebooting the PSOC
    setInternalRTC(address)  
except IOError as err:
    print("IO Error # {0} during RTC set: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
    exCaught += 1 #count the number of exceptions handled -B
    sys.exit("critical error") #exit on exception -B
try:
    time.sleep(0.1)
    getInternalRTC(address)

    #The watch battery voltage drains too fast. It is for the i2c real time clock.
    print("The watch battery voltage is " + str(readBatteryVoltage()) + " V")
except IOError as err:
    print("IO Error # {0} during RTC echo: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
    exCaught += 1 #count the number of exceptions handled -B
    if exitOnEx: sys.exit("critical error") #exit on exception -b
try:
    #Set the thresholds for the PMTs
    #Order is G, T3, T1, T4, T2
    #pmtThresh = [3,4,4,4,60]  #PSM: ORIGINAL VALUES 18 July 2021 DO NOT DELETE THIS LINE
    # pmtThresh = [3,5,2,2,60]
    # ch5Thresh = pmtThresh[4]
    # print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")
    # setPmtDAC(5, ch5Thresh, address)


    setTofDAC(1, 49, address)   
    setTofDAC(2, 49, address)

    #Set the thresholds for the PMTs
    #Order is G, T3, T1, T4, T2
    #pmtThresh = [3,4,4,4,60]  #PSM: ORIGINAL VALUES 18 July 2021 DO NOT DELETE THIS LINE
    pmtThresh = [3,4,4,3,60]
    ch5Thresh = pmtThresh[4]
    # print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")
    # setPmtDAC(5, ch5Thresh, address)

    # print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")
    # time.sleep(0.2)
    #For some bizarre reason, this reads back as zero when read the second time, but it really is still set
    # print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")
    for chan in range(1,5):
        setPmtDAC(chan, pmtThresh[chan-1], addrEvnt)
        time.sleep(0.1)
except IOError as err:
    print("IO Error # {0} during DAC set: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
    exCaught += 1 #count the number of exceptions handled -B
    sys.exit("critical error") #exit on exception -b
try:
    for channel in range(1,3):
        print("TOF DAC channel " + str(channel) + " was set to " + str(readTofDAC(channel, address)) + " counts.")
    for chan in range(1,5):
        print("Channel " + str(chan) + " PMT DAC was set to " + str(readPmtDAC(chan, addrEvnt)) + " counts")
        time.sleep(0.1)

    # print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")
    # setPmtDAC(5, ch5Thresh, address)

    print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")

    # Communication with the TOF chip is messed up due to conflict with the Main PSOC, I think.  Needs some work.
    #readTofConfig()

    #ret = ser.read()
    #print(ret)

except IOError as err:
    print("IO Error # {0} during DAC echo: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
    exCaught += 1 #count the number of exceptions handled -B
    if exitOnEx: sys.exit("critical error") #exit on exception -b
try:
    boards = [0,1,2,3,4,5,6,7]
    nTkrBoards = len(boards)
    if nTkrBoards > 0:
        tkrFPGAreset(0x00)
        tkrConfigReset(0x00)

        resetTrackerLogic()
        tkrSetNumLyrs(nTkrBoards)

        readErrors(address)

        #Some bug in reading layers 2 through 7 generates lots of internal PSOC errors, but the right result still gets returned.
        print("Trigger enable status = " + str(triggerEnableStatus()))
        print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(0)) + " for board " + str(0))
        print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(1)) + " for board " + str(1))
        print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(2)) + " for board " + str(2))
        print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(3)) + " for board " + str(3))
        print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(4)) + " for board " + str(4))
        print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(5)) + " for board " + str(5))
        print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(6)) + " for board " + str(6))
        print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(7)) + " for board " + str(7))
        #for brd in boards:
        #    print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(brd)) + " for board " + str(brd))
        readErrors(address)

        #sys.exit("abort")

        tkrSetTriggerSource(0)    # We want the external trigger
        trgsrc = bytes2int(tkrGetTriggerSource(0))
        print("The tracker trigger source is set to " + str(trgsrc))

        if asicReset: tkrAsicPowerOn()

        if asicReset: tkrAsicHardReset(0x1F)
        time.sleep(0.1)
        if asicReset: tkrAsicSoftReset(0x1F)

        #This is rather slow. It has to access all the ASICs 1 by 1 to generate data edges on which to calibrate the timing delays
        #in the FPGA that is receiving data from the ASICs.
        #Layer 5 generates some errors but seems okay.
        #Layer 7 crashed here until I explicitely eliminated its chip 4 from the process. That chip is broken (wire bond?).
        calibrateFPGAinputTiming(0)
        calibrateFPGAinputTiming(1)
        calibrateFPGAinputTiming(2)
        calibrateFPGAinputTiming(3)
        calibrateFPGAinputTiming(4)
        calibrateFPGAinputTiming(5)
        calibrateFPGAinputTiming(6)
        calibrateFPGAinputTiming(7)
     

        #tkrTrigEndStat(1, 1)
        #tkrTrigEndStat(0, 1)
        tkrSetDualTrig(0, 0)

        #Set up the ASIC configurations
        oneShot = 0
        gain = 0
        shaping = 0   #This was set to 1 for 2018, to give a slow shaping time. The fast time constant is much better now.
        bufSpeed = 3
        trigDelay = 6   #This is tuned to match the trigger timing from the PMTs. More fine tuning can be done with the shift
        trigWindow = 1  #register delay in the PSOC code, to delay the trigger by periods of the 24 MHz clock.
        ioCurrent = 2
        maxClust = 10
        tkrLoadASICconfig(0, 31, oneShot, gain, shaping, bufSpeed, trigDelay, trigWindow, ioCurrent, maxClust)
        readErrors(address)
        print("The tracker trigger source is set to " + str(bytes2int(tkrGetTriggerSource(0))))
        readErrors(address)
        for brd in boards:
            tkrGetASICconfig(brd, 3)
            
        #sys.exit("abort")

        #The following monitoring of the tracker power works fine but is kind of slow.
        #for brd in boards:
        #    tkrGetTemperature(brd)
        #    #tkrGetBusVoltage(brd, "flash18")
        #    tkrGetBusVoltage(brd, "fpga12")
        #    tkrGetBusVoltage(brd, "digi25")
        #    #tkrGetBusVoltage(brd, "i2c33")
        #    tkrGetBusVoltage(brd, "analog21")
        #    tkrGetBusVoltage(brd, "analog33")
        #    #tkrGetShuntCurrent(brd, "flash18")
        #    tkrGetShuntCurrent(brd, "fpga12")
        #    tkrGetShuntCurrent(brd, "digi25")
        #    #tkrGetShuntCurrent(brd, "i2c33")
        #    tkrGetShuntCurrent(brd, "analog21")
        #    tkrGetShuntCurrent(brd, "analog33")
        #    tkrGetShuntCurrent(brd, "bias100")

        readErrors(address)

        # Setting of the remaining ASIC registers.
        # The calibration and trigger mask registers are not used here
        # The data mask is just set with all channels on
        hitList = []
        hit = [2, 5]
        hitList.append(hit)
        hit = [1, 18]
        hitList.append(hit)
        hit = [1, 13]
        hitList.append(hit)
        hit = [2, 61]
        hitList.append(hit)
        for brd in boards:
            #tkrSetCalMask(brd, 31, hitList)
            #time.sleep(0.1)
            #tkrGetCalMask(brd, 3)

            #tkrSetDataMask(brd, 31, "mask", hitList)
            #time.sleep(0.1)
            #tkrGetDataMask(brd, 3)

            #tkrSetTriggerMask(brd, 31, "mask", hitList)
            #time.sleep(0.1)
            #tkrGetTriggerMask(brd, 3)

            #tkrSetDAC(brd, 31, "calibration", 60 , "high")
            #tkrGetDAC(brd, 3, "calibration")

            tkrSetDAC(brd, 31, "threshold", 21 , "low")
            tkrGetDAC(brd, 3, "threshold")

            tkrSetDataMask(brd, 31, "unmask", [])
            tkrGetDataMask(brd, 1)

            tkrSetTriggerMask(brd, 31, "unmask", [])
            tkrGetTriggerMask(brd, 3)

        #This is for testing calibration events but only works if there is just a single board to read out.
        if len(boards) == 1:
            triggerDelay = 6
            triggerTag = 0
            sendTkrCalStrobe(0, triggerDelay, triggerTag, True)

            time.sleep(0.01)
            readCalEvent(triggerTag, True)

            tkrGetASICconfig(0, 3)
            readErrors(address)

        #Count tracker noise triggers. This can be done for all layers but is kind of slow
        #for brd in boards:
            # Measure the trigger noise count
        getLyrTrgCnt(4)
except IOError as err:
    print("IO Error # {0} during tracker setup: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
    exCaught += 1 #count the number of exceptions handled -B
    if exitOnEx: sys.exit("critical error") #exit on exception -b
try:

    #Set the main trigger mask
    #Order is T1 T2 T3 T4
    #mask = 0x0C    # T1 & T3 DEBUG -B
    mask = 0x06    # T1 & T3 & T4 now T1 & T4 -B
    #mask = 0x07    # single T1
    #mask = 0x0b    # single T2
    # mask = 0x0d    # single T3
    #mask = 0x0e    # single T4
    print("Setting the first trigger mask to " + str(mask))
    setTriggerMask(1, mask)

    #This sets the size of the coincidence window in clock cycles
    setTriggerWindow(16)

    #There is a second trigger that can be prescaled
    #The tracker trigger is not used, for lack of a suitable cable (TBD)
    prescale = 4
    print("Setting the PMT trigger prescale to " + str(prescale))
    setTriggerPrescale("PMT", prescale)
    mask = 0x00    # T1 & T3 prescaled now All -B
    print("Setting the second trigger mask to " + str(mask))
    setTriggerMask(2, mask)
except IOError as err:
    print("IO Error # {0} during trigger set: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
    exCaught += 1 #count the number of exceptions handled -B
    sys.exit("critical error") #exit on exception -b
try:
    print("Get the trigger mask")
    print("The first trigger mask is set to  " + str(hex(getTriggerMask(1))))
    print("The second trigger mask is set to " + str(hex(getTriggerMask(2))))

except IOError as err:
    print("IO Error # {0} during trigger echo: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
    exCaught += 1 #count the number of exceptions handled -B
    if exitOnEx : sys.exit("critical error") #exit on exception -b
try:
    #Quick test of a counter
    print("Count on channel 2 = " + str(getChannelCount(2)))
    print("send reset pulse")
    logicReset(addrEvnt)
    time.sleep(0.3)
    print("Count on channel 2 = " + str(getChannelCount(2)))
    print("Before run, trigger enable status is " + str(triggerEnableStatus()))
except IOError as err:
    print("IO Error # {0} during final quick check: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
    exCaught += 1 #count the number of exceptions handled -B
    if exitOnEx : sys.exit("critical error") #exit on all exception -b

for x in range(numberOfRuns) :
    # Run a fixed number of events.
    # This program opens a file for each event for a single-event plot (can be commented out)
    # plus an nTuple file with PMT readings, and a text output file with most of the information per event
    # printed in an ASCII format.
    # The gnuplot program is needed for viewing the event plots.
    try:
        ADC, Sigma, TOF, sigmaTOF = limitedRun(runNumber, numberEvents, True) #true reads out tracker vaules
        f2 = open("dataOutput_run" + str(runNumber) + ".txt", "a")

        #Print some end of run summary stuff
        print("Ending the run at " + time.strftime("%c"))
        print("Average ADC values:")
        print("    T1 = " + str(ADC[0]) + " +- " + str(Sigma[0]))
        print("    T2 = " + str(ADC[1]) + " +- " + str(Sigma[1]))
        print("    T3 = " + str(ADC[2]) + " +- " + str(Sigma[2]))
        print("    T4 = " + str(ADC[3]) + " +- " + str(Sigma[3]))
        print("     G = " + str(ADC[4]) + " +- " + str(Sigma[4]))
        print("    Ex = " + str(ADC[5]) + " +- " + str(Sigma[5]))
        print("    TOF = " + str(TOF) + " +- " + str(sigmaTOF))

        f2.write("Ending the run at " + time.strftime("%c")+"\n")
        f2.write("Average ADC values:"+"\n")
        f2.write("     T1: " + str(ADC[0]) + " +- " + str(Sigma[0])+"\n")
        f2.write("     T2: " + str(ADC[1]) + " +- " + str(Sigma[1])+"\n")
        f2.write("     T3: " + str(ADC[2]) + " +- " + str(Sigma[2])+"\n")
        f2.write("     T4: " + str(ADC[3]) + " +- " + str(Sigma[3])+"\n")
        f2.write("      G: " + str(ADC[4]) + " +- " + str(Sigma[4])+"\n")
        f2.write("     Ex: " + str(ADC[5]) + " +- " + str(Sigma[5])+"\n")
        f2.write("   TOF2: " + str(TOF) + " +- " + str(sigmaTOF)+"\n")
    except IOError as err:
        print("IO Error # {0} during limitedRun: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
        exCaught += 1 #count the number of exceptions handled -B
        if exitOnEx : sys.exit("critical error") #exit on all exception -b
    except IndexError as err:
        print("Index Error # {0} during limitedRun: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
        exCaught += 1 #count the number of exceptions handled -B
        if exitOnEx : sys.exit("critical error") #exit on all exception -b
    except ValueError as err:
        print("Value Error # {0} during limitedRun: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
        exCaught += 1 #count the number of exceptions handled -B
        if exitOnEx : sys.exit("critical error") #exit on all exception -b
    try:
        f2 = open("dataOutput_run" + str(runNumber) + ".txt", "a") # added open in case of error, might need to check -B
    except IOError as err:
            print("IO Error # {0} during file open: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
            exCaught += 1 #count the number of exceptions handled -B
            if exitOnEx : sys.exit("critical error") #exit on all exception -b
    chName = ["G","T3","T1","T4","T2"]
    for ch in range(5):    #Get the singles counts from each of the 5 PMTs
        try:
            cnt = getEndOfRunChannelCount(ch+1)
            print("Counter for channel " + chName[ch] + " = " + str(cnt))
            f2.write("Counterforchannel: " + chName[ch] + " = " + str(cnt)+"\n")
        except IOError as err:
            print("IO Error # {0} during Channel {1} Count: {2}".format(exCaught, ch, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
            exCaught += 1 #count the number of exceptions handled -B
            if exitOnEx: sys.exit("critical error") #exit on exception -B

    try:
        readErrors(address)
    except IOError as err:
        print("IO Error # {0} ironically during Read Errors: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
        exCaught += 1 #count the number of exceptions handled -B
        if exitOnEx: sys.exit("critical error") #exit on exception -B

    if nTkrBoards > 0:
        fpgaConfig = "error"
        try:
            fpgaConfig = tkrGetFPGAconfig(0)
        except IOError as err:
            print("IO Error # {0} during Read FPGA Config: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
            exCaught += 1 #count the number of exceptions handled -B
            if exitOnEx : sys.exit("critical error") #exit on all exception -b
        except IndexError as err:
            print("Index Error # {0} during Read FPGA Config: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
            exCaught += 1 #count the number of exceptions handled -B
            if exitOnEx : sys.exit("critical error") #exit on all exception -b
        print("Tracker FPGA configuration = " + str(fpgaConfig))
        f2.write("TrackerFPGAconfiguration: " + str(fpgaConfig +"\n"))
        try:
            if asicReset: tkrAsicPowerOff()
            tkrTriggerDisable()
        except IOError as err:
            print("IO Error # {0} during Power off: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
            exCaught += 1 #count the number of exceptions handled -B
            if exitOnEx: sys.exit("critical error") #exit on exception -B

    try:
        f2.close()
        readErrors(address)
    except IOError as err:
        print("IO Error # {0} ironically during Read Errors: {1}".format(exCaught, err)) #just print for now but might need to end. would be better to break the setup inot critical setup and less important commands and handle the errors accordingly -B 
        exCaught += 1 #count the number of exceptions handled -B
        if exitOnEx: sys.exit("critical error") #exit on exception -B
    runNumber = runNumber + 1

closeCOM()


