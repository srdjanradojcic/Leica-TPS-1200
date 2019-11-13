"""
.. module:: GeoCom

"""
import sys
import serial
import time
from enum import Enum

ser = 0
Debug_Level = 1
GTrId = 0


class AUT_POSMODE(Enum):
    AUT_NORMAL = 0  # fast positioningmode
    AUT_PRECISE = 1
    AUT_Fast = 2


class AUT_ATRMODE(Enum):
    AUT_POSITION = 0,  # Positioning to the hz - and v - angle
    AUT_TARGET = 1  # Positioning to a target of the hz and v angle


class ResponseClass:
    """
    Manage the response from the station (transaction ID and parameters returned) and the
    error codes returned.

    :attribute RC_COM: communication return code
    :attribute TrId: transaction id
    :attribute RC: request return code
    :attribute parameters: list of returned parameters
    """

    RC_COM = 0
    TrId = 0
    RC = 0
    parameters = []

    def setResponse(self, response):
        """
        Instantiate a ResponseClass from an ASCII response.

        :param response: ASCII response from the station
        :type response: ResponseClass
        """
        if (Debug_Level == 2):
            print('response = ', response)
        # remove the ' from the string, remove the end-line character and split it up
        words = response.replace('\'', '').strip().split(',')
        # print words
        if (len(words) > 1):
            self.RC_COM = int(words[1])
            words2 = words[2].split(':')
            self.TrId = int(words2[0])
            self.RC = int(words2[1])
            self.parameters = words[3:len(words)]
            if (self.RC != 0 and Debug_Level == 1):
                print('Problem occurred, Error code: ', self.RC)


class SerialRequestError(Exception):
    """
    Instantiate a SerialRequestError from an Exception.

    :param Exception: An exception occurred during a request
    :type Exception: Exception
    """

    def __init__(self, value):
        """
        Init the Error

        :param value: the message from the exception
        :type value: str
        """
        self.value = value

    def __str__(self):
        """
        Return the message from the error as a string (str)
        """
        return repr(self.value)


def getTrId(request):
    """
    Get transaction ID from an ASCII request by parsing it.

    :param request: an ASCII resquest
    :type request: str
    :returns: parsed transaction ID
    :rtype: int
    """
    words = request.replace('\'', '').strip().split(',')[2].split(':')
    return int(words[0])


def SerialRequest(request, length=0, t_timeout=3):
    """
    Send a request to the server (total station).

    :param request: an ASCII request
    :param length: to remove later, not needed
    :param t_timeout: default is 3 seconds, could be higher or lower

    :returns: the corresponding response
    :rtype: ResponseClass

    :exception SerialRequestError: thrown if an error occurs during the communication
    """
    if (Debug_Level == 2):
        print('request = ', request)
    id = getTrId(request)
    response = ResponseClass()
    global ser

    try:  # try method for the case that TS is not connected
        ser.read(ser.inWaiting())
        ser.write(request + '\r\n')
        t_start = time.time()
        # do as long as:
        # 1: buffer has specific length
        # 2: if specific length not defined (=0), then until buffer > 0
        # 3: timeout not reached
        while ((ser.inWaiting() < length or (
                length == 0 and ser.inWaiting() == 0)) and time.time() - t_start < t_timeout):
            time.sleep(0.001)

        if (time.time() - t_start >= t_timeout):
            response.RC = 3077
            return response

        time.sleep(0.025)  # Short break to make sure serial port is not read while stuff is written
        serial_output = ser.read(ser.inWaiting())
        response.setResponse(serial_output)
        if response.TrId != id:
            response.RC = 3077
            return response
    except KeyboardInterrupt as e:
        raise KeyboardInterrupt(e)
    except:
        raise SerialRequestError("Leica TS communication error - not connected?")
        response.RC = 1
    return response


def HexToDec(hex_in):
    """
    Convert an hexadecimal number into a decimal number.

    :param hex_in: hexadecimal number to convert
    :type hex_in: int
    :returns: decimal representation of hex_in
    :rtype: int
    """
    dec_out = int(hex_in, 16)
    return dec_out


def CreateRequest(cmd, args=None):
    """
    Create an ASCII Request based on a command code and, if needed, corresponding arguments.

    :param cmd: function code to send to the Station
    :param args: list of arguments

    :returns: an ASCII request with this form
        [<LF>]%R1Q,cmd,<TrId>:[args]<Term>
    :rtype: str
    """
    global GTrId
    # \n is LF flag to flush buffer
    request = '\n%R1Q,'
    request = request + str(cmd) + ',' + str(GTrId)
    request = request + ':'

    GTrId += 1
    if GTrId == 8:
        GTrId = 0

    if (args != None):
        if (len(args) > 0):
            for i in range(0, len(args)):
                request = request + str(args[i])
                request = request + ','
            request = request + str(args[-1])
        return request


"""#############################################################################
########################### COM - COMMUNICATION ################################
################################################################################
Communication; functions to access some aspects of TPS1200 control, which are
close to communication.
These functions relate either to the client side or to the server side.
"""

"""
.. module :: GeoCom.com
"""


def COM_OpenConnection(ePort, eRate, nRetries=10):
    """
    [GeoCOM manual **p26**]

    Open a PC serial port and attempts to detect a theodolite based on the given baud rate.

    :param ePort: serial port
    :type ePort: str
    :param eRate: baud rate
    :type eRate: int
    :param nRetries: number of retries to initiate a connection
    :type nRetries: int

    :returns: [error, RC, []]

    * error=0 and RC=0 if the connection attempt was successful
    * error=1 if not

    :rtype: list
    """

    global ser
    try:
        ser = serial.Serial(
            port=ePort,
            baudrate=eRate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

        while (not ser.isOpen()):
            ser.open()
            if (not ser.isOpen()):
                ser.close()

        if (not ser.isOpen() and Debug_Level == 1):
            print('Problem opening port')

        # 0 = everything ok
        return [not ser.isOpen(), ser, 0]

    except Exception as e:
        print("Connection Error - Leica TS not connected?\n")
        print(str(e))
        return [1, 0, []]


def COM_CloseConnection():
    """
    [GeoCOM manual **p27**]

    Close the (current) open port and releases an established connection.

    :returns: [error, RC, []]. error=0 and RC=0 if the request is successful.
    :rtype: list
    """

    global ser
    ser.close()

    if (not ser.isOpen() and Debug_Level == 1):
        print('Problem closing port')

    return [ser.isOpen(), 0, []]


def COM_SwitchOnTPS(eOnMode=2):
    """ [GeoCOM manual **p96**] """

    request = CreateRequest('111', [eOnMode])

    response = SerialRequest(request, 0, 60)

    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Turn on TPS')

    elif (response.RC == 5):
        error = 0
        if (Debug_Level == 1):
            print('TPS already turned on')

    else:
        error = 1
        if (Debug_Level == 1):
            print('Problem turning TPS on')

    return [error, response.RC, []]


def COM_SwitchOffTPS(eOffMode=0):
    """ [GeoCOM manual **p97**] """

    request = CreateRequest('112', [eOffMode])

    response = SerialRequest(request, 0, 60)

    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Shut down TPS')

    else:
        error = 1
        if (Debug_Level == 1):
            print('Error shutting down TPS')

    return [error, response.RC, []]


def COM_GetSWVersion():
    """ [GeoCOM manual **p95**] """

    request = CreateRequest('110', [])

    response = SerialRequest(request, 0, 60)

    if (response.RC == 0):
        error = 0

    else:
        error = 1

    # Print a list [Software release, Software version, Software subversion]
    print(response.parameters)

    return [error, response.RC, []]


"""#############################################################################
########################## CSV - CENTRAL SERVICES ##############################
################################################################################
Central Services; this module provides functions to get or set central/basic
information about the TPS1200 instrument.
"""


def CSV_GetDateTime():
    """
    [GeoCOM manual **p107**]
    """
    DateTime = []

    request = CreateRequest('110', [])

    response = SerialRequest(request, 0, 60)

    response = SerialRequest()

    error = 1
    if (response.RC == 0):
        error = 0

        DateTime = [int(response.parameters[0])]
        for i in range(1, len(response.parameters)):
            DateTime.append(HexToDec(response.parameters[i]))

        if (Debug_Level == 1):
            print('Date and Time: ', DateTime)

    return [error, response.RC, DateTime]


def CSV_GetInstrumentNo():
    """
    Gets the factory defined serial number of the instrument.
    FlexLine Geocom 1.20 page 55
    :return: Serial Number
    """
    DateTime = []

    request = CreateRequest('5003:', [])
    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Serial number : ' + str(response.parameters))

    return [error, response.RC, response.parameters]


def CSV_GetInstrumentName():
    """
    Gets the instrument name, for example: TCRP1201 R300
    FlexLine Geocom 1.20 page 55
    :return: Serial Number
    """

    request = CreateRequest('5004', [])
    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Instrument name : ' + str(response.parameters))

    return [error, response.RC, response.parameters]


def CSV_GetReflectorlessClass():
    """
    This function returns information about the reflectorless and l
    ong range distance measurement (RL) of the instrument.
    FlexLine Geocom 1.20 page 58
    :return: Prism type
    """
    # TODO maybe this is not correct
    request = CreateRequest("5100", [])
    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Prism : ' + str(response.parameters))

    return [error, response.RC, response.parameters]


"""#############################################################################
########################## AUT - AUTOMATION ################################
################################################################################
Automatisation; a module which provides functions like the control of the
Automatic Target Recognition, Change Face function or Positioning functions.
"""

'''
AUT_NORMAL = 0, // fast positioning mode
AUT_PRECISE = 1 // exact positioning mode
'''
'''
AUT_ATRMODE // Possible modes of the target
// recognition
AUT_POSITION = 0, // Positioning to the hz- and v-angle
AUT_TARGET = 1 // Positioning to a target in the
// environment of the hz- and v-angle.
'''


def AUT_MakePositioning(Hz, V, POSMode=0, ATRMode=0, bDummy=0):
    """
    [GeoCOM manual **p49**]
    """

    request = CreateRequest('9027', [Hz, V, POSMode, ATRMode, bDummy])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Going to new position: ', Hz, ',', V)

    return [error, response.RC, []]


def AUT_Search(Hz_Area, V_Area, bDummy=0):
    """
    [GeoCOM manual **p56**]

    Performs an automatic target search within a given area.

    :parem Hz: horizontal search region [rad]
    :type Hz: int
    :param V: vertical search region [rad]
    :type V: int
    :param bDummy: reserved for future use, always set to false (quick reminder: *0* in GeoCOM is considered false)
    :type bDummy: int

    :returns: [error, RC, []]

    * error=0 and RC=0 if the search is successful
    * error=1 and RC=8710 if not

    :rtype: list
    """

    request = CreateRequest('9029', [Hz_Area, V_Area, bDummy])

    response = SerialRequest(request, 0, 120)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Target search successful')
    else:
        if (Debug_Level == 1):
            if (response.RC == 8710):
                print('No target found')

    return [error, response.RC, []]


# Does not work - connection time out...
# seems to be not needed, when the Leica is directed to the prism
def AUT_FineAdjust(dSrchHz=0.1, dSrchV=0.1):
    """
    [GeoCOM manual **p54**]

    Precisely positions the telescope crosshairs onto the target prism.

    :param dSrchHz: Search range Hz-axis [rad]
    :type dSrchHz: float
    :param dSrchV: Search range V-axis [rad]
    :type dSrchV: float

    :returns: [error, RC, []]

    * error=0 and RC=0 if the request is successful
    * error=1 otherwise

    :rtype: list
    """

    request = CreateRequest('9037', [dSrchHz, dSrchV, 0])

    response = SerialRequest(request, 0, 120)

    error = 1
    if (response.RC == 0):
        error = 0

    return [error, response.RC, []]


def AUT_LockIn():
    """
    [GeoCOM manual **p60**]

    If LOCK mode is activated (AUS_SetUserLockState), then the function starts the target tracking.
    The command is only possible if a AUT_FineAdjust command has been previously sent and
    successfully executed.

    :returns: [error, RC, []]

    * error=0 and RC=0 if the lock is successful
    * error=1 if not

    :rtype: list
    """

    request = CreateRequest('9013', [])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Lock successful')

    return [error, response.RC, []]


def AUT_GetSearchArea():
    """
    [GeoCom **p61**]

    Returns the current position and size of the PowerSearch Window.

    :returns: [error, RC, parameters]

    * error=0 and RC=0 if the request is successful
    * error=1 if not
    * parameters contains the position and size of the PowerSearch Window

    :rtype: list
    """
    request = CreateRequest('9042', [])

    response = SerialRequest(request)
    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Parameters: ', response.parameters)

    return [error, response.RC, response.parameters]


def AUT_SetSearchArea(dCenterHz, dCenterV, dRangeHz, dRangeV, bEnabled=1):
    """
    [GeoCom **p62**]

    Define the starting position of the search and its search area,
    then activates these PowerSearch parameters if bEnabled=1.

    :param dCenterHz: starting horizontal angular position
    :type dCenterHz: float
    :param dCenterV: starting vertical angular position
    :type dCenterV: float
    :param dRangeHz: horizontal search window
    :type dRangeHz: float
    :param dRangeV: vertical search window
    :type dRangeV: float
    :param bEnabled: activate (=1) or deactivate(=0) the parameters set for PowerSearch.
    :type bEnabled: int

    :returns: [error, RC, parameters]

    * error=0 and RC=0 if the request is successful
    * error=1 if not

    :rtype: list
    """
    request = CreateRequest('9043', [dCenterHz, dCenterV, dRangeHz, dRangeV, bEnabled])
    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0

    return [error, response.RC, []]


def AUT_PS_EnableRange(bEnable):
    """
    [GeoCom **p65**]

    Enables (bEnable=1) / disables (bEnable=0) the predefined PowerSearch window,
    including the predefined PowerSearch range limits, set by AUT_PS_SetRange.

    :returns: [error, RC, []]

    * error=0 and RC=0 if the request is successful
    * error=1 if not

    :rtype: list
    """

    request = CreateRequest('9048', [bEnable])
    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0

    return [error, response.RC, []]


def AUT_PS_SetRange(lMinDist, lMaxDist):
    """
    [GeoCom **p66**]

    Define the PowerSearch range limits.

    :param lMinDist: range lower bound
    :type lMinDist: float
    :param lMaxDist: range upper bound
    :type lMaxDist: float

    :returns: [error, RC, []]

    * error=0 and RC=0 if the request is successful
    * error=1 if not

    :rtype: list
    """
    request = CreateRequest('9047', [lMinDist, lMaxDist])
    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0

    return [error, response.RC, []]


def AUT_PS_SearchWindow():
    """
    [GeoCom **p67**]

    Start PowerSearch inside the given PowerSearch window,
    defined by AUT_SetSearchArea and inside the optional range given by AUT_PS_SetRange.

    :returns: [error, RC, []]

    * error=0 and RC=0 if the request is successful
    * error=1 if not
    	* RC = 7 if bad arguments were given (e.g. wrong format or wrong number of args)
    	* RC = 26 if function not  successfully completed
        * RC = 8720 if the working area is not defined
        * RC = 8710 if no target was found

    :rtype: list
    """
    request = CreateRequest('9052', [])
    response = SerialRequest(request, 0, 120)
    print(str(response.RC))
    error = 1
    if (response.RC == 0):
        error = 0

    return [error, response.RC, []]


def AUT_ReadToi():
    """
    This command reads the current setting for the positioning tolerances of the Hz- and V-
    nstrument axis. This command is valid for motorized instruments only.
    :return: [error, RC, [Tolerance Hz[double],Tolerance V[double]]
    """
    request = CreateRequest('9008', [])
    response = SerialRequest(request, 0, 120)
    print(str(response.RC))
    error = 1
    if (response.RC == 0):
        error = 0

    return [error, response.RC, []]


def AUT_SetTol(toleranceHz=1.0, toleranceV=1.0):
    """
    This command sets new values for the positioning tolerances of the Hz- and V- instrument axes.
    This command is valid for motorized instruments only.
    The tolerances must be in the range of 1[cc] ( =1.57079 E-06[rad] ) to 100[cc] ( =1.57079 E-04[rad]
    :return: error, RC, []
    """

    request = CreateRequest('9007', [toleranceHz, toleranceV])
    response = SerialRequest(request)
    print(str(response.RC))
    error = 1
    if (response.RC == 0):
        error = 0

    return [error, response.RC, []]


def AUT_ReadTimeout():
    """
    This command reads the current setting for the positioning time out (maximum time to perform positioning).
    :return: error, RC, RC, [TimeoutHz[double], TimeoutV[double]]
    """
    request = CreateRequest('9012', [])
    response = SerialRequest(request)
    print(str(response.RC))
    error = 1
    if (response.RC == 0):
        error = 0

    return [error, response.RC, [response.parameters]]


def AUT_SetTimeout():
    # TODO Maybe in the future
    pass


def AUT_ChangeFace(PosMode, ATRMode):
    """
    This procedure turns the telescope to the other face. If another function is active, for example locking onto a target,
    then this function is terminated and the procedure is executed. If the position mode is set to normal (PosMode = AUT_NORMAL)
    it is allowed that the current value of the compensator measurement is inexact. Positioning precise (PosMode = AUT_PRECISE)
    forces a new compensator measurement. If this measurement is not possible, the position does not take place.
    If ATR mode is activated and the ATR mode is set to AUT_TARGET, the instrument tries to position onto a target in the destination
    area. If LOCK mode is activated and the ATR mode is set to AUT_TARGET, the instrument tries to lock onto a target in the destination
    area.
    :return: error, RC []
    """
    if not isinstance(PosMode, AUT_POSMODE):
        raise TypeError('PosMode must be an instance of AUT_POSMODE Enum')

    if not isinstance(ATRMode, AUT_ATRMODE):
        raise TypeError('ATRMode must be an instance of AUT_ATRMODE Enum')

    request = CreateRequest('9028', [PosMode, ATRMode, 0])
    response = SerialRequest(request)
    error = 1
    if (response.RC == 0):
        error = 0

    return [error, response.RC, []]

def AUT_GetFineAdjustMode():
    """
    This function returns the current activated fine adjust positioning mode. This command is valid for all instruments,
    but has only effects for instruments equipped with ATR.

    9030

    :return:
    """
    request = CreateRequest('9030', [])
    response = SerialRequest(request)
    error = 1
    if (response.RC == 0):
        error = 0

    return [error, response.RC, [response.parameters]]

"""#############################################################################
#################### EDM - Electronic Distance Measurement #####################
################################################################################
Electronic Distance Meter; the module, which measures distances.

"""


def EDM_Laserpointer(eOn=0):
    """
    [GeoCOM manual **p114**]

    Turns on/off the laser pointer of the total station.

    :param eOn: On (*1*) or Off (*0*)
    :type eOn: int

    :returns: [error, RC, []]

    * error=0 and RC=0 if the request is successful
    * error=1 if not

    :rtype: list
    """

    request = CreateRequest('1004', [eOn])

    response = SerialRequest(request, 0, 30)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Laserpointer turned on/off')

    return [error, response.RC, []]


"""#############################################################################
################ TMC - Theodolite Measurement and Calculation ##################
################################################################################
Theodolite Measurement and Calculation; the core module for getting measurement
data.
"""


def TMC_GetStation():
    """
    This function is used to get the co-ordinates of the instrument station
    :return: %R1P,0,0:RC,E0[double],N0[double],H0[double],Hi[double]
    """
    request = CreateRequest("2009", [])
    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Coordinates : ' + str(response.parameters))

    # %R1P,0,0:RC,E0[double],N0[double],H0[double],Hi[double]
    return [error, response.RC, response.parameters]


def TMC_SetStation(e0=0.0, N0=0.0, H0=0.0, Hi=0.0):
    """
    This function is used to get the co-ordinates of the instrument station
    :return: %R1P,0,0:RC,E0[double],N0[double],H0[double],Hi[double]
    """
    request = CreateRequest('2010', [e0, N0, H0, Hi])
    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Coorinates set : ' + str(response.RC))

    # %R1P,0,0:RC,E0[double],N0[double],H0[double],Hi[double]
    return [error, response.RC, []]


def TMC_GetRefractiveMethod():
    """
    This function is used to get the current refraction model.
    :return:
    """
    request = CreateRequest("2091", [])
    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Prism : ' + str(response.parameters))

    return [error, response.RC, response.parameters]


def TMC_SetRefractiveMethod():
    """
    This function is used to set the refraction model.
    :return:
    """
    pass


def TMC_SetOrientation():
    """
    [GeoCOM manual **p148**]

    Orientate the instrument in Hz direction. It is a combination of an angle measurement to
    get the Hz offset and afterwards setting the angle Hz offset in order to orientate onto a target.

    :returns: [error, RC, []]

    * error=0 and RC=0 if the request is successful
    * error=1 if not

    :rtype: list
    """

    request = CreateRequest('2113', [0])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0

    return [error, response.RC, []]


def TMC_DoMeasure(cmd=1, mode=1):  # TMC Measurement Modes in geocom manual p.91
    """
    [GeoCOM manual **p141**]

    Carries out a distance measurement. Please note that this command does not output any values (distances).
    In order to get the values you have to use other measurement functions such as TMC_GetCoordinate , TMC_GetSimpleMea or TMC_GetAngle .

    :param cmd: TMC measurement mode (see **p127** of GeoCOM ref manual)
    :type cmd: int
    :param mode: Inclination sensor measurement mode
    :type mode: int

    :returns: [error, RC, []]

    * error=0 and RC=0 if the request is successful
    * error=1 if not

    :rtype: list
    """

    request = CreateRequest('2008', [cmd, mode])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Measuring successful')

    return [error, response.RC, []]


def TMC_SetEdmMode(mode=6):
    """
    [GeoCOM manual **p167**]

    Set the current measurement mode.

    :param mode: measurement mode
    :type mode: int

    Measurement modes available (check **p127**-**128** for further information) :

    Format : *<int value>*. *<enum constant name>* : *<description>*

    0. EDM_MODE_NOT_USED : Init value
    1. EDM_SINGLE_TAPE : IR Standard Reflector Tape
    2. EDM_SINGLE_STANDARD : IR Standard
    3. EDM_SINGLE_FAST : IR Fast
    4. EDM_SINGLE_LRANGE : LO Standard
    5. EDM_SINGLE_SRANGE : RL Standard
    6. EDM_CONT_STANDARD : Standard repeated measurement
    7. EDM_CONT_DYNAMIC : IR Tacking
    8. EDM_CONT_REFLESS : RL Tracking
    9. EDM_CONT_FAST : Fast repeated measurement
    10. EDM_AVERAGE_IR : IR Average
    11. EDM_AVERAGE_SR : RL Average
    12. EDM_AVERAGE_LR : LO Average
    13. EDM_PRECISE_IR : IR Precise (TS30, TM30)
    14. EDM_PRECISE_TAPE : IR Precise Reflector Tape (TS30, TM30)

    :returns: [error, RC, []]

    * error=0 and RC=0 if the request is successful
    * error=1 if not

    :rtype: list
    """

    request = CreateRequest('2020', [mode])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('EDM Mode set successfully')

    return [error, response.RC, []]


def TMC_GetCoordinate(WaitTime=100, mode=1):
    """ [GeoCOM manual **p130**] """

    coord = []

    request = CreateRequest('2082', [WaitTime, mode])

    response = SerialRequest(request)

    error = 0

    if (len(response.parameters) == 8):

        coord = [float(response.parameters[0]), float(response.parameters[1]), float(response.parameters[2])]

        if (Debug_Level == 1):
            print('Coordinates read successfully: ', coord)

    return [error, response.RC, coord]


def TMC_GetStation(WaitTime=100):
    """ [GeoCOM manual **p155**] """

    coord = []

    request = CreateRequest('2009', [WaitTime])

    response = SerialRequest(request)

    error = 0

    if (len(response.parameters) == 4):

        coord = [float(response.parameters[0]), float(response.parameters[1]), float(response.parameters[2]),
                 float(response.parameters[3])]

        if (Debug_Level == 1):
            print('Station coordinates received successfully! ', coord)

    return [error, response.RC, []]


def TMC_GetSimpleMea(WaitTime=100,
                     mode=1):  # TMC_GetSimpleMea - Returns angle and distance measurement - geocom manual p.95
    """
    [GeoCOM manual **p132**]

    Returns the angles and distance measurement data. This command does not issue a new distance measurement.
    A distance measurement has to be started in advance (call TMC_DoMeasure before this function).
    If no valid distance measurement is available and the distance measurement unit is not activated
    (by TMC_DoMeasure before the TMC_GetSimpleMea call) the angle measurement result is returned
    after the **WaitTime**.

    :param WaitTime: Delay to wait for the distance measurement to finish [ms]
    :type WaitTime: int
    :param mode: Inclination sensor measurement mode
    :type mode: int

    :returns: [error, RC, []]

    * error=0 and RC=0 if the request is successful
    * error=1 if not. In this case RC can be equal to:
        * 1284 : Accuracy of the measurement could not be verified by the system of the total station
        * 1285 : Only the angles of the station could be obtained (no distance measurement available)

    :rtype: list
    """

    coord = []
    request = CreateRequest('2108', [WaitTime, mode])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if len(response.parameters) < 3:
            return [1, 1, []]
        coord = [response.parameters[0], response.parameters[1], response.parameters[2]]
        if (Debug_Level == 1):
            print('Coordinates read successfully: ', coord)
    if (response.RC == 1284):
        error = 1284
        coord = [response.parameters[0], response.parameters[1], response.parameters[2]]
        if (Debug_Level == 1):
            print('Accuracy could not be verified: ', coord)
    if (response.RC == 1285):
        error = 1285
        coord = [response.parameters[0], response.parameters[1]]
        if (Debug_Level == 1):
            print('Angles read successfully: ', coord)

    return [error, response.RC, coord]


def TMC_QuickDist():
    """ [GeoCOM manual **p138**] """

    coord = []
    request = CreateRequest('2117')
    response = SerialRequest(request)
    error = 1
    if (response.RC == 0):
        error = 0
        coord = [response.parameters[0], response.parameters[1], response.parameters[2]]

    return [error, response.RC, coord]


def TMC_GetAngle(mode=1):
    """ Refer to *TMC_GetAngle5* in GeoCOM manual **p136** """
    coord = []
    request = CreateRequest('2107', [mode])

    response = SerialRequest(request)
    error = 1

    if (len(response.parameters) == 2):

        if (response.RC == 0):
            error = 0
            coord = [response.parameters[0], response.parameters[1]]

    return [error, response.RC, coord]


def TMC_GetEdmMode():
    """ """

    #    EDM_MODE = {0 : 'EDM_MODE_NOT_USED',
    #                1 : 'EDM_SINGLE_TAPE',
    #                2 : 'EDM_SINGLE_STANDARD',
    #                3 : 'EDM_SINGLE_FAST',
    #                4 : 'EDM_SINGLE_LRANGE',
    #                5 : 'BAP_CONT_REF_FAST',
    #                6 : 'BAP_CONT_RLESS_VISIBLE',
    #                7 : 'BAP_AVG_REF_STANDARD',
    #                8 : 'BAP_AVG_REF_VISIBLE',
    #                9 : 'BAP_AVG_RLESS_VISIBLE',
    #                10 :'BAP_CONT_REF_SYNCHRO',
    #                11 :'BAP_SINGLE_REF_PRECISE'}
    ##    EDM_MODE :
    ##        EDM_MODE_NOT_USED       0, // Init value
    ##                 1, // IR Standard Reflector Tape
    ##             2, // IR Standard
    ##                 3, // IR Fast
    ##               4, // LO Standard
    ##        EDM_SINGLE_SRANGE       5, // RL Standard
    ##        EDM_CONT_STANDARD       6, // Standard repeated measurement
    ##        EDM_CONT_DYNAMIC        7, // IR Tacking
    ##        EDM_CONT_REFLESS        8, // RL Tracking
    ##        EDM_CONT_FAST           9, // Fast repeated measurement
    ##        EDM_AVERAGE_IR          10,// IR Average
    ##        EDM_AVERAGE_SR          11,// RL Average
    ##        EDM_AVERAGE_LR          12,// LO Average
    ##        EDM_PRECISE_IR          13,// IR Precise (TS30, TM30)
    ##        EDM_PRECISE_TAPE        14,// IR Precise Reflector Tape (TS30, TM30)
    request = CreateRequest('2021', [])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('EDM Mode read successfully: ')

    return [error, response.RC, response.parameters]


#
# def TMC_SetEdmMode(mode) :
#
##    EDM_MODE :
##        EDM_MODE_NOT_USED       0, // Init value
##        EDM_SINGLE_TAPE         1, // IR Standard Reflector Tape
##        EDM_SINGLE_STANDARD     2, // IR Standard
##        EDM_SINGLE_FAST         3, // IR Fast
##        EDM_SINGLE_LRANGE       4, // LO Standard
##        EDM_SINGLE_SRANGE       5, // RL Standard
##        EDM_CONT_STANDARD       6, // Standard repeated measurement
##        EDM_CONT_DYNAMIC        7, // IR Tacking
##        EDM_CONT_REFLESS        8, // RL Tracking
##        EDM_CONT_FAST           9, // Fast repeated measurement
##        EDM_AVERAGE_IR          10,// IR Average
##        EDM_AVERAGE_SR          11,// RL Average
##        EDM_AVERAGE_LR          12,// LO Average
##        EDM_PRECISE_IR          13,// IR Precise (TS30, TM30)
##        EDM_PRECISE_TAPE        14,// IR Precise Reflector Tape (TS30, TM30)
#
#    request = CreateRequest('2020',[mode])
#
#    response = SerialRequest(request)
#
#    error = 1
#    if(response.RC==0) :
#        error = 0
#        if(Debug_Level==1) :
#            print 'EDM Mode set successfully'
#
#    return [error, response.RC, []]

"""#############################################################################
########################## MOT - Motorization ##################################
################################################################################
Motorization; the part, which can be used to control the movement and the speed
of movements of the instrument.
"""

'''
enum MOT_MODE #GeoCom manual p83
{
MOT_POSIT = 0, // configured for relative postioning
MOT_OCONST = 1, // configured for constant speed
// the only valid mode
// for SetVelocity is MOD_OCONST
MOT_MANUPOS = 2, // configured for manual positioning
// default setting
MOT_LOCK = 3, // configured as "Lock-In"-controller
MOT_BREAK = 4, // configured as "Brake"-controller
// do not use 5 and 6
MOT_TERM = 7 // terminates the controller task

'''


def MOT_StartController(ControlMode=1):
    """ [GeoCOM manual **p119**] """
    request = CreateRequest('6001', [ControlMode])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Motor controller started')

    return [error, response.RC, []]


'''
enum MOT_STOPMODE #GeoCom manual p83
{
MOT_NORMAL = 0, // slow down with current acceleration
MOT_SHUTDOWN = 1 // slow down by switch off power supply
'''


def MOT_StopController(Mode=0):
    """ [GeoCOM manual **p120**] """

    request = CreateRequest('6000', [Mode])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Motor controller stopped')

    return [error, response.RC, []]


'''

'''


def MOT_SetVelocity(Hz_speed, v_speed):
    """ [GeoCOM manual **p121**] """

    request = CreateRequest('6004', [Hz_speed, v_speed])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Velocity set')

    return [error, response.RC, []]


"""#############################################################################
######################## BAP - Basic Applications ##############################
################################################################################
Basic Applications; some functions, which can easily be used to get measuring
data.
"""

BAP_TARGET_TYPE = {0: 'BAP_REFL_USE',  # with reflector
                   1: 'BAP_REFL_LESS'}  # without reflector


def BAP_GetTargetType():
    """ [GeoCOM manual **p71**] """
    parameter = []
    request = CreateRequest('17022', [])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        parameter = response.parameters[0]

        if (Debug_Level == 1):
            print('Target type: ', BAP_TARGET_TYPE[int(response.parameters[0])][1])

    return [error, response.RC, parameter]


def BAP_SetTargetType(eTargetType=0):
    """ [GeoCOM manual **p72**] """

    request = CreateRequest('17021', [eTargetType])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Target type set successfully ')

    return [error, response.RC, []]


# BAP_TARGET_TYPE
# BAP_REFL_USE = 0 // with reflector
# BAP_REFL_LESS = 1 // without reflector

BAP_PRISMTYPE = {0: ['BAP_PRISM_ROUND', 'Leica Circular Prism'],
                 1: ['BAP_PRISM_MINI', 'Leica Mini Prism'],
                 2: ['BAP_PRISM_TAPE', 'Leica Reflector Tape'],
                 3: ['BAP_PRISM_360', 'Leica 360 Prism'],
                 4: ['BAP_PRISM_USER1', 'not supported'],
                 5: ['BAP_PRISM_USER2', 'not supported'],
                 6: ['BAP_PRISM_USER3', 'not supported'],
                 7: ['BAP_PRISM_360_MINI', 'Leica Mini 360 Prism'],
                 8: ['BAP_PRISM_MINI_ZERO', 'Leica Mini Zero Prism'],
                 9: ['BAP_PRISM_USE', 'User Defined Prism'],
                 10: ['BAP_PRISM_NDS_TAPE', 'Leica HDS Target'],
                 11: ['BAP_PRISM_GRZ121_ROUND', 'GRZ121 360 Prism for Machine Guidance'],
                 12: ['BAP_PRISM_MA_MP3122', 'MPR122 360 Prism for Machine Guidance']}


def BAP_GetPrismType():
    """ [GeoCOM manual **p73**] """
    parameter = []

    request = CreateRequest('17009', [])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        parameter = [response.parameters[0], BAP_PRISMTYPE[int(response.parameters[0])][1]]
        if (Debug_Level == 1):
            print('Prism type: ', BAP_PRISMTYPE[int(response.parameters[0])][1])

    return [error, response.RC, parameter]


def BAP_SetPrismType(ePrismType):
    """
    [GeoCOM manual **p74**]

    Sets the prism type for measurements with a reflector.
    Check **p69** of GeoCOM manual for all prism types.
    Prism types used with the main script:

    * BAP_PRISM_360 (value=3) : Leica 360 Prism
    * BAP_PRISM_360_MINI (value=7) : Leica Mini 360 Prism

    :param PrismType: Constant associated to a prism type
    :type PrismType: int

    :returns: [error, RC, []]

    * error=0 and RC=0 if the request is successful
    * error=1 if not

    :rtype: list
    """

    request = CreateRequest('17008', [ePrismType])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Prism type set')

    return [error, response.RC, []]


def BAP_SetMeasPrg(eMeasPrg):
    """ [GeoCOM manual **p81**] """

    request = CreateRequest('17019', [eMeasPrg])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Measurement program set')

    return [error, response.RC, []]


BAP_USER_MEASPRG = {0: ['BAP_SINGLE_REF_STANDARD', 'Reflector, Standard'],
                    1: ['BAP_SINGLE_REF_FAST', 'Reflector, Fast'],
                    2: ['BAP_SINGLE_REF_VISIBLE', 'Long Range, Standard'],
                    3: ['BAP_SINGLE_RLESS_VISIBLE', 'No Reflector, Standard'],
                    4: ['BAP_CONT_REF_STANDARD', 'Reflector, Tracking'],
                    5: ['BAP_CONT_REF_FAST', 'not available'],
                    6: ['BAP_CONT_RLESS_VISIBLE', 'No Reflector, Fast Tracking'],
                    7: ['BAP_AVG_REF_STANDARD', 'Reflector, Average'],
                    8: ['BAP_AVG_REF_VISIBLE', 'Long Range, Average'],
                    9: ['BAP_AVG_RLESS_VISIBLE', 'No Reflector, Average'],
                    10: ['BAP_CONT_REF_SYNCHRO', 'Reflector, Synchro Tracking'],
                    11: ['BAP_SINGLE_REF_PRECISE', 'not available']}


def BAP_MeasDistanceAngle(mode=6):
    """ [GeoCOM manual **p82**] """
    coord = []

    request = CreateRequest('17017', [mode])

    response = SerialRequest(request)

    error = None

    if (len(response.parameters) == 4):

        coord = [float(response.parameters[0]), float(response.parameters[1]), float(response.parameters[2]),
                 int(response.parameters[3])]

        if (Debug_Level == 1):
            print('Got data successfully: ', coord)

    return [error, response.RC, coord]


def BAP_GetMeasPrg():
    """ [GeoCOM manual **p80**] """

    parameter = []

    request = CreateRequest('17018', [])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        parameter = [response.parameters[0], BAP_USER_MEASPRG[int(response.parameters[0])][1]]
        if (Debug_Level == 1):
            print('Measurement program: ', BAP_USER_MEASPRG[int(response.parameters[0])][1])

    return [error, response.RC, parameter]


def BAP_SearchTarget(bDummy=0):
    """ [GeoCOM manual **p84**] """

    request = CreateRequest('17020', [bDummy])

    response = SerialRequest(request, 0, 10)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Prism found!')

    else:
        if (Debug_Level == 1):

            if (response.RC == 8710):
                print('No prism found!')

            elif (response.RC == 8711):
                print('Multiple prism found!')

    return [error, response.RC, []]


"""#############################################################################
############################## AUS - ALT User ##################################
################################################################################
The subsystem "Alt User" mainly contains functions behind the "SHIFT" + "USER"
button.
"""


def AUS_SetUserLockState(on=0):
    """ [GeoCOM manual **p42**] """

    request = CreateRequest('18007', [on])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Lock activated or deactivated')

    return [error, response.RC, []]


def AUS_SetUserAtrState(on=0):
    """ [GeoCOM manual **p40**] """

    request = CreateRequest('18005', [on])

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('ATR activated or deactivated')

    return [error, response.RC, []]


def AUS_GetUserLockState():
    """ [GeoCOM manual **p41**] """

    request = CreateRequest('18008')

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Lock activated or deactivated')

    return [error, response.RC, response.parameters]


def AUS_GetUserAtrState():
    """ [GeoCOM manual **p39**] """

    request = CreateRequest('18006')

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('ATR activated or deactivated')

    return [error, response.RC, response.parameters]

""""##############################################################################
################# BASIC MAN MACHINE INTERFACE – BMM #############################
##############################################################################"""

def  BMM_BeepAlarm():
    """
    This function produces a triple beep with the configured intensity and frequency,
    which cannot be changed. If there is a continuous signal active, it will be stopped before.
    :return:
    """
    request = CreateRequest('11004')

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Beep 3 times \n')

    return [error, response.RC, []]

def BMM_BeepNormal():
    """
    This function produces a single beep with the configured intensity and frequency, which cannot be changed.
    If a continuous signal is active, it will be stopped first.
    :return:
    """
    request = CreateRequest('11003')

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Beep once \n')

    return [error, response.RC, []]

def BMM_BeepOn(nIntens = 10):
    """
    This function switches on the beep-signal with the intensity nIntens.
    If a continuous signal is active, it will be stopped first. Turn off the beeping device with IOS_BeepOff.
    :return:
    """
    request = CreateRequest('20001')

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Beep once \n')

    return [error, response.RC, []]

def IOS_BeepOff():
    """
    This function switches off the beep-signal.
    :return:
    """
    request = CreateRequest('20000')

    response = SerialRequest(request)

    error = 1
    if (response.RC == 0):
        error = 0
        if (Debug_Level == 1):
            print('Beep once \n')

    return [error, response.RC, []]
