# bap1
Motor
using System;
using System.Threading;
using System.Timers;
using FieldTalk.Modbus.Master;
using LightUI;
using PressureMeasurementApp.Models.Common;
using Timer = System.Timers.Timer;

namespace PressureMeasurementApp.Models.Protocols.Modbus;

public class ModbusServer : IDisposable
{
    #region Constructor
    public ModbusServer(string comPort, IEventAggregator eventAggregator, byte servoId, byte displayId)
    {
        try
        {
            ComPort = comPort;
            ServoID = servoId;
            DisplayID = displayId;
            RegisterConverter = new RegisterConverter();
            RtuMasterProtocol = new MbusRtuMasterProtocol();
            RtuMasterProtocol.configureCountFromZero();
            RtuMasterProtocol.setTimeout(5000);

            var result = RtuMasterProtocol.openProtocol(
                comPort,
                115200,
                MbusSerialClientBase.SER_DATABITS_8,
                MbusSerialClientBase.SER_STOPBITS_1,
                MbusSerialClientBase.SER_PARITY_EVEN);

            if (result != BusProtocolErrors.FTALK_SUCCESS)
                throw new UnauthorizedAccessException("Připojení RTU selhalo");

            aggregator = eventAggregator;
            alarmTimer = new Timer(2000);
            alarmTimer.Elapsed += AlarmTimer_Elapsed;
            alarmTimer.Start();
        }
        catch (Exception e)
        {
            aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Error, "RTU nelze připojit", e));
            throw;
        }
    }
    #endregion

    #region Private methods
    private void AlarmTimer_Elapsed(object sender, ElapsedEventArgs e)
    {
        try
        {
            var data = new short[10];
            int result;
            lock (RtuMasterProtocol)
            {
                result = RtuMasterProtocol.readMultipleRegisters(ServoID, 0x2A41, data, 2);
            }

            if (result != BusProtocolErrors.FTALK_SUCCESS)
                throw new NotSupportedException(
                    $"Chyba komunikace: {BusProtocolErrors.getBusProtocolErrorText(result)}");

            if (data[0] == 0x0 && data[1] == 0x0)
                aggregator?.Publish(EServoTotalStopStatus.NON_ACTIVE);
            if (data[0] == 0x01 && data[1] == 0x0e6)
                aggregator?.Publish(EServoTotalStopStatus.ACTIVE);
        }
        catch (Exception ex1)
        {
            aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Error, "Neočekávaná chyba serva", ex1, false));
        }
    }
    #endregion
    #region private fields
    private readonly IEventAggregator aggregator;
    private readonly Timer alarmTimer;
    private const short START_ANGLE_FORWARD = 270;
    private const short STOP_ANGLE_FORWARD = 340;
    private const short ZERO_ANGLE_FORWARD = 0;
    private const short START_ANGLE_REVERSE = -270;
    private const short STOP_ANGLE_REVERSE = -340;
    private const short ZERO_ANGLE_REVERSE = -360;
    private const short ANGLE_MULTIPLIER = 1000;
    private const short SPEED_MULTIPLIER = 7;
    private const short ACCEL_DECCEL = 500;
    #endregion

    #region Properties
    public string ComPort { get; set; }
    public MbusRtuMasterProtocol RtuMasterProtocol { get; set; }
    public RegisterConverter RegisterConverter { get; set; }
    public byte ServoID { get; set; }
    public byte DisplayID { get; set; }
    public bool HomingComplete { get; set; }
    #endregion

    #region Public methods
    public bool ServoChangeHomingMethod(int method)
    {
        try
        {
            lock (RtuMasterProtocol)
            {
                //Homing method
                var returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6098, new[] { method });

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Save
                var bigVal = RegisterConverter.ToInt16(0x65766173);
                short[] table = { 0x05, 0, 0, 0, 0, 0, 0, bigVal[0], bigVal[1], 0, 0 };
                returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x1010, table, 11);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new ArgumentException(
                        $"Zápis Save ROM NOK: {BusProtocolErrors.getBusProtocolErrorText(returnStatus)}");

                //Check for EEPROM save
                int saveBit = -1;
                do
                {
                    short[] readReturn = new short[10];
                    returnStatus = RtuMasterProtocol.readMultipleRegisters(ServoID, 0x2D11, readReturn, 1);
                    if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                        throw new ArgumentException(
                            $"Chyba při čtení: servo ROM status: {BusProtocolErrors.getBusProtocolErrorText(returnStatus)}");
                    saveBit = (readReturn[0] & 0x0002) >> 1;
                } while (saveBit != 1);

                return true;
            }
        }
        catch (Exception e)
        {
            aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Error, "Chyba nastavení: ", e));
            return false;
        }
    }

    public void ServoHome()
    {
        try
        {
            lock (RtuMasterProtocol)
            {
                HomingComplete = false;
                //SW limit off
                short[] limitVals = { 0x02, 0, 0, 0, 0 };
                var returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x607D, limitVals, 5);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Homing speed
                var returnSpeed = RegisterConverter.ToInt16(10 * SPEED_MULTIPLIER);
                var creepSpeed = RegisterConverter.ToInt16(5 * SPEED_MULTIPLIER);
                short[] homingSpeeds = { 0x02, returnSpeed[0], returnSpeed[1], creepSpeed[0], creepSpeed[1] };
                returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6099, homingSpeeds, 5);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Homing accel deccel constants
                short[] constTable = { 0x0, 0, 0, 10 * SPEED_MULTIPLIER, ACCEL_DECCEL, ACCEL_DECCEL, 0, 0, 3 };
                returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2801, constTable, 9);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Set and check homing mode
                short[] mode = { 6 };
                returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6060, mode, 1);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                var modeCheck = new short[10];
                returnStatus = RtuMasterProtocol.readMultipleRegisters(ServoID, 0x6061, modeCheck, 1);
                if (modeCheck[0] != 6 || returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                {
                    aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Error,
                        "Inicializace serva NOK: mode select"));
                    HomingComplete = false;
                    return;
                }

                //OP enable
                short[] opEnable = { 0x0f };
                returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6040, opEnable, 1);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));
                Thread.Sleep(500);

                //Start homing
                short[] start = { 0x1F };
                returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6040, start, 1);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));
            }

            //Check for homing finished
            do
            {
                short[] status = new short[10];
                lock (RtuMasterProtocol)
                {
                    var returnStatus = RtuMasterProtocol.readMultipleRegisters(ServoID, 0x6041, status, 1);
                    if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                        throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));
                }

                var homingAtt = (status[0] & 0x1000) >> 12;
                var homingErr = (status[0] & 0x2000) >> 13;

                if (homingErr == 0x01)
                {
                    aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Error,
                        "Inicializace serva NOK: homing error"));
                    return;
                }

                if (homingAtt == 0x01)
                    HomingComplete = true;
                else
                    Thread.Sleep(500);
            } while (!HomingComplete);

            //Set point table mode
            lock (RtuMasterProtocol)
            {
                short[] pointMode = { -101 };
                var returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6060, pointMode, 1);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Info, "Home OK"));
            }
        }
        catch (Exception e)
        {
            aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Error, "Inicializace serva NOK", e));
        }
    }

    public bool ServoMoveOnceForward(int travelSpeed, int measureSpeed)
    {
        if (!HomingComplete)
            throw new NotSupportedException("Servo nezná nulu");

        try
        {
            lock (RtuMasterProtocol)
            {
                int returnStatus;
                //OP enable
                short[] opEnable = { 0x0F };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6040, opEnable, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));


                //Table 0->start
                var firstAngle = RegisterConverter.ToInt16(START_ANGLE_FORWARD * ANGLE_MULTIPLIER);
                short[] initTravelTable =
                {
                    0x00, firstAngle[0], firstAngle[1], (short)(travelSpeed * SPEED_MULTIPLIER), ACCEL_DECCEL,
                    ACCEL_DECCEL, 0, 1, 1
                };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2801, initTravelTable, 9);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Table start->finish
                var secondAngle = RegisterConverter.ToInt16(STOP_ANGLE_FORWARD * ANGLE_MULTIPLIER);
                short[] measureTravelTable =
                {
                    0x00, secondAngle[0], secondAngle[1], (short)(measureSpeed * SPEED_MULTIPLIER), ACCEL_DECCEL,
                    ACCEL_DECCEL, 0, 1, 2
                };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2802, measureTravelTable, 9);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Table finish->0
                var zeroAngle = RegisterConverter.ToInt16(ZERO_ANGLE_FORWARD * ANGLE_MULTIPLIER);
                short[] returnTable =
                {
                    0x00, zeroAngle[0], zeroAngle[1], (short)(travelSpeed * SPEED_MULTIPLIER), ACCEL_DECCEL,
                    ACCEL_DECCEL, 0, 0, 3
                };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2803, returnTable, 9);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Set first table
                short[] firstTable = { 0x01 };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2D60, firstTable, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Start
                short[] start = { 0x1F };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6040, start, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));
            }

            //Check for travel complete
            int travelStatus = -1;
            do
            {
                lock (RtuMasterProtocol)
                {
                    var status = new short[10];
                    int returnStatus;
                    do
                    {
                        returnStatus = RtuMasterProtocol.readMultipleRegisters(ServoID, 0x2D15, status, 1);
                    } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                    if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                        throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));
                    travelStatus = (status[0] & 0x0040) >> 6;
                }

                Thread.Sleep(50);
            } while (travelStatus != 1);

            return true;
        }
        catch (Exception e)
        {
            aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Error, "Selhání komunikace se servem OnceForward",
                e, false));
            return false;
        }
    }

    public bool ServoMoveOnceBackward(int travelSpeed, int measureSpeed)
    {
        if (!HomingComplete)
            throw new NotSupportedException("Servo nezná nulu");

        try
        {
            lock (RtuMasterProtocol)
            {
                //OP enable
                int returnStatus;
                short[] opEnable = { 0x0F };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6040, opEnable, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Table 0->start
                var firstAngle = RegisterConverter.ToInt16(START_ANGLE_REVERSE * ANGLE_MULTIPLIER);
                short[] initTravelTable =
                {
                    0x00, firstAngle[0], firstAngle[1], (short)(travelSpeed * SPEED_MULTIPLIER), ACCEL_DECCEL,
                    ACCEL_DECCEL, 0, 1, 5
                };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2801, initTravelTable, 9);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Table start->finish
                var secondAngle = RegisterConverter.ToInt16(STOP_ANGLE_REVERSE * ANGLE_MULTIPLIER);
                short[] measureTravelTable =
                {
                    0x00, secondAngle[0], secondAngle[1], (short)(measureSpeed * SPEED_MULTIPLIER), ACCEL_DECCEL,
                    ACCEL_DECCEL, 0, 1, 10
                };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2802, measureTravelTable, 9);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Table finish->0
                var zeroAngle = RegisterConverter.ToInt16(ZERO_ANGLE_REVERSE * ANGLE_MULTIPLIER);
                short[] returnTable =
                {
                    0x00, zeroAngle[0], zeroAngle[1], (short)(travelSpeed * SPEED_MULTIPLIER), ACCEL_DECCEL,
                    ACCEL_DECCEL, 0, 0, 15
                };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2803, returnTable, 9);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Set first table
                short[] firstTable = { 0x01 };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2D60, firstTable, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Start 
                short[] start = { 0x1F };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6040, start, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));
            }

            //Check for travel complete
            int travelStatus = -1;
            do
            {
                lock (RtuMasterProtocol)
                {
                    var status = new short[10];
                    int returnStatus;
                    do
                    {
                        returnStatus = RtuMasterProtocol.readMultipleRegisters(ServoID, 0x2D15, status, 1);
                    } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                    if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                        throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));
                    travelStatus = (status[0] & 0x0040) >> 6;
                }

                Thread.Sleep(50);
            } while (travelStatus != 1);

            return true;
        }
        catch (Exception e)
        {
            aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Error,
                "Selhání komunikace se servem OnceBackward", e, false));
            return false;
        }
    }

    public bool ServoMoveContinuousForward(int travelSpeed, int measureSpeed)
    {
        if (!HomingComplete)
            throw new NotSupportedException("Servo nezná nulu");

        try
        {
            lock (RtuMasterProtocol)
            {
                //OP enable
                int returnStatus;
                short[] opEnable = { 0x0F };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6040, opEnable, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Table 0->start
                var firstAngle = RegisterConverter.ToInt16(START_ANGLE_FORWARD * ANGLE_MULTIPLIER);
                short[] initTravelTable =
                {
                    0x00, firstAngle[0], firstAngle[1], (short)(travelSpeed * SPEED_MULTIPLIER), ACCEL_DECCEL,
                    ACCEL_DECCEL, 0, 1, 5
                };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2801, initTravelTable, 9);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Table start->finish
                var secondAngle = RegisterConverter.ToInt16(STOP_ANGLE_FORWARD * ANGLE_MULTIPLIER);
                short[] measureTravelTable =
                {
                    0x00, secondAngle[0], secondAngle[1], (short)(measureSpeed * SPEED_MULTIPLIER), ACCEL_DECCEL,
                    ACCEL_DECCEL, 0, 1, 10
                };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2802, measureTravelTable, 9);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Table finish->0
                var zeroAngle = RegisterConverter.ToInt16(ZERO_ANGLE_FORWARD * ANGLE_MULTIPLIER);
                short[] returnTable =
                {
                    0x00, zeroAngle[0], zeroAngle[1], (short)(travelSpeed * SPEED_MULTIPLIER), ACCEL_DECCEL,
                    ACCEL_DECCEL, 0, 8, 15
                };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2803, returnTable, 9);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Set first table
                short[] firstTable = { 0x01 };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2D60, firstTable, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Start 
                short[] start = { 0x1F };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6040, start, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));
            }

            return true;
        }
        catch (Exception e)
        {
            aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Error, "Selhání komunikace se servem ConForward",
                e, false));
            return false;
        }
    }

    public bool ServoMoveContinuousBackward(int travelSpeed, int measureSpeed)
    {
        if (!HomingComplete)
            throw new NotSupportedException("Servo nezná nulu");

        try
        {
            lock (RtuMasterProtocol)
            {
                //OP enable
                int returnStatus;
                short[] opEnable = { 0x0F };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6040, opEnable, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Table 0->start
                var firstAngle = RegisterConverter.ToInt16(START_ANGLE_REVERSE * ANGLE_MULTIPLIER);
                short[] initTravelTable =
                {
                    0x00, firstAngle[0], firstAngle[1], (short)(travelSpeed * SPEED_MULTIPLIER), ACCEL_DECCEL,
                    ACCEL_DECCEL, 0, 1, 5
                };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2801, initTravelTable, 9);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Table start->finish
                var secondAngle = RegisterConverter.ToInt16(STOP_ANGLE_REVERSE * ANGLE_MULTIPLIER);
                short[] measureTravelTable =
                {
                    0x00, secondAngle[0], secondAngle[1], (short)(measureSpeed * SPEED_MULTIPLIER), ACCEL_DECCEL,
                    ACCEL_DECCEL, 0, 1, 10
                };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2802, measureTravelTable, 9);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Table finish->0
                var zeroAngle = RegisterConverter.ToInt16(ZERO_ANGLE_REVERSE * ANGLE_MULTIPLIER);
                short[] returnTable =
                {
                    0x00, zeroAngle[0], zeroAngle[1], (short)(travelSpeed * SPEED_MULTIPLIER), ACCEL_DECCEL,
                    ACCEL_DECCEL, 0, 8, 15
                };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2803, returnTable, 9);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Set first table
                short[] firstTable = { 0x01 };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x2D60, firstTable, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));

                //Start 
                short[] start = { 0x1F };
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6040, start, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));
            }

            return true;
        }
        catch (Exception e)
        {
            aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Error, "Selhání komunikace se servem ConBackward",
                e, false));
            return false;
        }
    }

    public bool ServoClientMoveStop()
    {
        try
        {
            lock (RtuMasterProtocol)
            {
                short[] stop = { 0x10F };
                int returnStatus;
                do
                {
                    returnStatus = RtuMasterProtocol.writeMultipleRegisters(ServoID, 0x6040, stop, 1);
                } while (returnStatus == BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR);

                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));
            }

            return true;
        }
        catch (Exception e)
        {
            aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Error, "Selhání komunikace se servem STOP", e,
                false));
            return false;
        }
    }

    public void DisplaySetValue(double value)
    {
        try
        {
            lock (RtuMasterProtocol)
            {
                value = Math.Round(value, 1);
                short data = (short)(value * 100);
                var returnStatus = RtuMasterProtocol.writeSingleRegister(DisplayID, 0x0, data);
                if (returnStatus != BusProtocolErrors.FTALK_SUCCESS &&
                    returnStatus != BusProtocolErrors.FTALK_REPLY_TIMEOUT_ERROR &&
                    returnStatus != BusProtocolErrors.FTALK_CHECKSUM_ERROR)
                    throw new NotSupportedException(BusProtocolErrors.getBusProtocolErrorText(returnStatus));
            }
        }
        catch (Exception e)
        {
            aggregator?.Publish(new LoggedData(LoggedData.LoggedLevel.Error, "Selhání komunikace s monitorem", e));
        }
    }

    public void Dispose()
    {
        alarmTimer.Stop();
        RtuMasterProtocol?.closeProtocol();
        RtuMasterProtocol = null;
    }
    #endregion
}
