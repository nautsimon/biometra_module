
using BiometraLibrary.ApplicationClasses.ApplicationSettingsClasses;
using BiometraLibrary.CommunicationClasses.NetworkClasses;
using BiometraLibrary.DeviceExtComClasses.BlockClasses;
using BiometraLibrary.DeviceExtComClasses.BlockClasses.BlockDataClasses;
using BiometraLibrary.DeviceExtComClasses.DeviceComClasses;
using BiometraLibrary.DeviceExtComClasses.LoginOutClasses;
using BiometraLibrary.DeviceExtComClasses.LoginOutClasses.UserClasses.UserDataClasses;
using BiometraLibrary.DeviceExtComClasses.ProgClasses;
using BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses;
using BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgEditClasses;
using BiometraLibrary.DeviceExtComClasses.SystemClasses.InfoClasses;
using BiometraLibrary.DeviceExtComClasses.SystemClasses.InfoClasses.InfoDataClasses;
using BiometraLibrary.DeviceExtComClasses.SystemClasses.TcdaClasses;
using BiometraLibrary.HelperClasses.CheckStateHelperClasses;
using BiometraLibrary.HelperClasses.DataSetHelperClasses;
using BiometraLibrary.HelperClasses.ListHelperClasses;
using BiometraLibrary.HelperClasses.UnitHelperClasses;




public class Biometra_Functions
{

    //ApplicationSettings.CommunicationSettings.DeviceName = new ExternalDeviceName("BiometraLibTest");

    // Network connection
    public static void connect_network()
    {
        //Read all available network adapters
        AdvancedList<String> networkDeviceList = NetworkHelperClass.GetAllNetworkDeviceDescriptions();

        // Set UDP Params
        ApplicationSettings.CommunicationSettings.NetSettings.UdpComSettings.UdpComParams = new BiometraLibrary.CommunicationClasses.NetworkClasses.UdpClasses.UdpParams(networkDeviceList[0], new NetworkPort(55555), new NetworkPort(), BiometraLibrary.CommunicationClasses.EnCommunicationTimeout.TIMEOUT_1500ms);

        // Activate UDP connection
        ApplicationSettings.CommunicationSettings.NetSettings.UdpComSettings.EnableCommunication = true;

    }

    public static void connect_serial()
    {
        //settings
        ApplicationSettings.CommunicationSettings.SerialComSettings.SerialComParams = new BiometraLibrary.CommunicationClasses.SerialComClasses.SerialParams(BiometraLibrary.CommunicationClasses.SerialComClasses.EnBaudRate.BAUDRATE_115200, BiometraLibrary.CommunicationClasses.SerialComClasses.EnParity.PARITY_NONE, BiometraLibrary.CommunicationClasses.SerialComClasses.EnDataBits.DATABITS_8, BiometraLibrary.CommunicationClasses.SerialComClasses.EnStopBits.STOPBITS_ONE, BiometraLibrary.CommunicationClasses.EnCommunicationTimeout.TIMEOUT_1500ms);

        // disable COM-port filtering
        ApplicationSettings.CommunicationSettings.SerialComSettings.AllComPorts = true;


        //enable serial commmunication
        ApplicationSettings.CommunicationSettings.SerialComSettings.EnableCommunication = true;
    }

    // Potential TODO: set default file location

    public static AdvancedList<DeviceDescription> get_device_list()
    {
        //Search all available devices
        InfoCmds.GetAllComAvailableDevices(out DataSetList<AvailableDevice, NotAvailableDevice> foundDeviceList);
        //Read descriptions from all available devices

        //public class deviceList?
        AdvancedList<DeviceDescription> deviceList = DeviceCom.GetSavedDeviceDescriptions();

        return deviceList;
    }

    //Read device information
    public static void get_device_info(AdvancedList<DeviceDescription> deviceList)//TODO: pass in device list?)
    {
        CheckStateResult checkStateResult = DeviceCom.GetInformationsByDeviceDescription(deviceList[0], out DeviceInformations deviceInformations);

    }

    // Read number of found devices
    public static int get_num_devices()
    {
        int iNumOfDevices = DeviceCom.GetNumberOfScannedDevices();
        return iNumOfDevices;

    }
    // Won't need to mess with users, will always use admin

    // log in as admin
    public static void login_user(AdvancedList<DeviceDescription> deviceList)
    {
        try
        {
            //Create object for communication
            using (LoginOutCmds loginOutCmds = new LoginOutCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
            {
                //Register user ADM
                CheckStateResult checkStateResult = loginOutCmds.LoginUser(deviceList[0], new UserInitials("ADM"), new UserPassword("Admin"));
            }
            // Catch excxeption, triggered when user already logged in
        }
        catch (Exception ex) { Console.WriteLine(ex.Message); }

    }

    // Retrieve program from the device

    //retreieve all available programs from device
    public static string get_program_list(AdvancedList<DeviceDescription> deviceList)
    {
        try
        {
            //Create communication object
            using (ProgramCmds programCmds = new ProgramCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
            {
                // Read all availbel programs
                CheckStateResult checkStateResult = programCmds.GetProgramOverview(deviceList[0], new UserInitials("ADM"), out DataSetList<ProgramInfos> programList);
                return programList.ToString();
            }
        }
        catch (Exception ex) { Console.WriteLine(ex.Message); }
        return "ERROR: program list could not be retrieved";
    }


    // ^ can be done async

    // Creating a program TODO

    //Starting a program
    public static void run_program(AdvancedList<DeviceDescription> deviceList, int prog)
    {
        try
        {
            //create communication object
            using (BlockCmds blockCmds = new BlockCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
            {
                // run program and start run log file
                CheckStateResult checkStateResult = blockCmds.StartProgramOnBlock(deviceList[0], new UserInitials("ADM"), new ProgramNumber((byte)prog, EnProgramType.TYPE_PROGRAM), new BlockNumber(1), true);
            }
        }
        catch (Exception ex) { Console.WriteLine(ex.Message); }
    }

    //Stop program
    public static void stop_program(AdvancedList<DeviceDescription> deviceList)
    {
        try
        {
            //Create commmunication object
            using (BlockCmds blockCmds = new BlockCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
            {
                CheckStateResult checkStateResult = blockCmds.StopProgramOnBlock(deviceList[0], new BlockNumber(1));
            }
        }
        catch (Exception ex) { Console.WriteLine(ex.Message); }

    }
    //Pause/cont program?

    //open lid TODO: pass device and block numbers as variables
    public static void open_lid(AdvancedList<DeviceDescription> deviceList)
    {
        try
        {
            //Create communication object
            using (BlockCmds blockCmds = new BlockCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
            {
                CheckStateResult checkStateResult = blockCmds.OpenMotLid(deviceList[0], new BlockNumber(1));
            }
        }
        catch (Exception ex) { Console.WriteLine(ex.Message); }
    }

    //close lid TODO: pass device and block numbers as variables
    /*public*/
    public static void close_lid(AdvancedList<DeviceDescription> deviceList)
    {
        try
        {
            //Create communication object
            using (BlockCmds blockCmds = new BlockCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
            {
                CheckStateResult checkStateResult = blockCmds.CloseMotLid(deviceList[0], new BlockNumber(1));
            }
        }
        catch (Exception ex) { Console.WriteLine(ex.Message); }

    }

    // current block parameters list
    /*public*/
    public static void check_params(AdvancedList<DeviceDescription> deviceList)
    {
        try
        {
            //create communication object
            using (TcdaCmds tcdaCmds = new TcdaCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
            {
                //Query block parameters for all existing blocks
                CheckStateResult checkStateResult = tcdaCmds.GetBlockParamsDataSet(deviceList[0], out DataSetList<BlockParams, BlockNumber> blockParamList);
            }
        }
        catch (Exception ex) { Console.WriteLine(ex.Message); }

    }

    public static bool get_state(AdvancedList<DeviceDescription> deviceList)
    {

        //create communication object
        Console.WriteLine("getting block state...");
        using (TcdaCmds tcdaCmds = new TcdaCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
        {
            CheckStateResult deviceComResult = tcdaCmds.GetBlockState(deviceList[0], new BlockNumber(1), out BlockState blockState);
            bool s = blockState.ActiveBlock; //ActiveBlock or BlockStateView?
            return s;
        }


        //catch (Exception ex) { Console.WriteLine(ex.Message); }
        //return "ERROR in get_state function";

    }

    public static string get_lid_state(AdvancedList<DeviceDescription> deviceList) //TODO
    {
        try
        {
            using (TcdaCmds tcdaCmds = new TcdaCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
            {
                CheckStateResult deviceComResult = tcdaCmds.GetMotLidState(deviceList[0], out DataSetList<MotLidState, BlockNumber> motLidStateList);
                Console.WriteLine("getting lid state...");
                string lid_state = motLidStateList.ToString();
                if (lid_state == "0000 0000 0000 0011;;")
                {
                    //lid is open
                    return "open";
                }
                else if (lid_state == "0000 0000 0000 0101;;")
                {
                    //lid is closed
                    return "closed";
                }
                else
                {
                    // lid is in motion, loop back?
                    return "busy";
                }


            }


        }
        catch (Exception ex) { Console.WriteLine(ex.Message); }
        return "Error in get_lid_state function";
    }

    // TODO: look into command : checkallblocksfree

    //public static bool can_open(AdvancedList<DeviceDescription> deviceList)
    //{
    //    try
    //    {
    //        using (MotLidState motLidState = new MotLidState(ApplicationSettings.CommunicationSettings, deviceList[0]))
    //        {
    //            //CheckStateResult deviceComResult = motLidState.CanOpenLid(deviceList[0], out string canopen);
    //            bool canopen = motLidState.CanOpenLid;
    //            return canopen;
    //        }

    //    }
    //    catch (Exception ex) { Console.WriteLine(ex.Message); }
    //    return false;

    public static string get_temp_lid(AdvancedList<DeviceDescription> deviceList)
    {
        try
        {
            using (TcdaCmds tcdaCmds = new TcdaCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
            {
                CheckStateResult deviceComResult = tcdaCmds.GetHeatedLidTemp(deviceList[0], new BlockNumber(1), out Temperature heatedLidTemp);
                return heatedLidTemp.ToString();
            }
        }
        catch (Exception ex) { Console.WriteLine(ex.Message); }
        return "Error in get_lid_state function";
    }

    public static string get_temp_left(AdvancedList<DeviceDescription> deviceList)
    {
        try
        {
            using (TcdaCmds tcdaCmds = new TcdaCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
            {
                CheckStateResult deviceComResult = tcdaCmds.GetBlockTempLeft(deviceList[0], new BlockNumber(1), out Temperature leftBlockTemp);
                return deviceComResult.ToString();
            }

        }
        catch (Exception ex) { Console.WriteLine(ex.Message); }
        return "Error in get_temp_left function";
    }

    public static string get_temp_right(AdvancedList<DeviceDescription> deviceList)
    {
        try
        {
            using (TcdaCmds tcdaCmds = new TcdaCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
            {
                CheckStateResult deviceComResult = tcdaCmds.GetBlockTempRight(deviceList[0], new BlockNumber(1), out Temperature rightBlockTemp);
                return deviceComResult.ToString();
            }

        }
        catch (Exception ex) { Console.WriteLine(ex.Message); }
        return "Error in get_temp_right function";
    }

    public static string get_temp_middle(AdvancedList<DeviceDescription> deviceList)
    {
        try
        {
            using (TcdaCmds tcdaCmds = new TcdaCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
            {
                CheckStateResult deviceComResult = tcdaCmds.GetBlockTempMiddle(deviceList[0], new BlockNumber(1), out Temperature middleBlockTemp);
                return deviceComResult.ToString();
            }

        }
        catch (Exception ex) { Console.WriteLine(ex.Message); }
        return "Error in get_temp_middle function";
    }

    public static List<string> get_temp_all(AdvancedList<DeviceDescription> deviceList)
    {

        List<string> temp_list = new List<string>();
        temp_list.Add(get_temp_left(deviceList));
        temp_list.Add(get_temp_right(deviceList));
        temp_list.Add(get_temp_middle(deviceList));
        temp_list.Add(get_temp_lid(deviceList));
        return temp_list;

    }

    public static AdvancedList<DeviceDescription> Connect()
    {
        ApplicationSettings.LoadApplicationSettings();
        connect_serial();
        ApplicationSettings.SaveApplicationSettings();
        Console.WriteLine("Serial Connection Enabled");
        AdvancedList<DeviceDescription> device_list = get_device_list();
        get_device_info(device_list);
        Console.WriteLine("Device Connected");
        login_user(device_list);
        Console.WriteLine("User logged in");

        return device_list;
    }

    public static int check_clear(AdvancedList<DeviceDescription> deviceList)
    {
        // Checks to make sure the device is open, and not running

        // check if device is running
        bool is_active = get_state(deviceList);
        if (is_active == true) // block is active
        {
            Console.WriteLine("Protocol still in progress, waiting...");
            System.Threading.Thread.Sleep(5000);
            return 1;
        }
        else
        {
            // check if device open
            string lid_status = get_lid_state(deviceList);
            if (lid_status == "busy")
            {
                Console.WriteLine("lid not open or closed, waiting...");
                System.Threading.Thread.Sleep(10000);
                string new_lid_status = get_lid_state(deviceList);
                return 1;
                //if (new_lid_status == "busy")
                //{
                //    Console.WriteLine("ERROR: lid stuck in busy state"); //TODO: cycle close and open here
                //    return -1;
                //}
            }
            if (lid_status == "closed")
            {
                Console.WriteLine("lid is closed, opening...");
                open_lid(deviceList);
                System.Threading.Thread.Sleep(10000);
                return 1;
            }
            else // lid is open, and block is not active
            {
                // check temp?
                return 0;
            }
        }


    }

    public static int wait_until_ready(AdvancedList<DeviceDescription> deviceList)
    {
        int plate_status = 1;
        while (plate_status == 1)
        {
            plate_status = check_clear(deviceList);
            if (plate_status == -1)
            {
                Console.WriteLine("ERROR"); // TODO: need get_error function
                return -1;
            }

            System.Threading.Thread.Sleep(5000);
        }
        if (plate_status == 0)
        {
            Console.WriteLine("Plate ready to be retrieved");
            return 0;
        }
        else
        {
            Console.WriteLine("ERROR:unknown plate status");
            return -1;
        }
    }





    //static void Main(string[] args)
    //{
    //    ApplicationSettings.LoadApplicationSettings();
    //    //connect_network();
    //    connect_serial();
    //    ApplicationSettings.SaveApplicationSettings();
    //    Console.WriteLine("Network Connection Enabled");
    //    AdvancedList<DeviceDescription> device_list = get_device_list();
    //    get_device_info(device_list);
    //    Console.WriteLine("Device Connected");
    //    login_user(device_list);
    //    //open_lid(device_list);
    //    //close_lid(device_list);
    //    Console.WriteLine(get_lid_state(device_list));
    //    Console.WriteLine(get_state(device_list)); // TODO: translate
    //    string a = get_temp_lid(device_list);
    //    Console.WriteLine(a);
    //}
}

