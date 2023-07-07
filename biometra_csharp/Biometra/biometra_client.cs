
using BiometraLibrary.ApplicationClasses.ApplicationSettingsClasses;
using BiometraLibrary.HelperClasses.ListHelperClasses;
using BiometraLibrary.CommunicationClasses.NetworkClasses;
using BiometraLibrary.DeviceExtComClasses.SystemClasses.InfoClasses;
using BiometraLibrary.DeviceExtComClasses.SystemClasses.InfoClasses.InfoDataClasses;
using BiometraLibrary.HelperClasses.DataSetHelperClasses;
using BiometraLibrary.DeviceExtComClasses.LoginOutClasses;
using BiometraLibrary.HelperClasses.CheckStateHelperClasses;
using BiometraLibrary.DeviceExtComClasses.DeviceComClasses;
using BiometraLibrary.DeviceExtComClasses.ProgClasses;
using BiometraLibrary.DeviceExtComClasses.LoginOutClasses.UserClasses.UserDataClasses;
using BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses;
using BiometraLibrary.DeviceExtComClasses.BlockClasses;
using BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgEditClasses;
using BiometraLibrary.DeviceExtComClasses.BlockClasses.BlockDataClasses;
using BiometraLibrary.DeviceExtComClasses.SystemClasses.TcdaClasses;
using BiometraLibrary.HelperClasses.UnitHelperClasses;

//TODO: maybe add hardcoded in variables for device number and block number

namespace Biometra
{


    public class Client
    {

        //ApplicationSettings.CommunicationSettings.DeviceName = new ExternalDeviceName("BiometraLibTest");

        // Network connection
        static void connect_network()
        {
            //Read all available network adapters
            AdvancedList<String> networkDeviceList = NetworkHelperClass.GetAllNetworkDeviceDescriptions();

            // Set UDP Params
            ApplicationSettings.CommunicationSettings.NetSettings.UdpComSettings.UdpComParams = new BiometraLibrary.CommunicationClasses.NetworkClasses.UdpClasses.UdpParams(networkDeviceList[0], new NetworkPort(55555), new NetworkPort(), BiometraLibrary.CommunicationClasses.EnCommunicationTimeout.TIMEOUT_1500ms);

            // Activate UDP connection
            ApplicationSettings.CommunicationSettings.NetSettings.UdpComSettings.EnableCommunication = true;

        }

        static void connect_serial()
        {
            //settings
            ApplicationSettings.CommunicationSettings.SerialComSettings.SerialComParams = new BiometraLibrary.CommunicationClasses.SerialComClasses.SerialParams(BiometraLibrary.CommunicationClasses.SerialComClasses.EnBaudRate.BAUDRATE_115200, BiometraLibrary.CommunicationClasses.SerialComClasses.EnParity.PARITY_NONE, BiometraLibrary.CommunicationClasses.SerialComClasses.EnDataBits.DATABITS_8, BiometraLibrary.CommunicationClasses.SerialComClasses.EnStopBits.STOPBITS_ONE, BiometraLibrary.CommunicationClasses.EnCommunicationTimeout.TIMEOUT_1500ms);

            //enable serial commmunication
            ApplicationSettings.CommunicationSettings.SerialComSettings.EnableCommunication = true;
        }

        // Potential TODO: set default file location

        static AdvancedList<DeviceDescription> get_device_list()
        {
            //Search all available devices
            InfoCmds.GetAllComAvailableDevices(out DataSetList<AvailableDevice, NotAvailableDevice> foundDeviceList);

            Console.WriteLine("found devices", foundDeviceList);

            //Read descriptions from all available devices

            //public class deviceList?
            AdvancedList<DeviceDescription> deviceList = DeviceCom.GetSavedDeviceDescriptions();

            return deviceList;
        }

        //Read device information
        static void get_device_info(AdvancedList<DeviceDescription> deviceList)//TODO: pass in device list?)
        {
            CheckStateResult checkStateResult = DeviceCom.GetInformationsByDeviceDescription(deviceList[0], out DeviceInformations deviceInformations);

        }

        // Read number of found devices
        static int get_num_devices()
        {
            int iNumOfDevices = DeviceCom.GetNumberOfScannedDevices();
            return iNumOfDevices;

        }
        // Won't need to mess with users, will always use admin

        // log in as admin
        static void login_user(AdvancedList<DeviceDescription> deviceList)
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

        // wont need to log a user off, or create new users


        // device settings? maybe... TODO: section 5.4.2

        // Retrieve program from the device

        //retreieve all available programs from device
        static void get_program_list(AdvancedList<DeviceDescription> deviceList)
        {
            try
            {
                //Create communication object
                using (ProgramCmds programCmds = new ProgramCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
                {
                    // Read all availbel programs
                    CheckStateResult checkStateResult = programCmds.GetProgramOverview(deviceList[0], new UserInitials("ADM"), out DataSetList<ProgramInfos> programList);
                }
            }
            catch (Exception ex) { Console.WriteLine(ex.Message); }
        }


        // ^ can be done async

        // Creating a program TODO

        //Starting a program
        static void run_program(AdvancedList<DeviceDescription> deviceList)
        {
            try
            {
                //create communication object
                using (BlockCmds blockCmds = new BlockCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
                {
                    // run program and start run log file
                    CheckStateResult checkStateResult = blockCmds.StartProgramOnBlock(deviceList[0], new UserInitials("ADM"), new ProgramNumber(4, EnProgramType.TYPE_PROGRAM), new BlockNumber(1), true);
                }
            }
            catch (Exception ex) { Console.WriteLine(ex.Message); }
        }

        //Stop program
        //public
        static void stop_program(AdvancedList<DeviceDescription> deviceList)
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
        /*public*/
        static void open_lid(AdvancedList<DeviceDescription> deviceList)
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
        static void close_lid(AdvancedList<DeviceDescription> deviceList)
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
        static void check_params(AdvancedList<DeviceDescription> deviceList)
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

        static string get_state(AdvancedList<DeviceDescription> deviceList)
        {
            try
            {
                //create communication object
                using (TcdaCmds tcdaCmds = new TcdaCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
                {
                    CheckStateResult deviceComResult = tcdaCmds.GetBlockState(deviceList[0], new BlockNumber(1), out BlockState blockState);
                    return deviceComResult.ToString();
                }

            }
            catch (Exception ex) { Console.WriteLine(ex.Message); }
            return "ERROR in get_state function";
        }

        static string get_lid_state(AdvancedList<DeviceDescription> deviceList)
        {
            try
            {
                using (TcdaCmds tcdaCmds = new TcdaCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
                {
                    CheckStateResult deviceComResult = tcdaCmds.GetMotLidState(deviceList[0], out DataSetList<MotLidState, BlockNumber> motLidStateList);
                    return deviceComResult.ToString();
                }


            }
            catch (Exception ex) { Console.WriteLine(ex.Message); }
            return "Error in get_lid_state function";
        }

        static string check_temp_lid(AdvancedList<DeviceDescription> deviceList)
        {
            try
            {
                using (TcdaCmds tcdaCmds = new TcdaCmds(ApplicationSettings.CommunicationSettings, deviceList[0]))
                {
                    CheckStateResult deviceComResult = tcdaCmds.GetHeatedLidTemp(deviceList[0], new BlockNumber(1), out Temperature heatedLidTemp);
                }
            }
            catch (Exception ex) { Console.WriteLine(ex.Message); }
            return "Error in get_lid_state function";
        }

        static string get_temp_left(AdvancedList<DeviceDescription> deviceList)
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

        static string get_temp_right(AdvancedList<DeviceDescription> deviceList)
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

        static string get_temp_middle(AdvancedList<DeviceDescription> deviceList)
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


        static void Main(string[] args)
        {
            ApplicationSettings.LoadApplicationSettings();
            ApplicationSettings.SaveApplicationSettings();
            //Console.WriteLine("HEREeeeee");
            //connect_network();
            //connect_serial();
            try
            {
                connect_serial();
            }
            catch (Exception ex) { Console.WriteLine(ex.Message); }
            Console.WriteLine("Network Connection Enabled");
            AdvancedList<DeviceDescription> device_list = get_device_list();
            try
            {
                get_device_info(device_list);
            }
            catch (Exception ex) { Console.WriteLine(ex.Message); }
            Console.WriteLine("Device Connected");
            login_user(device_list);
            get_state(device_list);
            open_lid(device_list);
        }
    }
}


// run log file TODO

