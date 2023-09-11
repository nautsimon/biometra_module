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
using BiometraLibrary.CommunicationClasses.SerialComClasses;
using System.IO.Ports;
using NetMQ;
using Newtonsoft.Json;
using NetMQ.Sockets;
using System.Security.Cryptography;
using System.Text;
//using System.Net.Configuration;
using System.Diagnostics;
//using System.Windows.Forms;

public class Biometra
{
    public class Message
    {
        public string action_handle { get; set; }
        public Dictionary<string, string> action_vars { get; set; }

    }

    public static void Main(string[] args)
    {
        AdvancedList<DeviceDescription> device_list = Biometra_Functions.Connect();
        string action = "READY";
        string status = "";

        using (var server = new ResponseSocket("tcp://*:2001")) //TODO: change
        {
            while (action != "Shutdown")
            {
                Console.Out.WriteLine(server.ToString());
                string t = server.ReceiveFrameString();
                // TODO:Check to make sure that block is open and not running



                Console.Out.WriteLine(t);
                Message m = JsonConvert.DeserializeObject<Message>(t);
                Console.Out.WriteLine(m.action_handle);
                if (m.action_handle == ("run_protocol"))
                {
                    string prog = m.action_vars["program"];
                    int prog_int = Int32.Parse(prog);
                    Biometra_Functions.run_program(device_list); //TODO: add prog number as arg

                }
                else if (m.action_handle == ("open_lid"))
                {
                    // TODO
                }
                else if (m.action_handle == ("close_lid"))
                {
                    // TODO
                }
                else if (m.action_handle == ("get_status"))
                {
                    // TODO
                }
                else
                {
                    // TODO: no correct command
                }
            }
        }

    }
    public Biometra()
    {

    }
}

