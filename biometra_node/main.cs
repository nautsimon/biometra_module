using BiometraLibrary.DeviceExtComClasses.SystemClasses.InfoClasses.InfoDataClasses;
using BiometraLibrary.HelperClasses.ListHelperClasses;
using NetMQ;
using NetMQ.Sockets;
using Newtonsoft.Json;
using System.Text;

public class Biometra
{
    public class Message
    {
        public string action_handle { get; set; }
        public Dictionary<string, string> action_vars { get; set; }

    }

    public static void Main(string[] args)
    {

        AdvancedList<DeviceDescription> device_list = Biometra_Functions.FindDevices();
        string action = "READY";
        string status = "";
        byte[] msg;
        Dictionary<string, string> response;

        using (var server = new ResponseSocket("tcp://*:2002"))
        {
            while (action != "Shutdown")
            {
                Console.Out.WriteLine(server.ToString());
                string t = server.ReceiveFrameString();
                // TODO:Check to make sure that block is open and not running



                Console.Out.WriteLine(t);
                Message m = JsonConvert.DeserializeObject<Message>(t);
                Console.Out.WriteLine(m.action_handle);
                string plate_type = m.action_vars["plate_type"];
                int plate_type_int = Int32.Parse(plate_type);
                int device_num = Biometra_Functions.Connect(device_list, plate_type_int);
                if (m.action_handle == ("run_protocol"))
                {
                    string prog = m.action_vars["program"];
                    int prog_int = Int32.Parse(prog);
                    Biometra_Functions.run_program(device_list, prog_int, device_num);
                    System.Threading.Thread.Sleep(10000);
                    int plate_status = Biometra_Functions.wait_until_ready(device_list, device_num);
                    //TODO: raise error if plate_status is -1

                    // send response when protocol is finished
                    response = new Dictionary<string, string>();
                    response.Add("action_response", "StepStatus.SUCCEEDED");
                    response.Add("action_msg", "program ran");
                    response.Add("action_log", "program ran");
                    msg = Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(response));
                    server.SendFrame(msg);
                }
                else if (m.action_handle == ("open_lid"))
                {
                    Biometra_Functions.open_lid(device_list, device_num);
                    System.Threading.Thread.Sleep(25000);
                    //TODO: check if lid is closed, then check if its open
                    response = new Dictionary<string, string>();
                    response.Add("action_response", "StepStatus.SUCCEEDED");
                    response.Add("action_msg", "lid opened");
                    response.Add("action_log", "lid opened");
                    msg = Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(response));
                    server.SendFrame(msg);

                }
                else if (m.action_handle == ("close_lid"))
                {
                    Biometra_Functions.close_lid(device_list, device_num);
                    System.Threading.Thread.Sleep(25000);
                    //TODO: check if lid is open, then check if its closed
                    response = new Dictionary<string, string>();
                    response.Add("action_response", "StepStatus.SUCCEEDED");
                    response.Add("action_msg", "lid closed");
                    response.Add("action_log", "lid closed");
                    msg = Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(response));
                    server.SendFrame(msg);
                }
                else if (m.action_handle == ("get_status"))
                {
                    bool is_active = Biometra_Functions.get_state(device_list, device_num);
                    if (is_active == true)
                    {
                        response = new Dictionary<string, string>();
                        response.Add("action_response", "StepStatus.SUCCEEDED");
                        response.Add("action_msg", "block running");
                        response.Add("action_log", "block running");
                        msg = Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(response));
                        server.SendFrame(msg);
                    }
                    else
                    {
                        response = new Dictionary<string, string>();
                        response.Add("action_response", "StepStatus.SUCCEEDED");
                        response.Add("action_msg", "block free");
                        response.Add("action_log", "block free");
                        msg = Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(response));
                        server.SendFrame(msg);
                    }
                }
                else
                {
                    // TODO: no correct command
                    Console.WriteLine("INVALID COMMAND");
                }
            }
        }

    }
}

