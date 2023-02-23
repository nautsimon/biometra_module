import clr
from pathlib import Path 

dotnet_path = Path(__file__).resolve().parent / 'dotnet' / 'BiometraLibraryNet'
clr.AddReference(str(dotnet_path))
import BiometraLibrary 

class biometra_trobot():
    def __init__(self):
        self.settings =  BiometraLibrary.ApplicationClasses.ApplicationSettingsClasses.ApplicationSettings
        self.settings.LoadApplicationSettings()
        self.settings.CommunicationSettings.SerialComSettings.EnableCommunication = True 

        self.info = BiometraLibrary.DeviceExtComClasses.SystemClasses.InfoClasses.InfoCmds          
        self.device_desc = self.find_device()

        self.block_cmds = BiometraLibrary.DeviceExtComClasses.BlockClasses.BlockCmds(self.settings.CommunicationSettings, self.device_desc)
        self.block_n = BiometraLibrary.DeviceExtComClasses.BlockClasses.BlockDataClasses.BlockNumber(1)
        
        self.tcda_cmds = BiometraLibrary.DeviceExtComClasses.SystemClasses.TcdaClasses.TcdaCmds(self.settings.CommunicationSettings, self.device_desc)
        
        self.program_cmds = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgramCmds(self.settings.CommunicationSettings, self.device_desc)

        # self.fileInfo = BiometraLibrary.DeviceExtComClasses.DeviceComClasses.DeviceComParams
        self.DataSetList = BiometraLibrary.HelperClasses.DataSetHelperClasses.DataSetList

        self.login_user()

    def check_error(self, err):
        if err:
            self.error = err
            print(err)
            return -1
        return 0

    def find_device(self):
        n, avail = self.info.GetAllComAvailableDevices()
        device_desc = avail.get_DataList().get_Item(0).DeviceInfos.DeviceDescriptionInfo
        return device_desc
    
    def login_user(self):
        login = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.LoginOutCmds(self.settings.CommunicationSettings,self.device_desc)
        user =  BiometraLibrary.DeviceExtComClasses.LoginOutClasses.UserClasses.UserDataClasses.UserInitials('ADM')
        passwd = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.UserClasses.UserDataClasses.UserPassword('Admin')
        err = login.LoginUser(self.device_desc,user,passwd)
        return self.check_error(err)
    
    def create_program(self):
        # self.file = BiometraLibrary.FileClasses.FileWorkClasses.DeviceFileWorkClasses.ProgramFileWorker(self.fileInfo)
        self.programFileWorker = BiometraLibrary.FileClasses.FileWorkClasses.DeviceFileWorkClasses.ProgramFileWorker()
        self.checkStateResult = self.programFileWorker.ReadAllProgramTemplateInfosToShow(self.DataSetList)

        # Create new program
        self.pcrProgram = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.PcrProgram()
    
    def create_program(self, params):
        pass

    def get_state(self):
        pass

    def list_programs(self):
        err, program_list = self.program_cmds.GetProgramOverview(self.device_desc, self.user)

    def create_pcr_program():
        pass

    def run_pcr_program(self, prog):
        prog_type = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgEditClasses.EnProgramType.TYPE_PROGRAM
        program_n = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgEditClasses.ProgramNumber(prog, prog_type)
        err = self.block_cmds.StartProgramOnBlock(self.device_desc, self.user, program_n, self.block_n, True)
        self.check_error(err)

    def stop_program(self):
        err = self.block_cmds.StopProgramOnBlock(self.device_desc, self.block_n)
        self.check_error(err)
    
    def open_lid(self):
        self.block_cmds.OpenMotLid(self.device_desc,self.block_n)

    def close_lid(self):
        self.block_cmds.CloseMotLid(self.device_desc,self.block_n)
        
    def get_lid_state(self):
        err, status = self.tcda_cmds.GetMotLidState(self.device_desc, self.block_n)
        self.check_error(err)
        if str(status) == "0000 0000 0000 0011":
            print("The lid is open on thermocycler " + f"{self.device_desc}")
        elif str(status) == "0000 0000 0000 0101":
            print("The lid is closed on thermocycler " + f"{self.device_desc}")
        else:
            print("The lid is currently moving on thermocycler " + f"{self.device_desc}")


    def check_temp_left(self):
        err, temp = self.tcda_cmds.GetBlockTempLeft(self.device_desc, self.block_n)
        self.check_error(err)
        print("The left temprature of thermocycler " + f"{self.device_desc}" + " is " +f"{temp}")
        return temp

    def check_temp_right(self):
        err, temp = self.tcda_cmds.GetBlockTempRight(self.device_desc, self.block_n)
        self.check_error(err)
        print("The right temprature of thermocycler " + f"{self.device_desc}" + " is " +f"{temp}")
        return temp

    def check_temp_middle(self):
        err, temp = self.tcda_cmds.GetBlockTempMiddle(self.device_desc, self.block_n)
        self.check_error(err)
        print("The middle temprature of thermocycler " + f"{self.device_desc}" + " is " +f"{temp}")
        return temp
    
    def check_temp_lid(self):
        err, temp = self.tcda_cmds.GetHeatedLidTemp(self.device_desc, self.block_n)
        self.check_error(err)
        print("The lid temprature of thermocycler " + f"{self.device_desc}" + " is " +f"{temp}")
        return temp

    def check_temp_all(self):
        left = self.check_temp_left()
        right = self.check_temp_right()
        middle = self.check_temp_middle()
        all_temps = [left, middle, right]
        print("The combined tempratures of thermocycler " + f"{self.device_desc}" + " is " +f"{all_temps[0]}, {all_temps[1]}, {all_temps[2]}")
        return all_temps

if __name__ == "__main__":
    test = biometra_trobot()
    # test.lid_open()
    test.lid_close()

