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
        user = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.UserClasses.UserDataClasses.UserInitials('ADM')
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
    # return self.pcr???                

    def open_lid(self):
        self.block_cmds.OpenMotLid(self.device_desc,self.block_n)

    def close_lid(self):
        self.block_cmds.CloseMotLid(self.device_desc,self.block_n)

if __name__ == "__main__":
    test = biometra_trobot()
    # test.lid_open()
    test.lid_close()

