

import os
import clr

here = os.getcwd()
clr.AddReference(os.path.join(here,'dotnet','BiometraLibraryNet'))
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

        self.login_user()

    def find_device(self):
        n, avail = self.info.GetAllComAvailableDevices()   
        device_desc = avail.get_DataList().get_Item(0).DeviceInfos.DeviceDescriptionInfo
        return device_desc
    
    def login_user(self):
        login = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.LoginOutCmds(self.settings.CommunicationSettings,self.device_desc)
        user = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.UserClasses.UserDataClasses.UserInitials('ADM')
        passwd = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.UserClasses.UserDataClasses.UserPassword('Admin')
        err = login.LoginUser(self.device_desc,user,passwd)
        if err:
            print(err)

    def lid_open(self):
        self.block_cmds.OpenMotLid(self.device_desc,self.block_n)

    def lid_close(self):
        self.block_cmds.CloseMotLid(self.device_desc,self.block_n)

if __name__ == "__main__":
    test = biometra_trobot()
    test.lid_open()
    test.lid_close()
