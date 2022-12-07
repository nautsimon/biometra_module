

import os
import clr

here = os.getcwd()
clr.AddReference(os.path.join(here,'dotnet','BiometraLibraryNet'))
import BiometraLibrary 


settings =  BiometraLibrary.ApplicationClasses.ApplicationSettingsClasses.ApplicationSettings
settings.LoadApplicationSettings()
settings.CommunicationSettings.SerialComSettings.EnableCommunication = True 



user = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.UserClasses.UserDataClasses.UserInitials('ADM')
passwd = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.UserClasses.UserDataClasses.UserPassword('Admin')

def find_device():
    n, avail = info.GetAllComAvailableDevices()   
    device_desc = avail.get_DataList().get_Item(0).DeviceInfos.DeviceDescriptionInfo
    return device_desc

info = BiometraLibrary.DeviceExtComClasses.SystemClasses.InfoClasses.InfoCmds          
device_desc = find_device()

block_cmds = BiometraLibrary.DeviceExtComClasses.BlockClasses.BlockCmds(settings.CommunicationSettings, device_desc)
block_n = BiometraLibrary.DeviceExtComClasses.BlockClasses.BlockDataClasses.BlockNumber(1)

program_cmds = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgramCmds(settings.CommunicationSettings, device_desc)
pcrProgram = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.PcrProgram

def login_user():  
    login_cmds = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.LoginOutCmds(settings.CommunicationSettings,device_desc)
    return check_error(login_cmds.LoginUser(device_desc,user,passwd))
   
login_user()

def check_error(err):
    if err:
        print(err)

def list_programs():
    err, program_list = program_cmds.GetProgramOverview(device_desc,user)
    print(program_list.ToString())

def create_pcr_program():
    pass

def run_pcr_program(prog):
prog_type = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgEditClasses.EnProgramType.TYPE_PROGRAM
program_n = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgEditClasses.ProgramNumber(prog, prog_type)
err = block_cmds.StartProgramOnBlock(device_desc, user, program_n, block_n, True)
    check_error(err)

def stop_program():
    err = block_cmds.StopProgramOnBlock(device_desc, block_n)
    check_error(err)

def lid_open():
    err = block_cmds.OpenMotLid(device_desc,block_n)
    check_error(err)

def lid_close():
    err = block_cmds.CloseMotLid(device_desc,block_n)
    check_error(err)

def create_incubation():
    pass


