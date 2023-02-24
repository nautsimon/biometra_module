
# for linux 
#sudo apt install mono-devel 
#pip install pythonnet

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
# program_file_worker = BiometraLibrary.FileClasses.FileWorkClasses.DeviceFileWorkClasses.ProgramFileWorker()

login_cmds = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.LoginOutCmds(settings.CommunicationSettings,device_desc)

tcda_cmds = BiometraLibrary.DeviceExtComClasses.SystemClasses.TcdaClasses.TcdaCmds(settings.CommunicationSettings, device_desc)

#data_set_list = BiometraLibrary.HelperClasses.DataSetHelperClasses.DataSetList
#program_infos_to_show = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgramInfosToShow


def check_error(err):
    if err:
        print(err)

def login_user():  
    return check_error(login_cmds.LoginUser(device_desc,user,passwd))
   
   
   
login_user()
## to here is "connect", log, check

def list_programs():
    err, program_list = program_cmds.GetProgramOverview(device_desc,user)
    print(program_list.ToString())

def read_all_program_templates():
    program_file_worker = BiometraLibrary.FileClasses.FileWorkClasses.DeviceFileWorkClasses.ProgramFileWorker()
    program_infos_to_show, prog_template_list = program_file_worker.ReadAllProgramTemplateInfosToShow()
    return program_infos_to_show, prog_template_list

def create_program():
    infos_to_show, template_list = read_all_program_templates()
    # check to make sure there's a least one template, if not, maybe create one?
    if len(template_list.DataList) == 0: # maybe use template_list.CheckIfDataAvailable()
        #create new, blank template
        pass
    #TODO: need way of either specifying template, or always selecting blank one and creating from scratch
    pcrProgram = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.PcrProgram()

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

def get_params_data():
    err, set_list = tcda_cmds.GetBlockParamsDataSet(device_desc)
    check_error(err)
    print("DATA SET LIST", set_list)
    temp = str(set_list)
    sub = temp.split(";")
    print("LIST", sub)

def get_status():
    err, status = tcda_cmds.GetBlockState(device_desc, block_n)
    check_error(err)
    print('STATUS', status)

def get_lid_state():
    err, status = tcda_cmds.GetMotLidState(device_desc, block_n)
    check_error(err)
    if str(status) == "0000 0000 0000 0011":
        print("The lid is open on thermocycler " + f"{device_desc}")
    elif str(status) == "0000 0000 0000 0101":
        print("The lid is closed on thermocycler " + f"{device_desc}")
    else:
        print("The lid is currently moving on thermocycler " + f"{device_desc}")

#a, b = program_cmds.GetAllTemplatePrograms(device_desc)

#GetDeviceBlockInfo maybe just block info from gui?
#GetHoldTime
#GetMotLidState_Finished()
#GetRemainingTime

#block commands
#RecieveBusyCmd
#StartProgramOnBlock
#UpdateDeviceNameFromDeviceDescription


def check_temp_left():
    err, temp = tcda_cmds.GetBlockTempLeft(device_desc, block_n)
    check_error(err)
    print("The left temprature of thermocycler " + f"{device_desc}" + " is " +f"{temp}")
    return temp

def check_temp_right():
    err, temp = tcda_cmds.GetBlockTempRight(device_desc, block_n)
    check_error(err)
    print("The right temprature of thermocycler " + f"{device_desc}" + " is " +f"{temp}")
    return temp

def check_temp_middle():
    err, temp = tcda_cmds.GetBlockTempMiddle(device_desc, block_n)
    check_error(err)
    print("The middle temprature of thermocycler " + f"{device_desc}" + " is " +f"{temp}")
    return temp

def check_temp_lid():
    err, temp = tcda_cmds.GetHeatedLidTemp(device_desc, block_n)
    check_error(err)
    print("The lid temprature of thermocycler " + f"{device_desc}" + " is " +f"{temp}")
    return temp

def check_temp_all():
    left = check_temp_left()
    right = check_temp_right()
    middle = check_temp_middle()
    all_temps = [left, middle, right]
    print("The combined tempratures of thermocycler " + f"{device_desc}" + " is " +f"{all_temps[0]}, {all_temps[1]}, {all_temps[2]}")
    return all_temps