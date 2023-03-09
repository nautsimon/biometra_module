
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
# pcrProgram = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.PcrProgram
# program_file_worker = BiometraLibrary.FileClasses.FileWorkClasses.DeviceFileWorkClasses.ProgramFileWorker()

login_cmds = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.LoginOutCmds(settings.CommunicationSettings,device_desc)

tcda_cmds = BiometraLibrary.DeviceExtComClasses.SystemClasses.TcdaClasses.TcdaCmds(settings.CommunicationSettings, device_desc)
#program_number = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgEditClasses.ProgramNumber
#data_set_list = BiometraLibrary.HelperClasses.DataSetHelperClasses.DataSetList
#program_infos_to_show = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgramInfosToShow
#blank_template = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgramName.PROG_NAME_BLANK_P
# prog_name = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgramName()


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
    # infos_to_show, template_list = read_all_program_templates()
    # check to make sure there's a least one template, if not, maybe create one?
    # if len(template_list.DataList) == 0: # maybe use template_list.CheckIfDataAvailable()
    #     #create new, blank template
    #     pass
    #TODO: need way of either specifying template, or always selecting blank one and creating from scratch
    
    pcrProgram = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.PcrProgram()
    
    # create block number TODO
    en_block_number = BiometraLibrary.DeviceExtComClasses.BlockClasses.BlockDataClasses.EnBlockTypeNumber
    block_type = BiometraLibrary.DeviceExtComClasses.BlockClasses.BlockDataClasses.BlockType
    
    # set name of program
    prog_name = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgramName("programName")
    # prog_name.set_Value = ("programname")
    pcrProgram.ProgramEditInfo.ProgName = prog_name
    
    # set user directory of program (admin user)
    pcrProgram.ProgramEditInfo.UserDirectory = user
    
    # creation date, must be format dd.MM.yyyy HH:mm:ss system.datetime
    
    #create lid settings and lid temperature
    lid_temp = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgHeadClasses.LidTemperature("50 °C", pcrProgram.ProgramHeadInfo.BlockTyp.BlockTypData)
    # lid_settings
    
 # 0000 0000 0010 0000 - before run
 # 0000 0000 0100 0001 - run started (Run, Preheat), then Run, plateau
 # 0000 0000 0110 0000 - run stopped (cool_heat preheat) after stop, (plateau, cool_heat)

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


def get_run_status():
    '''
    determines the current run state of the device, returns:
    1: protool in progress
    0: no protocol in progress
    -1: unknown error
    '''
    
    err, status = tcda_cmds.GetBlockState(device_desc, block_n)
    check_error(err)
    if str(status) == "0000 0000 0100 0001":
        print("There is currently a protocol in progress on thermocycler " + f"{device_desc}")
        return 1
    elif str(status) == "0000 0000 0110 0000":
        print("There is currently no protocol in progress on thermocycler " + f"{device_desc}")
        return 0
    else:
        print("Unknown state of themrocycler " + f"{device_desc}")
        return -1

def get_lid_state():
    '''
    determines current state of lid.
    returns:
    1: lid closed
    0: lid open
    -1: lid in motion
    '''
    err, status = tcda_cmds.GetMotLidState(device_desc, block_n)
    check_error(err)
    if str(status) == "0000 0000 0000 0011":
        print("The lid is open on thermocycler " + f"{device_desc}")
        return 0
    elif str(status) == "0000 0000 0000 0101":
        print("The lid is closed on thermocycler " + f"{device_desc}")
        return 1
    else:
        print("The lid is currently moving on thermocycler " + f"{device_desc}")
        return -1

def check_plate_avail():
    '''
    checkes the run progress, lid state, and temperature of the block in order 
    to tell if plate is safe for retreival
    returns:
    1: plate available
    0: run currently underway, wait
    -1: error
    '''
    # check if run currently underway
    run_status = get_run_status()
    if run_status == 1:
        time = get_time_left()
        print("Run will complete in " + f"{time}, " + "waiting...")
    elif run_status == 0:
        # No current run, check if lid is open
        lid_state = get_lid_state()
        if lid_state == 1:
            # lid closed, open lid, and wait
            lid_open()
        elif lid_state == -1:
            print("Lid is in motion, wait")
        elif lid_state == 0:
            # lid is open, check temperature
            pass
        
        
    elif run_status == -1:
        print("Error involving run status")
        
    
def get_time_left():
    '''
    checks how much time is remaining in the current program
    returns:
    time: string of how much time is left: 00h 00m 00s
    '''
    # check to make sure there's a protocol underway
    run_status = get_run_status()
    if run_status == 1:
        err, time = tcda_cmds.GetRemainingTime(device_desc, block_n)
        check_error(err)
        print("There is " + f"{time} " "left on thermocycler " + f"{device_desc}")
        return time
    else:
        print("There is no protocol being run on thermocycler " + f"{device_desc}")

#a, b = program_cmds.GetAllTemplatePrograms(device_desc)

#GetDeviceBlockInfo maybe just block info from gui?
#GetHoldTime
#GetMotLidState_Finished()
#GetRemainingTime

#block commands
#RecieveBusyCmd
#StartProgramOnBlock
#UpdateDeviceNameFromDeviceDescription

# zeros in get params data = loop counters

#progtype_standard = getdeviceprogtype
#getnumofblocks = 1
#getnumofcontrollers = 3
# getprogramstep = 00 (after time)
#2nd time = getholdtime

# getmotlidstate.motlidstateflags = meaning of number value
#canopen/closelid


# run_status.ActiveBlock = bool whether current run
#status.

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
    lid = check_temp_lid()
    all_temps = [left, middle, right]
    print("The combined tempratures of thermocycler " + f"{device_desc}" + " is " +f"{all_temps[0]}, {all_temps[1]}, {all_temps[2]} " + "with a lid temperature of " + f"{lid}")
    return all_temps