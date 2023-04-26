from os import stat
import clr
from pathlib import Path
import time as time 


dotnet_path = Path(__file__).resolve().parent / 'dotnet' / 'BiometraLibraryNet'
clr.AddReference(str(dotnet_path))
import BiometraLibrary 

class Functions:
    def __init__(self):
        self.settings =  BiometraLibrary.ApplicationClasses.ApplicationSettingsClasses.ApplicationSettings
        self.settings.LoadApplicationSettings()
        self.settings.CommunicationSettings.SerialComSettings.EnableCommunication = True 

        self.info = BiometraLibrary.DeviceExtComClasses.SystemClasses.InfoClasses.InfoCmds          
        self.device_desc = self.find_device()
        self.login_user()

        self.block_cmds = BiometraLibrary.DeviceExtComClasses.BlockClasses.BlockCmds(self.settings.CommunicationSettings, self.device_desc)
        self.block_n = BiometraLibrary.DeviceExtComClasses.BlockClasses.BlockDataClasses.BlockNumber(1)
        self.tcda_cmds = BiometraLibrary.DeviceExtComClasses.SystemClasses.TcdaClasses.TcdaCmds(self.settings.CommunicationSettings, self.device_desc)
        self.program_cmds = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgramCmds(self.settings.CommunicationSettings, self.device_desc)
        self.error_log_cmds = BiometraLibrary.DeviceExtComClasses.SystemClasses.ErrorLogClasses.ErrorLogCmds(self.settings.CommunicationSettings, self.device_desc)
        
        self.lid_state = self.get_lid_state()
        self.run_status = self.get_run_status()
        self.run_time_left = self.get_time_left()
        self.status_msg = ""
        
        
    def find_device(self):
        n, avail = self.info.GetAllComAvailableDevices()
        device_desc = avail.get_DataList().get_Item(0).DeviceInfos.DeviceDescriptionInfo
        return device_desc
    
    def login_user(self):
        login = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.LoginOutCmds(self.settings.CommunicationSettings,self.device_desc)
        self.user =  BiometraLibrary.DeviceExtComClasses.LoginOutClasses.UserClasses.UserDataClasses.UserInitials('ADM')
        self.passwd = BiometraLibrary.DeviceExtComClasses.LoginOutClasses.UserClasses.UserDataClasses.UserPassword('Admin')
        err = login.LoginUser(self.device_desc,self.user,self.passwd)
        return self.check_error(err)

    def check_error(self, err):
        if err:
            self.error = err
            print(err)
            self.status_msg = -1
        else:
            self.status_msg = 0
    #TODO: more sophisticated, maybe gets called from "get error" in main

    def get_status(self):
        err, status = self.tcda_cmds.GetBlockState(self.device_desc, self.block_n)
        status_str = str(status.BlockStateView)
        if status_str == "BLKSTATE_FREE":
            if self.get_lid_state == 0:
                return "READY"
        elif status_str == "BLKSTATE_RUN":
            return "BUSY"
        elif status_str == "BLKSTATE_ERROR":
            return "ERROR"
        elif status_str == "BLKSTATE_PREHEATING":
            return "BUSY"
        elif status_str == "BLKSTATE_PAUSE":
            print("PROGRAM IS PAUSED")
            return "BUSY"
        else:
            return "UNKNOWN"

    def create_program(self):
        # self.file = BiometraLibrary.FileClasses.FileWorkClasses.DeviceFileWorkClasses.ProgramFileWorker(self.fileInfo)
        self.programFileWorker = BiometraLibrary.FileClasses.FileWorkClasses.DeviceFileWorkClasses.ProgramFileWorker()
        self.checkStateResult = self.programFileWorker.ReadAllProgramTemplateInfosToShow(self.DataSetList)

        # Create new program
        self.pcrProgram = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.PcrProgram()
    

    def list_programs(self):
        err, program_list = self.program_cmds.GetProgramOverview(self.device_desc, self.user)
        print(program_list)

    def create_pcr_program():
        pass

    def run_pcr_program(self, prog):
        self.close_lid()
        prog_type = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgEditClasses.EnProgramType.TYPE_PROGRAM
        program_n = BiometraLibrary.DeviceExtComClasses.ProgClasses.ProgDataClasses.ProgEditClasses.ProgramNumber(prog, prog_type)
        err = self.block_cmds.StartProgramOnBlock(self.device_desc, self.user, program_n, self.block_n, True)
        time.sleep(5)
        self.run_status = self.get_run_status()
        self.check_error(err)

    def stop_program(self):
        err = self.block_cmds.StopProgramOnBlock(self.device_desc, self.block_n)
        self.check_error(err)
    
    def open_lid(self):
        err, status = self.tcda_cmds.GetMotLidState(self.device_desc, self.block_n)
        self.check_error(err)
        if status.CanOpenLid == True:
            print("Opening Lid")
            self.block_cmds.OpenMotLid(self.device_desc,self.block_n)
            time.sleep(5)
            while status.CanCloseLid == False:
                print("Opening Lid")
                err, status = self.tcda_cmds.GetMotLidState(self.device_desc, self.block_n)
                time.sleep(1)
        else:
            print("Lid already open")
        self.lid_state = self.get_lid_state()

    def close_lid(self):
        err, status = self.tcda_cmds.GetMotLidState(self.device_desc, self.block_n)
        self.check_error(err)
        if status.CanCloseLid == True:
            print("Closing Lid")
            self.block_cmds.CloseMotLid(self.device_desc,self.block_n)
            time.sleep(5)
            while status.CanOpenLid == False:
                print("Closing Lid")
                err, status = self.tcda_cmds.GetMotLidState(self.device_desc, self.block_n)
                time.sleep(1)
        else:
            print("Lid already closed")
        self.lid_state = self.get_lid_state()

    def plate_ready(self):
        '''
        determines whether or not a plate is ready to be removed from the biometra
        returns:
        0: ready to remove
        1: not ready
        -1: error
        '''
        # check if there is a protocol running on thermocycler
        run_status = self.get_run_status()
        if run_status == 1:
            print("Protocol still in progress, waiting...")
            return 1
        elif run_status == -1:
            print("Error in run")
            return -1
        elif run_status == 0:
            #no protocol in progress, check lid status
            lid_status = self.get_lid_state()
            if lid_status == 1:
                # lid is closed
                self.open_lid()
                return 1 # takes approx. 20 seconds to open lid
            elif lid_status == -1:
                print("lid is in motion")
                time.sleep(5)
                return 1
            elif lid_status == 0:
                # lid is open, make sure temperature is cool enough to retreive plate
                lid_temp = float(str(self.check_temp_lid())[:-2])
                right_temp = float(str(self.check_temp_right())[:-2])
                left_temp = float(str(self.check_temp_left())[:-2])
                mid_temp = float(str(self.check_temp_middle())[:-2])
                
                if lid_temp > 50:
                    print("Lid temp at " + f"{lid_temp} " + "deg C, cooling...")
                    return 1
                elif mid_temp > 50:
                    print("Mid block temp at " + f"{mid_temp} " + "deg C, cooling...")
                    return 1
                elif right_temp > 50:
                    print("Right block temp at " + f"{lid_temp} " + "deg C, cooling...")
                    return 1
                elif left_temp > 50:
                    print("Left block temp at " + f"{lid_temp} " + "deg C, cooling...")
                    return 1
                else:
                    print("Block cool, plate available")
                    return 0
            
            
            
        
    def get_run_status(self):
        '''
        determines the current run state of the device, returns:
        1: protool in progress
        0: no protocol in progress
        -1: unknown error
        '''
        
        err, status = self.tcda_cmds.GetBlockState(self.device_desc, self.block_n)
        self.check_error(err)
        if status.ActiveBlock == True:
            print("There is currently a protocol in progress on thermocycler " + f"{self.device_desc}")
            return 1
        elif status.ActiveBlock == False:
            print("There is currently no protocol in progress on thermocycler " + f"{self.device_desc}")
            return 0
        else:
            print("Unknown state of themrocycler " + f"{self.device_desc}")
            return -1
            
        
    def get_lid_state(self):
        '''
        determines current state of lid.
        returns:
        1: lid closed
        0: lid open
        -1: lid in motion
        '''
        err, status = self.tcda_cmds.GetMotLidState(self.device_desc, self.block_n)
        self.check_error(err)
        if str(status) == "0000 0000 0000 0011":
            print("The lid is open on thermocycler " + f"{self.device_desc}")
            self.lid_state = 0
            return 0
        elif str(status) == "0000 0000 0000 0101":
            print("The lid is closed on thermocycler " + f"{self.device_desc}")
            self.lid_state = 1
            return 1
        else:
            print("The lid is currently moving on thermocycler " + f"{self.device_desc}")
            self.lid_state = -1
            return -1

    def get_time_left(self):
        '''
        checks how much time is remaining in the current program
        returns:
        time: string of how much time is left: 00h 00m 00s
        '''
        # check to make sure there's a protocol underway
        err, time = self.tcda_cmds.GetRemainingTime(self.device_desc, self.block_n)
        self.check_error(err)
        print("There is " + f"{time} " "left on thermocycler " + f"{self.device_desc}")
        return time
    
    def countdown(self):
        """
        checks the time remaining in the protocol, and converts it into seconds to sleep until protocol completes 
        """
        hours = 0
        mins = 0
        secs = 0
        total_secs = 0
        time_left = self.get_time_left()
        print("waiting...")
        time_left = str(time_left)
        time_left = time_left.split(' ')
        for i in range(len(time_left)):
            time_left[i] = int(time_left[i][:-1])
        hours = time_left[0] * 3600
        mins = time_left[1] * 60
        secs = time_left[2]    
        total_secs = hours + mins + secs
        time.sleep(total_secs)
            
    def wait_until_ready(self) -> int:
        """
        Calls plate ready until the plate is made available

        Parameters
        ----------
        None
        
        """
        plate_status = 1
        while plate_status == 1:
            plate_status = self.plate_ready()
        
            if plate_status == -1:
                status = self.get_status()
                print(status)
                return -1
        if plate_status == 0:
            print("Plate is ready to be retreived")
            return 0

            
    def check_temp_left(self):
        '''
        checks the temperature on the left side of the block
        returns:
        temp (string)
        '''
        err, temp = self.tcda_cmds.GetBlockTempLeft(self.device_desc, self.block_n)
        self.check_error(err)
        # print("The left temprature of thermocycler " + f"{self.device_desc}" + " is " +f"{temp}")
        return temp

    def check_temp_right(self):
        '''
        checks the temperature on the right side of the block
        returns:
        temp (string)
        '''
        err, temp = self.tcda_cmds.GetBlockTempRight(self.device_desc, self.block_n)
        self.check_error(err)
        # print("The right temprature of thermocycler " + f"{self.device_desc}" + " is " +f"{temp}")
        return temp

    def check_temp_middle(self):
        '''
        checks the temperature on the middle of the block
        returns:
        temp (string)
        '''
        err, temp = self.tcda_cmds.GetBlockTempMiddle(self.device_desc, self.block_n)
        self.check_error(err)
        # print("The middle temprature of thermocycler " + f"{self.device_desc}" + " is " +f"{temp}")
        return temp
    
    def check_temp_lid(self):
        '''
        checks the temperature of the lid
        returns:
        temp (string)
        '''
        err, temp = self.tcda_cmds.GetHeatedLidTemp(self.device_desc, self.block_n)
        self.check_error(err)
        # print("The lid temprature of thermocycler " + f"{self.device_desc}" + " is " +f"{temp}")
        return temp

    def check_temp_all(self):
        '''
        returns the temperature of each side of the block and lid
        returns:
        temps: List[string]
        '''
        left = self.check_temp_left()
        right = self.check_temp_right()
        middle = self.check_temp_middle()
        all_temps = [left, middle, right]
        print("The combined tempratures of thermocycler " + f"{self.device_desc}" + " is " +f"{all_temps[0]}, {all_temps[1]}, {all_temps[2]}")
        return all_temps

if __name__ == "__main__":
    test = Functions()
    # test.lid_open()
    test.lid_close()

