import curses
import socket
import pygame, time

REVERSE = False

MW_HEYTI_LO = 0x33
MW_HEYTI_HI = 0x40
MW_TST_LO = 0x66
MW_TST_HI = 0x78
MW_EPC_LO = 0x77
MW_EPC_HI = 0x78
CW_CONN_TI_ON = 0x4E
CW_CONN_TI_OFF = 0x58
CW_GBSETPOS_ON = 0x71
CW_STSETPOS_ON = 0x73   
CW_GBSETPOS_OFF = 0x81
CW_STSETPOS_OFF = 0x83
CW_SETTIMEOUT_OFF = 0x8B
CW_SETTIMEOUT_ON  = 0x8C
CW_SETIOCHECK_OFF = 0x8D
CW_SETIOCHECK_ON  = 0x8E
CW_TX_POS = 0xC2
CW_TX_GBPOS = 0xA1
CW_TX_STPOS = 0xA3
CW_TX_WVVEL = 0xA5

cfg_cw_on= [CW_CONN_TI_ON, CW_SETIOCHECK_ON, CW_SETTIMEOUT_ON, CW_GBSETPOS_ON, 
            CW_STSETPOS_ON, 0, 0, 0, 0, 0, 0, 0]


own_host_name = socket.gethostname()
own_ip_addr = socket.gethostbyname(own_host_name)
peer_ip_addr = '192.168.0.145' #'169.254.30.68'
peer_ip_port = 52001
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#sock.bind((socket.gethostname(), 52002))

pre_st = 0;
pre_gb = 0;
                     
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
print 'Initialized Joystick : %s' % j.get_name()


MAX_RPM = 200


def get():
    out = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    it = 0 #iterator
    pygame.event.pump()
    
    #Read input from the two joysticks       
    for i in range(0, j.get_numaxes()):
        out[it] = j.get_axis(i)
        it+=1
    #Read input from buttons
    for i in range(0, j.get_numbuttons()):
        out[it] = j.get_button(i)
        it+=1
    return out
    

#---------------------------------------------------
#  set_connect
#---------------------------------------------------    
def set_connect(which):
    """ Send the test command to TI"""  
    print "TI Connect"
    cw = cfg_cw_on[which]
    b0 = MW_HEYTI_LO
    b1 = MW_HEYTI_HI
    b2 = cw
    b3 = 0
    datb = bytearray([b0, b1, b2, b3])
    sock.sendto(datb, (peer_ip_addr, peer_ip_port))
#---------------------------------------------------
#  send_epc_cmd
#---------------------------------------------------
def send_epc_cmd():
    """ Send the test command to TI"""

    b0 = MW_EPC_LO       
    b1 = MW_EPC_HI
    b2 = 1
    b3 = 0
    datb = bytearray([b0, b1, b2, b3])
    sock.sendto(datb, (peer_ip_addr, peer_ip_port))
#---------------------------------------------------
#  rqm_gb_ipSend
#---------------------------------------------------
def rqm_gb_ipSend(pos):
    global pre_gb
   # self.current_gb_pos = pos
    b0 = CW_TX_POS
    b1 = CW_TX_WVVEL
    b2 = (int)((pos    ) & 0xff)
    b3 = (int)((pos>> 8) & 0xff) 
    b4 = (int)((pre_st ) & 0xff)
    b5 = (int)((pre_st>>8) & 0xff)
    pre_gb = pos
    datb = bytearray([b0, b1, b2, b3, b4, b5])
    sock.sendto(datb, (peer_ip_addr, peer_ip_port))
#---------------------------------------------------
#  rqm_gb_move
#---------------------------------------------------
def rqm_gb_move(self, cmd):
    global sTestGBPos, sTestGBSTEP
    if cmd>50:
      sTestGBPos = sTestGBPos+sTestGBSTEP
      if sTestGBPos>960:
        sTestGBSTEP = -sTestGBSTEP
      elif sTestGBPos<-960:
        sTestGBSTEP = -sTestGBSTEP
      pos = sTestGBPos
    elif cmd>19:
      pos = rqm_gb_pTestPos[cmd-20]
    else:
      pos = rqm_st_pos[cmd-1]
    print("rqm_gb_move:", str(pos))
    self.text.config(state=NORMAL)
    self.text.insert("1.0","WL Move to:" + str(pos) +"\n")
    self.text.config(state=DISABLED)
    self.rqm_gb_ipSend(pos)
#---------------------------------------------------
#  rqm_st_ipSend
#---------------------------------------------------
def rqm_st_ipSend(pos):
    global pre_st
   # self.current_st_pos = pos
    b0 = CW_TX_POS
    b1 = CW_TX_WVVEL
    b2 = (int)((pre_gb ) & 0xff)
    b3 = (int)((pre_gb>>8) & 0xff)
    b4 = (int)((pos    ) & 0xff) 
    b5 = (int)((pos>> 8) & 0xff)
    pre_st = pos
    datb = bytearray([b0, b1, b2, b3, b4, b5])
    sock.sendto(datb, (peer_ip_addr, peer_ip_port))
#---------------------------------------------------
#  rqm_st_move
#---------------------------------------------------
def rqm_st_move(self, cmd):
    global sTestPos, sTestSTEP
    if cmd>50:
      sTestPos = sTestPos+sTestSTEP
      if sTestPos>960:
        sTestSTEP = -sTestSTEP
      elif sTestPos<-960:
        sTestSTEP = -sTestSTEP
      pos = sTestPos
    elif cmd>19:
      pos = rqm_st_pTestPos[cmd-20]
    else:
      pos = rqm_st_pos[cmd-1]
    print("rqm_st_move:", str(pos))
    self.text.config(state=NORMAL)
    self.text.insert("1.0","WR Move to:" + str(pos) +"\n")
    self.text.config(state=DISABLED)
    self.rqm_st_ipSend(pos)
    
    
    
if __name__ == "__main__":

    print("*****************")
    set_connect(0)
    set_connect(1)
    set_connect(3)
    set_connect(4)



    while True:
        time.sleep(0.1)
        val =  get()

        turn, go = val[0], val[3]  
        
        if abs(turn) < 0.01:
            wl = MAX_RPM * go * -1
            wr = MAX_RPM * go * -1
        elif turn < 0:
            wl = (go * MAX_RPM* -1) * abs(turn)
            wr = (go *MAX_RPM* -1) * (1 - abs(turn))
        
        elif turn > 0:
            wr = (go * MAX_RPM * -1) * abs(turn)
            wl = (go *MAX_RPM * -1) * (1 - abs(turn))
        
        print wl, wr

   
        if REVERSE:
            rqm_st_ipSend(int(-wr))
            rqm_gb_ipSend(int(-wl))
        else:
            rqm_st_ipSend(int(wl))
            rqm_gb_ipSend(int(wr))

    
## Run the thread and the GUI main loop


